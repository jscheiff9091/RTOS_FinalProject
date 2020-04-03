/*
 * tasks.c
 *
 *  Created on: Mar 30, 2020
 *      Author: Jacob S
 */

#include "tasks.h"
#include "game.h"
#include "slider.h"
#include "lcd.h"
#include "gpio.h"
#include "em_gpio.h"

#include "math.h"

#include  <cpu/include/cpu.h>
#include  <common/include/common.h>
#include  <kernel/include/os.h>
#include  <kernel/include/os_trace.h>

#include  <common/include/lib_def.h>
#include  <common/include/rtos_utils.h>
#include  <common/include/toolchains.h>


// RTOS Global variables
// Task Control Blocks
OS_TCB startTaskTCB;
OS_TCB idleTaskTCB;
OS_TCB rdGenTaskTCB;
OS_TCB dirTaskTCB;
OS_TCB physModTaskTCB;
OS_TCB spdTaskTCB;
OS_TCB ledTaskTCB;
OS_TCB vehStTaskTCB;
OS_TCB gmMonTaskTCB;
OS_TCB lcdTaskTCB;

//Task stacks
CPU_STK startTaskStack[START_STACK_SIZE];
CPU_STK idleTaskStack[IDLE_STACK_SIZE];
CPU_STK rdGenTaskStack[RDGEN_STACK_SIZE];
CPU_STK dirTaskStack[DIR_STACK_SIZE];
CPU_STK physModTaskStack[PHYSMOD_STACK_SIZE];
CPU_STK	spdTaskStack[SPD_STACK_SIZE];
CPU_STK	ledTaskStack[LED_STACK_SIZE];
CPU_STK vehStTaskStack[VEHST_STACK_SIZE];
CPU_STK gmMonTaskStack[GMMON_STACK_SIZE];
CPU_STK lcdTaskStack[LCD_STACK_SIZE];

//Flag Groups
OS_FLAG_GRP dirChngFlags;
OS_FLAG_GRP ledWarnFlags;
OS_FLAG_GRP lcdFlags;
OS_FLAG_GRP	btnEventFlags;
//OS_FLAG_GRP gmSetupFlags;

//Semaphores
OS_SEM physModSem;
OS_SEM gmMonSem;
OS_SEM spdUpdateSem;
//OS_SEM gmModeSem;

//Timers
OS_TMR dirTmr;
OS_TMR rdGenTmr;

//Mutexes
OS_MUTEX vehStLock;
OS_MUTEX physTupLk;
OS_MUTEX wayPtLock;

/* Start Task */
void StartTask(void* p_arg) {
    RTOS_ERR  err;

    PP_UNUSED_PARAM(p_arg);       				//Prevent compiler warning.

    BSP_TickInit();                           	//Initialize Kernel tick source.
	Common_Init(&err);                          // Call common module initialization example.
	APP_RTOS_ASSERT_CRITICAL(err.Code == RTOS_ERR_NONE, ;);
	BSP_OS_Init();								//Initialize the kernel

#if defined(uCProbe)
	/* Call these functions to set up uC Probe */
	CPU_TS_TmrInit();							//Initialize timestamp source

	OSStatTaskCPUUsageInit(&err);   			//Call function to initialize stat task
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	CPU_IntDisMeasMaxCurReset();				//Return something about interrupt timing

	OSStatReset(&err);							//Reset Stats? Also said to call this in Appendix B
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
#endif


	/**** Create all semaphores used *****/
	//Create semaphore used to signal that the physics model needs to be updated
	OSSemCreate(&physModSem, "Physics Model Signal Semaphore", CNT_ZERO, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	//Create semaphore used to signal the Vehicle state has updated and the game monitor task has to update the game status
	OSSemCreate(&gmMonSem, "Game Monitor Task Signal Semaphore", CNT_ZERO, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	//Create semaphore used to signal that speed state variable has been updated
	OSSemCreate(&spdUpdateSem, "Speed Update Signal Semaphore", CNT_ZERO, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	/**** Create all Flag groups used ****/
	//Create event flag group to communicate which vehicle state has been updated
	OSFlagCreate(&dirChngFlags, "Direction Change Event Flags", DIR_FLG_CLR, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	//Create the event flag to notify the LED Driver task of LED State change
	OSFlagCreate(&ledWarnFlags, "Vehicle Warning Event Flag", LED_WARN_CLR_FLAGS, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	//Create the event flag to notify that one of the buttons has been pressed
	OSFlagCreate(&btnEventFlags, "Button Event Flags", BTN_FLG_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);


	/**** Create all mutexs used *****/
	//Create mutex to protect the speed setpoint data
	OSMutexCreate(&vehStLock, "Vehicle State Data Mutex", &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	//Create mutex to protect the physics tuple
	OSMutexCreate(&physTupLk, "Physics Tuple Mutex", &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	//Create mutex to protect waypoint FIFO
	OSMutexCreate(&wayPtLock, "Waypoint FIFO Mutex", &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);


	/**** Create all timers used ****/
	//Create the timer to schedule the vehicle direction task
	OSTmrCreate(&dirTmr,
				"Direction Update Task Timer",
				NO_DLY,
				DIR_TMR_CNT,
				OS_OPT_TMR_PERIODIC,
				&SLD_DirTimerCallback,
				DEF_NULL,
				&err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	//Create vehicle turning monitor timeout timer
	OSTmrCreate(&rdGenTmr,
				"Vehicle Turn Timeout Timer",
				NO_DLY,
				RDGEN_TMR_CNT,
				OS_OPT_TMR_ONE_SHOT,
				&Game_RoadGenerationCallback,
				DEF_NULL,
				&err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);


	/**** Create all tasks ******/
	//Create the road generation task
	OSTaskCreate(&rdGenTaskTCB,
				 "Road Generation Task",
				 RoadGenerateTask,
				 DEF_NULL,
				 RDGEN_TASK_PRIO,
				 &rdGenTaskStack[0],
				 (RDGEN_STACK_SIZE / 2u),
				 RDGEN_STACK_SIZE,
				 0u,
				 0u,
				 DEF_NULL,
				 OS_OPT_TASK_STK_CLR,
				 &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	//Create slider input task
	OSTaskCreate(&dirTaskTCB,
				 "Direction Update Task",
				 DirectionUpdateTask,
				 DEF_NULL,
				 DIR_TASK_PRIO,
				 &dirTaskStack[0],
				 (DIR_STACK_SIZE / 2u),
				 DIR_STACK_SIZE,
				 0u,
				 0u,
				 DEF_NULL,
				 OS_OPT_TASK_STK_CLR,
				 &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	//Create the road generation task
	OSTaskCreate(&physModTaskTCB,
				 "Physics Model Update Task",
				 PhysicsModelTask,
				 DEF_NULL,
				 RDGEN_TASK_PRIO,
				 &physModTaskStack[0],
				 (PHYSMOD_STACK_SIZE / 2u),
				 PHYSMOD_STACK_SIZE,
				 0u,
				 0u,
				 DEF_NULL,
				 OS_OPT_TASK_STK_CLR,
				 &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	//Create button input task
	OSTaskCreate(&spdTaskTCB,
				 "Speed Update Task",
				 SpeedUpdateTask,
				 DEF_NULL,
				 SPD_TASK_PRIO,
				 &spdTaskStack[0],
				 (SPD_STACK_SIZE / 2u),
				 SPD_STACK_SIZE,
				 0u,
				 0u,
				 DEF_NULL,
				 OS_OPT_TASK_STK_CLR,
				 &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	//Create LED Warning task
	OSTaskCreate(&ledTaskTCB,
				 "LED Warning Task",
				 LEDWarningTask,
				 DEF_NULL,
				 LED_TASK_PRIO,
				 &ledTaskStack[0],
				 (LED_STACK_SIZE / 2u),
				 LED_STACK_SIZE,
				 0u,
				 0u,
				 DEF_NULL,
				 OS_OPT_TASK_STK_CLR,
				 &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


	//Create Vehicle State Update task
	OSTaskCreate(&vehStTaskTCB,
				 "Vehicle State Update Task",
				 VehicleStateTask,
				 DEF_NULL,
				 VEHST_TASK_PRIO,
				 &vehStTaskStack[0],
				 (VEHST_STACK_SIZE / 2u),
				 VEHST_STACK_SIZE,
				 0u,
				 0u,
				 DEF_NULL,
				 OS_OPT_TASK_STK_CLR,
				 &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	//Create Game Monitor task
	OSTaskCreate(&gmMonTaskTCB,
				 "Game Monitor Task",
				 GameMonitorTask,
				 DEF_NULL,
				 GMMON_TASK_PRIO,
				 &gmMonTaskStack[0],
				 (GMMON_STACK_SIZE / 2u),
				 GMMON_STACK_SIZE,
				 0u,
				 0u,
				 DEF_NULL,
				 OS_OPT_TASK_STK_CLR,
				 &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


	//Create LCD Display Task
	OSTaskCreate(&lcdTaskTCB,
				 "LCD Display Task",
				 LCDDisplayTask,
				 DEF_NULL,
				 LCD_TASK_PRIO,
				 &lcdTaskStack[0],
				 (LCD_STACK_SIZE / 2u),
				 LCD_STACK_SIZE,
				 0u,
				 0u,
				 DEF_NULL,
				 OS_OPT_TASK_STK_CLR,
				 &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

    __enable_irq();						//Global Enable Interrupts

    //Create Idle task
    OSTaskCreate(&idleTaskTCB,
				 "Idle Task",
				 IdleTask,
				 DEF_NULL,
				 IDLE_TASK_PRIO,
				 &idleTaskStack[0],
				 (IDLE_STACK_SIZE / 2u),
				 IDLE_STACK_SIZE,
				 0u,
				 0u,
				 DEF_NULL,
				 OS_OPT_TASK_STK_CLR,
				 &err);

	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	while(1);
}

/* Waypoint generator task */
void RoadGenerateTask(void* p_args) {
	RTOS_ERR err;
	CPU_TS timestamp;

	bool fifoFull;

	while(1) {
		fifoFull = false;
		while(!fifoFull) {
			OSMutexPend(&wayPtLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
			if(road.waypoints.totalWayPts == 0 || road.waypoints.currWayPts >= FIFO_CAPACITY) {
				fifoFull = true;
			}
			else {

			}
			OSMutexPost(&wayPtLock, OS_OPT_POST_NONE, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}
	}
}

/* Vehicle Direction Task */
void DirectionUpdateTask(void * p_args) {

	RTOS_ERR err;																		//Create RTOS variables
	CPU_TS timestamp;
	OS_FLAGS dirFlags;

	SLD_Init();       																	//Initialize CAPSENSE driver and set initial slider state

	Direction_t prevDir = Straight; 													//Assume default is straight,
	Direction_t localDir;

	while(1) {
		localDir = SLD_GetDirection();                                  				//Get current direction
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

		if(prevDir != localDir) {
			dirFlags = OSFlagPendGetFlagsRdy(&err);

			if(dirFlags == DIR_FLG_NONE) {												//No flags currently set, do not need to clear them
				OSFlagPost(&dirChngFlags, (1 << localDir), OS_OPT_POST_FLAG_SET, &err);	//Direction flag set
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
			}
			else {																		//Direction flag outdated, clear and update
				OSFlagPost(&dirChngFlags, dirFlags, OS_OPT_POST_FLAG_CLR, &err);		//Current direction flag cleared
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

				OSFlagPost(&dirChngFlags, (1 << localDir), OS_OPT_POST_FLAG_SET, &err);	//Direction flag updated to new direction
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
			}
		}
		prevDir = localDir;																//Update local direction variable

		OSTimeDly(100u, OS_OPT_TIME_DLY, &err);											//Wait 100ms
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
	}
}

/* Physics Model Update Task */
void PhysicsModelTask(void* p_args) {
	RTOS_ERR err;
	CPU_TS timestamp;

	uint16_t cpAccelSd;
	uint16_t cpAccelFwd;
	uint16_t cpVel;
	uint16_t cpPower;

	while(1) {
		OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);		//Make copies of the current data in the tuple
		cpAccelFwd = vehPhys.accelFwd;
		cpAccelSd = vehPhys.accelSd;
		cpVel = vehPhys.velocity;
		cpPower = vehPhys.power;
		OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

		cpAccelFwd = cpPower / cpVel;											//Calculate forward acceleration
		cpAccelSd = pow(cpVel, 2) / vehSpecs.turnRadius;						//Calculate sideways acceleration
		cpVel += cpAccelFwd * PHYS_UPDATE_RATE;									//Calculate vehicle velocity
		cpPower += cpVel * PHYS_UPDATE_RATE;									//Calculate vehicle power

		OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);		//Update values in the physics in the tuple
		vehPhys.accelFwd = cpAccelFwd;
		vehPhys.accelSd = cpAccelSd;
		vehPhys.velocity = cpVel;
		vehPhys.power = cpPower;
		OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

		OSTimeDly(PHYS_UPDATE_RATE * 1000, OS_OPT_TIME_DLY, &err);				//Pause for physics update period
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
	}
}

/* Button Input Task */
void SpeedUpdateTask(void* p_args) {

	RTOS_ERR err;
	CPU_TS timestamp;

	OS_FLAGS btnFlags;
	int localSpd = 0;

	GPIO_InitBTNs();															//Enable push buttons

	while(1) {
		// Wait to be signaled by button ISR
		btnFlags = OSFlagPend(&btnEventFlags, BTN_FLG_ANY, 0, (OS_OPT_PEND_BLOCKING | OS_OPT_PEND_FLAG_SET_ANY), &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		if(btnFlags & BTN0_PRESS) {												//Button 0 pressed
			localSpd += localSpd * .05;											//Increase speed
			if(localSpd >= 100) {
				localSpd = 100;
			}
			else if(localSpd < 20) {
				localSpd = 20;
			}

			OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
			vehState.speed = localSpd;											//Update speed
			OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

			OSSemPost(&spdUpdateSem, OS_OPT_POST_1, &err);						//Signal to vehicle state task that speed has been updated
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

			while(!(GPIO_PortInGet & BTN0_PIN)) {								//While button 0 is being held down
				localSpd += localSpd * .05;										//Increase speed
				if(localSpd >= 100) {
					localSpd = 100;
				}
				else if(localSpd < 20) {
					localSpd = 20;
				}

				OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
				vehState.speed = localSpd;										//Update speed
				OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

				OSSemPost(&spdUpdateSem, OS_OPT_POST_1, &err);					//Signal to vehicle state task that speed has been updated
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

				OSTimeDly(10u, OS_OPT_TIME_DLY, &err);							//Give player time to release button
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			}
		}
		else if(btnFlags & BTN1_PRESS) {										//Button 1 pressed
			localSpd -= localSpd * .05;											//Increase speed
			if(localSpd < 20) {													//Speed too low, stop car entirely
				localSpd = 0;
			}

			OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
			vehState.speed = localSpd;											//Update speed
			OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

			OSSemPost(&spdUpdateSem, OS_OPT_POST_1, &err);						//Signal to vehicle state task that speed has been updated
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

			while(!(GPIO_PortInGet & BTN1_PIN)) {								//While button 0 is being held down
				localSpd -= localSpd * .05;										//Increase speed
				if(localSpd <= 20) {											//If speed gets to 20 stop car
					localSpd = 0;
				}

				OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
				vehState.speed = localSpd;										//Update speed
				OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

				OSSemPost(&spdUpdateSem, OS_OPT_POST_1, &err);					//Signal to vehicle state task that speed has been updated
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

				OSTimeDly(10u, OS_OPT_TIME_DLY, &err);							//Give player time to release button
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			}
		}

		OSFlagPost(&btnEventFlags, btnFlags, OS_OPT_POST_FLAG_CLR, &err);		//Clear button event flag
	}
}

/* LED Warning Task */
void LEDWarningTask(void* p_args) {

	RTOS_ERR err;
	CPU_TS timestamp;
	PP_UNUSED_PARAM(p_args);       				//Prevent compiler warning.

	GPIO_InitLEDs();							//Enable LEDs
	OS_FLAGS ledEventFlags;						//Event flags

	while(1) {
		//Wait for LED event
		ledEventFlags = OSFlagPend(&LEDDriverEvent, LED_WARN_ALL, 0, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		OSFlagPost(&LEDDriverEvent, ledEventFlags, OS_OPT_POST_FLAG_CLR, &err);				//Clear flags
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		//Check speed warning light events
		if(ledEventFlags & LED_WARN_SPD_VIOLATION){
			GPIO_PinOutSet(LED0_PORT, LED0_PIN);
		}
		else if(ledEventFlags & LED_WARN_CLR_SPD_VIOLATION) {
			GPIO_PinOutClear(LED0_PORT, LED0_PIN);
		}

		//Check turn warning light flags
		if(ledEventFlags & LED_WARN_TRN_VIOLATION) {
			GPIO_PinOutSet(LED1_PORT, LED1_PIN);
		}
		else if(ledEventFlags & LED_WARN_CLR_TRN_VIOLATION) {
			GPIO_PinOutClear(LED1_PORT, LED1_PIN);
		}
	}
}

/* LCD Display Task */
void LCDDisplayTask(void* p_args) {
	RTOS_ERR err;
	CPU_TS timestamp;
	char* dirStr[] = DIRECTION_STRINGS;
	char buffer[15];
	int currSpeed;
	Direction_t currDir;
	bool spdChng, dirChng = false;

	//Initialize the display
	DISPLAY_Init();

	if (RETARGET_TextDisplayInit() != TEXTDISPLAY_EMSTATUS_OK) {
	while (1) ;
	}

	// Initialize local copy of the speed
	OSMutexPend(&setptDataMutex, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	currSpeed = setptData.speed;
	OSMutexPost(&setptDataMutex, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	//Initialize local copy of direction
	OSMutexPend(&vehDirMutex, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	currDir = vehicleDir.dir;
	OSMutexPost(&vehDirMutex, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	//Print initial direction
	itoa(currSpeed, buffer, 10);
	printf("Direction: %s\nSpeed: %s", dirStr[currDir], buffer);

	//Start Task timer
//	OSTmrStart(&LCDDispTmr, &err);
//	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
//
//	//Wait for task timer to expire
//	OSTaskSuspend(&LCDDispTaskTCB, &err);
//	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	while(1) {
		//Update local copy of the speed
		OSMutexPend(&setptDataMutex, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
		if(currSpeed != setptData.speed) {
			currSpeed = setptData.speed;
			spdChng = true;
		}
		OSMutexPost(&setptDataMutex, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		//Update local copy of the direction
		OSMutexPend(&vehDirMutex, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
		if(currDir != vehicleDir.dir) {
			currDir = vehicleDir.dir;
			dirChng = true;
		}
		OSMutexPost(&vehDirMutex, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		//Update display if necessary
		if(spdChng || dirChng) {
			printf("\f");
			itoa(currSpeed, buffer, 10);
			printf("Direction: %s\nSpeed: %s", dirStr[currDir], buffer);
			spdChng = false;
			dirChng = false;
		}

//		//Wait for next iteration
//		OSTaskSuspend(&LCDDispTaskTCB, &err);
//		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		OSTimeDly(100u, OS_OPT_TIME_DLY, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	}
}

/* Idle Task */
void IdleTask(void* p_args) {

	RTOS_ERR err;

	OSTaskDel(&startTaskTCB, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	while(1) {
		EMU_EnterEM1();
	}
}

//void VehicleMonitorTask(void* p_args) {
//	RTOS_ERR err;
//	CPU_TS timestamp;
//	OS_FLAGS flags;
//
//	bool speedWarn = false;		//LED Currently signaling a spped warning
//	bool turnWarn = false;		//LED Currently signaling a turn warning
//	Direction_t currDir = Straight;
//	int currSpeed = 40;
//
//	//Start the timer
//	OSTmrStart(&vehTurnTimeout, &err);
//	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//
//	while(1) {
//		//Wait for speed change, direction change, or hard left/right timeout
//		flags = OSFlagPend(&vehMonFlags, VEH_MON_SET_FLAGS, 0, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_BLOCKING, &timestamp, &err);
//		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//		OSFlagPost(&vehMonFlags, flags, OS_OPT_POST_FLAG_CLR, &err);
//
//		if(flags & SPD_SETPT_FLAG) {																		//Speed change occurred
//			//Critical section
//			OSMutexPend(&setptDataMutex, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
//			currSpeed = setptData.speed;
//			OSMutexPost(&setptDataMutex, OS_OPT_POST_NONE, &err);
//			if(currSpeed > 70 && !speedWarn) {														//Send Speed violation if speed is greater than 70 for any drection
//				OSFlagPost(&LEDDriverEvent, LED_WARN_SPD_VIOLATION, OS_OPT_POST_FLAG_SET, &err);
//				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//				speedWarn = true;
//			}
//			else if(currSpeed > 50 && !speedWarn && currDir != Straight) {							//Send speed warning if speed is greater than 50 and the drive is turning
//				OSFlagPost(&LEDDriverEvent, LED_WARN_SPD_VIOLATION, OS_OPT_POST_FLAG_SET, &err);
//				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//				speedWarn = true;
//			}
//			else if(setptData.speed < 75 && speedWarn && currDir == Straight) {						//Clear speed violation if driver not turning and speed less than 75
//				OSFlagPost(&LEDDriverEvent, LED_WARN_CLR_SPD_VIOLATION, OS_OPT_POST_FLAG_SET, &err);
//				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//				speedWarn = false;
//			}
//			else if(setptData.speed < 55 && speedWarn) {											//Clear speed violation if speed is less than 55 for any turn direction
//				OSFlagPost(&LEDDriverEvent, LED_WARN_CLR_SPD_VIOLATION, OS_OPT_POST_FLAG_SET, &err);
//				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//				speedWarn = false;
//			}
//		}
//
//		if(flags & VEH_DIR_FLAG) {																			//Vehicle direction changed
//			OSMutexPend(&vehDirMutex, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
//			currDir = vehicleDir.dir;																		//Get Current direction
//			OSMutexPost(&vehDirMutex, OS_OPT_POST_NONE, &err);
//			if(turnWarn) {																					//turn off warning
//				turnWarn = false;
//				OSFlagPost(&LEDDriverEvent, LED_WARN_CLR_TRN_VIOLATION, OS_OPT_POST_FLAG_SET, &err);		//Signal LED task to turn off turn warning light
//				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//
//			}
//			else {																							//Only need to stop timer if it hasn't already expired
//				OSTmrStop(&vehTurnTimeout, OS_OPT_TMR_NONE, DEF_NULL, &err);								//Restart the timer
//				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//			}
//			OSTmrStart(&vehTurnTimeout, &err);
//			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//
//
//			if(currSpeed > 70 && !speedWarn) {															//Speed greater than or equal to 75, speed violation
//				OSFlagPost(&LEDDriverEvent, LED_WARN_SPD_VIOLATION, OS_OPT_POST_FLAG_SET, &err);
//				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//				speedWarn = true;
//			}
//			else if(currSpeed < 75 && speedWarn && currDir == Straight) {								//Speed less than or equal to 70, and direction straight clear speed violation
//				OSFlagPost(&LEDDriverEvent, LED_WARN_CLR_SPD_VIOLATION, OS_OPT_POST_FLAG_SET, &err);
//				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//				speedWarn = false;
//			}
//
//			else if(currSpeed > 50 && !speedWarn && currDir != Straight) {								//Speed greater than or equal to 55 and direction is not straigt
//				OSFlagPost(&LEDDriverEvent, LED_WARN_SPD_VIOLATION, OS_OPT_POST_FLAG_SET, &err);
//				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//				speedWarn = true;
//			}
//			else if(currSpeed < 55 && speedWarn) {														//Any speed less than 55, no speed violation
//				OSFlagPost(&LEDDriverEvent, LED_WARN_CLR_SPD_VIOLATION, OS_OPT_POST_FLAG_SET, &err);
//				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//				speedWarn = false;
//			}
//		}
//
//		if(flags & VEH_TURNTM_FLAG) {																		//Hard left/right timeout expired
//			turnWarn = true;																				//Set turn warning state variable
//			OSFlagPost(&LEDDriverEvent, LED_WARN_TRN_VIOLATION, OS_OPT_POST_FLAG_SET, &err);				//Signal LED task to turn on turn warning light
//			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//		}
//	}
//}
//
//void VehicleTurnTimeout(void* tmr, void* p_args) {
//	RTOS_ERR err;
//
//	OSFlagPost(&vehMonFlags, VEH_TURNTM_FLAG, OS_OPT_POST_FLAG_SET, &err);	//Vehicle turn hard left/right timeout, notify Vehicle Monitor task
//	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//}

