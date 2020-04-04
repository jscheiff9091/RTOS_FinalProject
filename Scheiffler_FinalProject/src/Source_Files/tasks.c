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
#include "em_emu.h"

#include "bsp.h"
#include  <bsp_os.h>

#include <time.h>
#include <stdlib.h>
#include <stdio.h>

#include "display.h"
#include "textdisplay.h"
#include "retargettextdisplay.h"

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
OS_TMR ledToggleTmr;

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
	//Create intermittent LED warning timer
	OSTmrCreate(&ledToggleTmr,
				"LED Toggle Timer",
				TOGGLE_TIMEOUT,
				NO_DLY,
				OS_OPT_TMR_ONE_SHOT,
				&LEDToggleTmrCallback,
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


//	//Create Vehicle State Update task
//	OSTaskCreate(&vehStTaskTCB,
//				 "Vehicle State Update Task",
//				 VehicleStateTask,
//				 DEF_NULL,
//				 VEHST_TASK_PRIO,
//				 &vehStTaskStack[0],
//				 (VEHST_STACK_SIZE / 2u),
//				 VEHST_STACK_SIZE,
//				 0u,
//				 0u,
//				 DEF_NULL,
//				 OS_OPT_TASK_STK_CLR,
//				 &err);
//	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	//Create Game Monitor task
//	OSTaskCreate(&gmMonTaskTCB,
//				 "Game Monitor Task",
//				 GameMonitorTask,
//				 DEF_NULL,
//				 GMMON_TASK_PRIO,
//				 &gmMonTaskStack[0],
//				 (GMMON_STACK_SIZE / 2u),
//				 GMMON_STACK_SIZE,
//				 0u,
//				 0u,
//				 DEF_NULL,
//				 OS_OPT_TASK_STK_CLR,
//				 &err);
//	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


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
	time_t t;
	int xDiff;

	srand(time(&t));																				//Seed random number generator

	while(1) {
		fifoFull = false;
		while(!fifoFull) {
			OSMutexPend(&wayPtLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);						//Lock fifo
			if(road.waypoints.totalWayPts == 0 || road.waypoints.currWayPts >= FIFO_CAPACITY) {		//Check if waypoints need to be added
				fifoFull = true;																	//Fifo full or all waypoints for a level have been created
			}
			else {																					//More waypoints needed
				xDiff = GET_XDIFF;																	//Get the x-offset from the previous waypoint
				FIFO_Append(&road.waypoints, xDiff);												//Add new waypoint to the queue
			}
			OSMutexPost(&wayPtLock, OS_OPT_POST_NONE, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}
		OSTimeDly(250u, OS_OPT_TIME_DLY, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
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
			OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
			vehState.vehDir = localDir;
			OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

//			if(dirFlags == DIR_FLG_NONE) {												//No flags currently set, do not need to clear them
//				OSFlagPost(&dirChngFlags, (1 << localDir), OS_OPT_POST_FLAG_SET, &err);	//Direction flag set
//				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//			}
//			else {																		//Direction flag outdated, clear and update
//				OSFlagPost(&dirChngFlags, dirFlags, OS_OPT_POST_FLAG_CLR, &err);		//Current direction flag cleared
//				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//
//				OSFlagPost(&dirChngFlags, (1 << localDir), OS_OPT_POST_FLAG_SET, &err);	//Direction flag updated to new direction
//				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//			}
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
		cpAccelSd = (cpVel*cpVel) / vehSpecs.turnRadius;	//Calculate sideways acceleration
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

			while(!(GPIO_PortInGet(BTN0_PORT) & (1 << BTN0_PIN))) {								//While button 0 is being held down
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

				OSTimeDly(100u, OS_OPT_TIME_DLY, &err);							//Give player time to release button
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

			while(!(GPIO_PortInGet(BTN1_PORT) & (1 << BTN1_PIN))) {								//While button 0 is being held down
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

				OSTimeDly(100u, OS_OPT_TIME_DLY, &err);							//Give player time to release button
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
	OS_FLAGS warnFlags;							//Looking for warnings
	bool slipWarn, headWarn = false;			//Warning state variables

	while(1) {
		//Wait for LED event
		if(!slipWarn && !headWarn) {
			ledEventFlags = OSFlagPend(&ledWarnFlags, LED_WARN_ANY, 0, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_BLOCKING, &timestamp, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}
		else if(!slipWarn && headWarn) {
			ledEventFlags = OSFlagPend(&ledWarnFlags, ANY_EX_HEAD, 0, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_BLOCKING, &timestamp, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}
		else if(slipWarn && !headWarn) {
			ledEventFlags = OSFlagPend(&ledWarnFlags, ANY_EX_SLIP, 0, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_BLOCKING, &timestamp, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}
		else {
			ledEventFlags = OSFlagPend(&ledWarnFlags, ANY_EX_WARN, 0, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_BLOCKING, &timestamp, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}
		//Clear current event flags
		OSFlagPost(&ledWarnFlags, ledEventFlags, OS_OPT_POST_FLAG_CLR, &err);				//Clear flags
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		//Check if warning flags cleared
		warnFlags = OSFlagPendGetFlagsRdy(&err);
		if(slipWarn && !(warnFlags & TIRE_SLIP_WARN)) {
			slipWarn = false;
			GPIO_PinOutClear(LED0_PORT, LED0_PIN);
		}
		if(headWarn && !(warnFlags & VEH_HEAD_WARN)) {
			headWarn = false;
			GPIO_PinOutClear(LED1_PORT, LED1_PIN);
		}

		//Check speed warning light events
		if(!slipWarn && (ledEventFlags & TIRE_SLIP_WARN)){
			GPIO_PinOutSet(LED0_PORT, LED0_PIN);
		}
		if(ledEventFlags & TIRE_OFF_ROAD) {
			GPIO_PinOutSet(LED0_PORT, LED0_PIN);
		}
		if(!headWarn && (ledEventFlags & VEH_HEAD_WARN)) {
			GPIO_PinOutSet(LED1_PORT, LED1_PIN);
		}
		if(ledEventFlags & VEH_OFF_ROAD) {
			GPIO_PinOutSet(LED1_PORT, LED1_PIN);
		}
		if(ledEventFlags & TOGGLE_WARN_LED) {
			if(slipWarn) {
				GPIO_PinOutToggle(LED0_PORT, LED0_PIN);
			}
			if(headWarn) {
				GPIO_PinOutToggle(LED1_PORT, LED1_PIN);
			}
			OSTmrStart(&ledToggleTmr, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}
	}
}

/* LCD Display Task */
void LCDDisplayTask(void* p_args) {
	RTOS_ERR err;
	CPU_TS timestamp;

	char* dirStr[] = DIRECTION_STRINGS;
	char buffer[10];
	Direction_t dir;
	uint16_t speed;
	int numWaypoints;
	int x;
	int y;
	struct WayPt_t* head;

	//Initialize the display
	DISPLAY_Init();

	if (RETARGET_TextDisplayInit() != TEXTDISPLAY_EMSTATUS_OK) {
		while (1);
	}

	while(1) {
		//Copy Vehicle state information
		OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		dir = vehState.vehDir;
		speed = vehState.speed;
		OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		//Copy Waypoint and fifo information
		OSMutexPend(&wayPtLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		numWaypoints = road.waypoints.currWayPts;
		head = FIFO_Peek(&road.waypoints);
		x = head->xPos;
		y = head->yPos;
		FIFO_Pop(&road.waypoints);
		OSMutexPost(&wayPtLock, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		printf("\f");
		//Print direction
		printf("Direction: %s\n", dirStr[dir]);
		//Print speed
		itoa(speed, buffer, 10);
		printf("Speed: %s\n", buffer);

		//Print waypoint data
		itoa(numWaypoints, buffer, 10);
		printf("Num of Way Pts: %s\n", buffer);
		itoa(x, buffer, 10);
		printf("X Pos Way Pt: %s\n", buffer);
		itoa(y, buffer, 10);
		printf("Y Pos Way Pt: %s\n", buffer);

		OSTimeDly(3000u, OS_OPT_TIME_DLY, &err);
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

