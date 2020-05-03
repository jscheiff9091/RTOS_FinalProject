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
#include <string.h>
#include <stdbool.h>

#include "display.h"
#include "glib.h"
#include "dmd.h"

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
OS_SEM menuUpdateSem;

//Timers
OS_TMR ledToggleTmr;

//Mutexes
OS_MUTEX vehStLock;
OS_MUTEX physTupLk;
OS_MUTEX wayPtLock;
OS_MUTEX usedRdLock;
OS_MUTEX gameStLock;

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

	//Create semaphore used to signal to the LCD task that the menu needs to be updated
	OSSemCreate(&menuUpdateSem, "LCD Update Menu Signal Semaphore", CNT_ZERO, &err);
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

	//Create flags for to signal LCD task
	OSFlagCreate(&lcdFlags, "LCD Update Signal Flags", BTN_FLG_NONE, &err);
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

	//Create mutex to protect game state variable
	OSMutexCreate(&gameStLock, "Game State Variable Mutex", &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	//Create mutex to protect waypoint FIFO
	OSMutexCreate(&usedRdLock, "Used Road Waypoints FIFO Mutex", &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	ResetStateVars();

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
				xDiff = GET_XDIFF((selections.difficulty + 1));										//Get the x-offset from the previous waypoint
				FIFO_Append(&road.waypoints, xDiff, USE_X_DIFF);									//Add new waypoint to the queue
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


			OSFlagPost(&dirChngFlags, (1 << localDir), OS_OPT_POST_FLAG_SET, &err);	//Direction flag updated to new direction
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
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

	double cpAccelSd, cpAccelFwd;
	uint16_t cpPower, cpRad;
	double cpVel;

	while(1) {
		OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);		//Make copies of the current data in the tuple
		cpAccelFwd = vehPhys.accelFwd;
		cpAccelSd = vehPhys.accelSd;
		cpVel = vehPhys.velocity;
		cpPower = vehPhys.power;
		OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

		if(cpPower > 0 && cpVel < 4) {
			cpAccelFwd = 5;
		}
		else if(cpPower == 0 && cpVel != 0) {									//Slowing down with no power input
			cpAccelFwd = -5;
		}
		else if(cpVel != 0){
			cpAccelFwd = ((double)cpPower) / cpVel;										//Calculate forward acceleration
		}

		OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);		//Get copy of current turn radius
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
		cpRad = vehState.radius;
		OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

		if(cpRad != 0) {
			cpAccelSd = (cpVel*cpVel) / vehSpecs.turnRadius;					//Calculate sideways acceleration
		}
		else {
			cpAccelSd = 0;
		}
		cpVel += cpAccelFwd * PHYS_UPDATE_RATE;									//Calculate vehicle velocity
		if(cpVel < 0) {															//No reverse functionality
			cpVel = 0;
		}

		OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);		//Update values in the physics in the tuple
		vehPhys.accelFwd = cpAccelFwd;
		vehPhys.accelSd = cpAccelSd;
		vehPhys.velocity = cpVel;
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
	GameState_t tempState;
	double accel = 0;
	bool firstPress = true;

	GPIO_InitBTNs();															//Enable push buttons
	OSFlagPost(&btnEventFlags, BTN_FLG_ANY, OS_OPT_POST_FLAG_CLR, &err);

	while(1) {
		// Wait to be signaled by button ISR
		btnFlags = OSFlagPend(&btnEventFlags, BTN_FLG_ANY, 0, (OS_OPT_PEND_BLOCKING | OS_OPT_PEND_FLAG_SET_ANY), &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		OSMutexPend(&gameStLock, 0u, OS_OPT_PEND_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		tempState = gameState;																//Update local copy of variable
		OSMutexPost(&gameStLock, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		if(tempState == GamePlay){
			if(btnFlags & BTN0_PRESS) {														//Button 0 pressed
				if(selections.gameMode == TimeTrial && firstPress) {
					gameStats.startTime = OSTimeGet(&err);
					APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
					firstPress = false;
				}
				if(accel < 1) { 															//Open throttle by 5%
					accel += .05;
				}

				OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				vehPhys.power = (uint16_t) (accel * vehSpecs.maxPower);
				OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

				while(!(GPIO_PortInGet(BTN0_PORT) & (1 << BTN0_PIN))) {						//While button 0 is being held down
					if(accel < 1) { 														//Open throttle by 5%
						accel += .05;
					}

					OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
					APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
					vehPhys.power = (uint16_t) (accel * vehSpecs.maxPower);
					OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
					APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

					OSTimeDly(150u, OS_OPT_TIME_DLY, &err);									//Give player time to release button
					APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				}
			}
			else if(btnFlags & BTN1_PRESS) {												//Button 1 pressed
				if(accel > 0) {																//Close throttle by 5%
					accel -= .05;
				}

				OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				vehPhys.power = (uint16_t) (accel * vehSpecs.maxPower);
				OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

				while(!(GPIO_PortInGet(BTN1_PORT) & (1 << BTN1_PIN))) {						//While button 0 is being held down
					if(accel > 0) {                                                         //Close throttle by 5%
						accel -= .05;
					}

					OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
					APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
					vehPhys.power = (uint16_t) (accel * vehSpecs.maxPower);
					OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
					APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

					OSTimeDly(50u, OS_OPT_TIME_DLY, &err);							//Give player time to release button
					APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				}
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
	OS_FLAGS ledStatusFlags;						//Event flags
	bool slipWarn = false;
	bool headWarn = false;			//Warning state variables

	while(1) {
		//Wait for LED event
		if(!slipWarn && !headWarn) {
			ledStatusFlags = OSFlagPend(&ledWarnFlags, LED_WARN_ANY, 0, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_BLOCKING, &timestamp, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}
		else if(!slipWarn && headWarn) {
			ledStatusFlags = OSFlagPend(&ledWarnFlags, ANY_EX_HEAD, 0, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_BLOCKING, &timestamp, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}
		else if(slipWarn && !headWarn) {
			ledStatusFlags = OSFlagPend(&ledWarnFlags, ANY_EX_SLIP, 0, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_BLOCKING, &timestamp, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}
		else {
			ledStatusFlags = OSFlagPend(&ledWarnFlags, ANY_EX_WARN, 0, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_BLOCKING, &timestamp, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}

		//Check speed warning light events
		if(ledStatusFlags & VEH_OFF_ROAD) {
			GPIO_PinOutSet(LED1_PORT, LED1_PIN);
			while(1) {
				OSTimeDly(5000u, OS_OPT_TIME_DLY, &err);
			}
		}
		else if(ledStatusFlags & TIRE_OFF_ROAD) {
			GPIO_PinOutSet(LED0_PORT, LED0_PIN);
			while(1) {
				OSTimeDly(5000u, OS_OPT_TIME_DLY, &err);
			}
		}
		else {
			if(ledStatusFlags & TIRE_SLIP_WARN){
				slipWarn = true;
				GPIO_PinOutSet(LED0_PORT, LED0_PIN);
				OSTmrStart(&ledToggleTmr, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			}
			if(ledStatusFlags & TIRE_SLIP_WARN_CLR) {
				slipWarn = false;
				GPIO_PinOutClear(LED0_PORT, LED0_PIN);
			}
			if(ledStatusFlags & VEH_HEAD_WARN) {
				headWarn = true;
				GPIO_PinOutSet(LED1_PORT, LED1_PIN);
				OSTmrStart(&ledToggleTmr, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			}
			if(ledStatusFlags & VEH_HEAD_WARN_CLR) {
				headWarn = false;
				GPIO_PinOutClear(LED1_PORT, LED1_PIN);
			}
			if(ledStatusFlags & TOGGLE_WARN_LED) {
				if(slipWarn) {
					GPIO_PinOutToggle(LED0_PORT, LED0_PIN);
				}
				if(headWarn) {
					GPIO_PinOutToggle(LED1_PORT, LED1_PIN);
				}
				OSTmrStart(&ledToggleTmr, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			}
			//Clear current event flags
			OSFlagPost(&ledWarnFlags, ledStatusFlags, OS_OPT_POST_FLAG_CLR, &err);				//Clear flags
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}
	}
}

/* Vehicle State Update Task */
void VehicleStateTask(void* p_args) {
	RTOS_ERR err;
	CPU_TS timestamp;

	bool turning = false;
	OS_FLAGS turnFlags;
	OS_FLAGS statusFlags;
	OS_SEM_CTR physSemStatus;
	GameState_t tempState = GameStart;

	while(1) {

		OSMutexPend(&gameStLock, 0u, OS_OPT_PEND_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		tempState = gameState;														//Update local copy of variable
		OSMutexPost(&gameStLock, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		while(tempState != GamePlay) {												//Don't do anything unless game is being played
			OSTimeDly(500u, OS_OPT_TIME_DLY, &err);									//Sleep
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

			OSMutexPend(&gameStLock, 0u, OS_OPT_PEND_BLOCKING, &timestamp, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			tempState = gameState;													//Update local copy of variable
			OSMutexPost(&gameStLock, OS_OPT_POST_NONE, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}

		//Get flag and semaphore statuses
		physSemStatus = OSSemPend(&physModSem, PEND_NB_TIMEOUT, OS_OPT_PEND_NON_BLOCKING, &timestamp, &err);
		//APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		turnFlags = OSFlagPend(&dirChngFlags, DIRCHG_ANY, PEND_NB_TIMEOUT, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_NON_BLOCKING, &timestamp, &err);
		OSFlagPost(&dirChngFlags, turnFlags, OS_OPT_POST_FLAG_CLR, &err);

		statusFlags = OSFlagPost(&lcdFlags, 0, OS_OPT_POST_FLAG_CLR, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		if(!(statusFlags & GAME_OVER)) {
			if(turnFlags & ANY_TURN) {															//If vehicle turn status changed the turn footprint must be updated and flags cleared
				CalculateCircle();
				turning = true;
				OSFlagPost(&dirChngFlags, turnFlags, OS_OPT_POST_FLAG_CLR, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			}
			if(turnFlags & STRAIGHT) {												//If direction changed from turning to not turning, reset state variable
				turning = false;
			}
			if(turning) {															//If vehicle in a turn its direction must be updated
				UpdateVehicleAngle();
			}
			UpdateVehiclePosition();												//Position Updated on every pass
			if(physSemStatus || turnFlags) {										//If physics tuple has been updated or the turn hass changed the slip parameter is out of date
				UpdateSlip();
			}
		}

		OSSemPost(&gmMonSem, OS_OPT_POST_1, &err);								//Signal game monitor task to run
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		OSTimeDly(VEHST_UPDATE_RATE * 1000u, OS_OPT_TIME_DLY, &err);			//Delay until next update required
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	}
}

/* LCD Display Task */
void LCDDisplayTask(void* p_args) {
	RTOS_ERR err;
	CPU_TS timestamp;

	EMSTATUS status;
	GLIB_Rectangle_t car;
	GameState_t tempState = GameStart;
	struct WayPt_t wayPtArray[30];
	uint8_t size = 0;
	OS_FLAGS flags;
	//char msg[50];

	status = DISPLAY_Init();						//Initialize the display
	if(status != DISPLAY_EMSTATUS_OK) {
		return;
	}

	status = DMD_init(0);							//Initialize the display controller
	if(status != DMD_OK) {
		return;
	}

	GLIB_Context_t  glibContext;					//Initialize graphics library
	status = GLIB_contextInit(&glibContext);
	if (GLIB_OK != status) {
		return;
	}

	//Finish display setup
	glibContext.backgroundColor = White;			//Set Background color
	glibContext.foregroundColor = Black;
	GLIB_setFont(&glibContext, (GLIB_Font_t *)&GLIB_FontNarrow6x8);
	GLIB_clear(&glibContext);
	car.xMin = CAR_XMIN;							//Set rectangle corners
	car.xMax = CAR_XMAX;
	car.yMin = CAR_YMIN;
	car.yMax = CAR_YMAX;

	//Initialize road array
	size = DrawWaypoints(&glibContext, wayPtArray, size, false);

	while(1) {
		if(tempState == GameStart) {
			SelectGameMode(&glibContext);											//User selects game modes with buttons
			SelectDifficulty(&glibContext);											//User selects difficulty
			SelectVehicle(&glibContext);											//User selects vehicle
			PrintGameInit(&glibContext);											//Print selections and wait for confirmation/stallwhile waypoint FIFO gets filled

			flags = OSFlagPend(&btnEventFlags, BTN1_PRESS, 0u, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_BLOCKING, &timestamp, &err); //Wait for user to press B0 to advance to game
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

			OSMutexPend(&gameStLock, 0u, OS_OPT_PEND_BLOCKING, &timestamp, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			gameState = GamePlay;													//Update global state variable
			tempState = gameState;													//Update local copy of variable
			OSMutexPost(&gameStLock, OS_OPT_POST_NONE, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}

		while(tempState == GamePlay) {
			flags = OSFlagPend(&lcdFlags, LCD_FLAG_ANY, 0, OS_OPT_PEND_BLOCKING | OS_OPT_PEND_FLAG_SET_ANY, &timestamp, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

			if(flags & NEW_SHIFT_MESSAGE) {
				GLIB_clear(&glibContext);									//Clear Waypoints
				GLIB_drawRect(&glibContext, &car);							//Draw Vehicle
				DrawVehicleDirLine(&glibContext);							//Draw Vehicle direction
				size = DrawWaypoints(&glibContext, wayPtArray, size, true);	//Draw Waypoints/update road fifos
				PrintVehicleState(&glibContext);							//Print velocity and acceleration
				DMD_updateDisplay();										//Update LCD with changes

				OSFlagPost(&lcdFlags, flags, OS_OPT_POST_FLAG_CLR, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

				OSMutexPend(&gameStLock, 0u, OS_OPT_PEND_BLOCKING, &timestamp, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				tempState = gameState;													//Update local copy of variable
				OSMutexPost(&gameStLock, OS_OPT_POST_NONE, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			}
			else if(flags & GAME_OVER) {
				CleanOS();
				GLIB_clear(&glibContext);
				PrintGameOverStatus(&glibContext);
				DMD_updateDisplay();

				OSFlagPost(&lcdFlags, flags, OS_OPT_POST_FLAG_CLR, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

				gameState = GameEnd;													//Update local copy of variable
				tempState = gameState;
			}
		}

		if(tempState == GameEnd) {
			OSFlagPend(&btnEventFlags, BTN1_PRESS, 0, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_BLOCKING, &timestamp, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			OSFlagPost(&btnEventFlags, BTN1_PRESS, OS_OPT_POST_FLAG_CLR, &err);

			tempState = GameResetSelect(&glibContext);
			size = 0;
			size = DrawWaypoints(&glibContext, wayPtArray, size, false);
			RestartOS();
		}
	}
}

/* Game monitor task */
void GameMonitorTask(void* p_args) {
	CPU_TS timestamp;
	RTOS_ERR err;

	double localXPos, localYPos = 0;
	//bool slipWarn = false;
	bool headWarn = false;
	OS_FLAGS flags;
	GameState_t tempState;

	while(1) {
		OSMutexPend(&gameStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		tempState = gameState;
		OSMutexPost(&gameStLock, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		if(tempState == GamePlay) {
			OSSemPend(&gmMonSem, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

			flags = OSFlagPost(&lcdFlags, 0, OS_OPT_POST_FLAG_CLR, &err);					//Get game status
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

			if(!(flags & GAME_OVER)) {
				/* Compute and send the shift message */
				OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);			//Compute the change in Position
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				localXPos = vehState.xPos;
				localYPos = vehState.yPos;
				OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


				/* Check if new vehicle position is out of bounds */
				OSMutexPend(&usedRdLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				if((usedRoad.waypoints.head->next != NULL) && (localYPos > usedRoad.waypoints.head->next->yPos)) {		//This is so ugly, it is checking if vehicle has passed the next waypoint yet
					FIFO_Pop(&usedRoad.waypoints);
					gameStats.wayPtsPassed++;
				}
				if(usedRoad.waypoints.currWayPts == 1) {
					OSFlagPost(&lcdFlags, GAME_OVER, OS_OPT_POST_FLAG_SET, &err);
					APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				}
				OSMutexPost(&usedRdLock, OS_OPT_POST_NONE, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				if(OutsideBoundary(localXPos, localYPos)) {												//Check if the vehicle has left the road
					gameStats.gameResult = LeftRoad;
					OSFlagPost(&lcdFlags, GAME_OVER, OS_OPT_POST_FLAG_SET, &err);
					APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
					OSFlagPost(&ledWarnFlags, VEH_OFF_ROAD | VEH_HEAD_WARN_CLR, OS_OPT_POST_FLAG_SET, &err);
					APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				}
				else {
					OSFlagPost(&lcdFlags, NEW_SHIFT_MESSAGE, OS_OPT_POST_FLAG_SET, &err);				//Notify LCD task to update display
					APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				}

				/* Check if vehicle is on course to go off couse within 30m */
				headWarn = TrendingOut(localXPos, localYPos);
				if(headWarn) {													//If trending out set LED blink flag
					OSFlagPost(&ledWarnFlags, VEH_HEAD_WARN, OS_OPT_POST_FLAG_SET, &err);
					APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				}
				else {																		//If not trending out and flag was set, clear the LED blink flag
					OSFlagPost(&ledWarnFlags, VEH_HEAD_WARN_CLR, OS_OPT_POST_FLAG_SET, &err);
					APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				}

				/* Check slip warnings */
				flags = OSFlagPost(&ledWarnFlags, 0, OS_OPT_POST_FLAG_SET, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

				OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				if(vehState.prcntSlip > (.9 * VEH_SLIP_TOLERANCE) && !(flags & TIRE_SLIP_WARN)) {							//Slip percent greater than 90% of the tolerated slip
						OSFlagPost(&ledWarnFlags, TIRE_SLIP_WARN, OS_OPT_POST_FLAG_SET, &err);
						APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				}
				else if(vehState.prcntSlip < (.9 * VEH_SLIP_TOLERANCE) && (flags & TIRE_SLIP_WARN)) {						//Tire no longer slipping, clear flag
						OSFlagPost(&ledWarnFlags, TIRE_SLIP_WARN_CLR, OS_OPT_POST_FLAG_SET, &err);
						APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				}
				else if(vehState.prcntSlip > VEH_SLIP_TOLERANCE) {															//Tire slip exceeded tolerated limit
					gameStats.gameResult = SpunOut;
					OSFlagPost(&lcdFlags, GAME_OVER, OS_OPT_POST_FLAG_SET, &err);
					APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
					OSFlagPost(&ledWarnFlags, TIRE_OFF_ROAD | TIRE_SLIP_WARN_CLR, OS_OPT_POST_FLAG_SET, &err);
					APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				}
				OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			}
		}
		else {
			OSTimeDly(500u, OS_OPT_TIME_DLY, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}
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

/* Clean OS */
void CleanOS(void) {
	RTOS_ERR err;

	OSSchedLock(&err);
	OSFlagPost(&dirChngFlags, DIRCHG_ANY, OS_OPT_POST_FLAG_CLR, &err);			//Clear all flags
	OSFlagPost(&ledWarnFlags, LED_FLAGS_ALL, OS_OPT_POST_FLAG_CLR, &err);
	OSFlagPost(&lcdFlags, LCD_FLAG_ANY, OS_OPT_POST_FLAG_CLR, &err);
	OSFlagPost(&btnEventFlags, BTN_FLG_ANY, OS_OPT_POST_FLAG_CLR, &err);

	OSSemPendAbort(&physModSem, OS_OPT_PEND_ABORT_ALL, &err);					//Clear all semaphores
	OSSemPendAbort(&gmMonSem, OS_OPT_PEND_ABORT_ALL, &err);

	OSMutexPendAbort(&vehStLock, OS_OPT_PEND_ABORT_ALL, &err);					//Clear all mutexes
	OSMutexPendAbort(&physTupLk, OS_OPT_PEND_ABORT_ALL, &err);
	OSMutexPendAbort(&wayPtLock, OS_OPT_PEND_ABORT_ALL, &err);
	OSMutexPendAbort(&usedRdLock, OS_OPT_PEND_ABORT_ALL, &err);
	OSMutexPendAbort(&gameStLock, OS_OPT_PEND_ABORT_ALL, &err);

	OSTaskDel(&rdGenTaskTCB, &err);												//Close all tasks not being used in menus
	OSTaskDel(&dirTaskTCB, &err);
	OSTaskDel(&physModTaskTCB, &err);
	OSTaskDel(&spdTaskTCB, &err);
	OSTaskDel(&ledTaskTCB, &err);
	OSTaskDel(&vehStTaskTCB, &err);
	OSTaskDel(&gmMonTaskTCB, &err);

	OSSchedUnlock(&err);
}

/* Restart OS */
void RestartOS(void) {
	RTOS_ERR err;

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
}
