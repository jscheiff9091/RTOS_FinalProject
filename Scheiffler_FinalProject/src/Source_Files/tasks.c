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
//OS_SEM gmModeSem;

//Timers
OS_TMR ledToggleTmr;

//Mutexes
OS_MUTEX vehStLock;
OS_MUTEX physTupLk;
OS_MUTEX wayPtLock;
OS_MUTEX usedRdLock;

//Message Queues
OS_Q LCDShiftQ;

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

	//Create mutex to protect waypoint FIFO
	OSMutexCreate(&usedRdLock, "Used Road Waypoints FIFO Mutex", &err);
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

			dirFlags = OSFlagPend(&dirChngFlags, DIRCHG_ANY, 0, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_NON_BLOCKING, &timestamp, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
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

	uint16_t cpAccelSd, cpAccelFwd, cpVel, cpPower, cpRad;

	while(1) {
		OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);		//Make copies of the current data in the tuple
		cpAccelFwd = vehPhys.accelFwd;
		cpAccelSd = vehPhys.accelSd;
		cpVel = vehPhys.velocity;
		cpPower = vehPhys.power;
		OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

		if(cpVel != 0) {
			cpAccelFwd = cpPower / cpVel;										//Calculate forward acceleration
		}
		else if(cpPower > 0 && cpVel == 0) {
			cpAccelFwd = 1;
		}
		else if(cpPower == 0 && cpVel != 0) {									//Slowing down with no power input
			cpAccelFwd = -1;
		}

		OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);		//Get copy of current turn radius
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
		cpRad = vehState.radius;
		OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

		if(cpRad == 0) {
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
	double accel = 0;

	GPIO_InitBTNs();															//Enable push buttons

	while(1) {
		// Wait to be signaled by button ISR
		btnFlags = OSFlagPend(&btnEventFlags, BTN_FLG_ANY, 0, (OS_OPT_PEND_BLOCKING | OS_OPT_PEND_FLAG_SET_ANY), &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		if(btnFlags & BTN0_PRESS) {														//Button 0 pressed
			if(accel < 1) { 															//Open throttle by 5%
				accel += .05;
			}

			OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			vehPhys.power = (uint16_t) accel * vehSpecs.maxPower;
			OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

			while(!(GPIO_PortInGet(BTN0_PORT) & (1 << BTN0_PIN))) {						//While button 0 is being held down
				if(accel < 1) { 														//Open throttle by 5%
					accel += .05;
				}

				OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				vehPhys.power = (uint16_t) accel * vehSpecs.maxPower;
				OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

				OSTimeDly(100u, OS_OPT_TIME_DLY, &err);									//Give player time to release button
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			}
		}
		else if(btnFlags & BTN1_PRESS) {												//Button 1 pressed
			if(accel > 0) {																//Close throttle by 5%
				accel -= .05;
			}

			OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			vehPhys.power = (uint16_t) accel * vehSpecs.maxPower;
			OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

			while(!(GPIO_PortInGet(BTN1_PORT) & (1 << BTN1_PIN))) {						//While button 0 is being held down
				if(accel > 0) {                                                         //Close throttle by 5%
					accel -= .05;
				}

				OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
				vehPhys.power = (uint16_t) accel * vehSpecs.maxPower;
				OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
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

/* Vehicle State Update Task */
void VehicleStateTask(void* p_args) {
	RTOS_ERR err;
	CPU_TS timestamp;

	bool turning = false;
	OS_FLAGS turnFlags;
	OS_SEM_CTR physSemStatus;

	while(1) {
		//Get flag and semaphore statuses
		physSemStatus = OSSemPend(&physModSem, PEND_NB_TIMEOUT, OS_OPT_PEND_NON_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		turnFlags = OSFlagPend(&dirChngFlags, DIRCHG_ANY, PEND_NB_TIMEOUT, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_NON_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		if(turnFlags) {															//If vehicle turn status changed the turn footprint must be updated and flags cleared
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

	char* dirStr[] = DIRECTION_STRINGS;
	char buffer[10];
	Direction_t dir;
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

/* Game monitor task */
void GameMonitorTask(void* p_args) {
	CPU_TS timestamp;
	RTOS_ERR err;

	int16_t localXPos, localYPos = 0;
	int16_t xDiff, yDiff = 0;
	ScreenShift_t* scrnShft;
	OS_FLAGS flags;

	// Initialize used waypoints FIFO
	OSMutexPend(&wayPtLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	struct WayPt_t* temp = FIFO_Peek(&road.waypoints);

	OSMutexPend(&usedRdLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	FIFO_Append(&usedRoad.waypoints, 0, USE_TRUE_X);
	FIFO_Append(&usedRoad.waypoints, temp->xPos, USE_TRUE_X);

	OSMutexPost(&wayPtLock, OS_OPT_POST_NONE, &err);											//Release road FIFO locks
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	OSMutexPost(&usedRdLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


	while(1) {
		OSSemPend(&gmMonSem, PEND_TIMEOUT, OS_OPT_PEND_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		/* Compute and send the shift message */
		OSMutexPend(&vehStLock, PEND_TIMEOUT, OS_OPT_PEND_BLOCKING, &timestamp, &err);			//Compute the change in Position
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		xDiff = vehState.xPos - localXPos;
		yDiff = vehState.yPos - localYPos;
		localXPos = vehState.xPos;
		localYPos = vehState.yPos;
		OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		scrnShft = (ScreenShift_t*) malloc(sizeof(ScreenShift_t));								//Create pointer to shift message
		scrnShft->xShift = xDiff;
		scrnShft->yShift = yDiff;

		OSQPost(&LCDShiftQ, (void*)scrnShft, sizeof(ScreenShift_t), OS_OPT_POST_FIFO, &err);	//Send shift message to LCD task
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		OSFlagPost(&lcdFlags, NEW_SHIFT_MESSAGE, OS_OPT_POST_FLAG_SET, &err);					//Notify LCD Task of message in the queue
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


		/* Check if new vehicle position is out of bounds */
		OSMutexPend(&usedRdLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		if(localYPos > usedRoad.waypoints.head->next->yPos) {									//This is so ugly, it is checking if vehicle has passed the next waypoint yet
			FIFO_Pop(&usedRoad.waypoints);
		}
		OSMutexPost(&usedRdLock, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		if(OutsideBoundary(localXPos, localYPos)) {												//Check if the vehicle has left the road
			OSFlagPost(&lcdFlags, GAME_OVER, OS_OPT_POST_FLAG_SET, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			OSFlagPost(&ledWarnFlags, VEH_OFF_ROAD, OS_OPT_POST_FLAG_SET, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}

		/* Check if vehicle is on course to go off couse within 30m */
		flags = OSFlagPend(&ledWarnFlags, LED_WARN_ANY, 0, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_NON_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		if(TrendingOut(localXPos, localYPos)) {													//If trending out set LED blink flag
			OSFlagPost(&ledWarnFlags, VEH_HEAD_WARN, OS_OPT_POST_FLAG_SET, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}
		else {
			if(flags & VEH_HEAD_WARN) {													//If not trending out and flag was set, clear the LED blink flag
				OSFlagPost(&ledWarnFlags, VEH_HEAD_WARN, OS_OPT_POST_FLAG_CLR, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			}
		}

		/* Check slip warnings */
		OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		if(vehState.prcntSlip > (.9 * VEH_SLIP_TOLERANCE)) {							//Slip percent greater than 90% of the tolerated slip
			if(!(flags & TIRE_SLIP_WARN)) {
				OSFlagPost(&ledWarnFlags, TIRE_SLIP_WARN, OS_OPT_POST_FLAG_SET, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			}
		}
		else {
			if(flags & TIRE_SLIP_WARN) {												//Tire no longer slipping, clear flag
				OSFlagPost(&ledWarnFlags, TIRE_SLIP_WARN, OS_OPT_POST_FLAG_CLR, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			}
		}

		if(vehState.prcntSlip > .9 * VEH_SLIP_TOLERANCE) {								//Tire slip exceeded tolerated limit
			OSFlagPost(&lcdFlags, GAME_OVER, OS_OPT_POST_FLAG_SET, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
			OSFlagPost(&ledWarnFlags, TIRE_OFF_ROAD, OS_OPT_POST_FLAG_SET, &err);
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

