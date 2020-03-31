/***************************************************************************//**
 * @file
 * @brief Simple LED Blink Demo for SLSTK3402A
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "bsp.h"

#include "gpio.h"
#include "slider.h"
#include "capsense.h"
#include "cmu.h"
#include "main.h"
#include "lcd.h"

#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"

#include  <bsp_os.h>

#include  <cpu/include/cpu.h>
#include  <common/include/common.h>
#include  <kernel/include/os.h>
#include  <kernel/include/os_trace.h>

#include  <common/include/lib_def.h>
#include  <common/include/rtos_utils.h>
#include  <common/include/toolchains.h>

#include "display.h"
#include "textdisplay.h"
#include "retargettextdisplay.h"

#define HFRCO_FREQ 		40000000

// RTOS Global variables
OS_TCB startTaskTCB;
OS_TCB idleTaskTCB;
OS_TCB vehicleMonTaskTCB;

CPU_STK startTaskStack[START_STACK_SIZE];
CPU_STK idleTaskStack[IDLE_STACK_SIZE];
CPU_STK vehicleMonTaskStack[VEH_MON_STACK_SIZE];

OS_FLAG_GRP vehMonFlags;
OS_TMR	vehTurnTimeout;

/* Main */
int main(void)
{
	KeithGInit();

	CMU_RouteGPIOClock();       //Enable GPIO Clock


	/* Initialize tasks, OS, etc. */
	RTOS_ERR err;
	CPU_Init();								//Example Code called these functions...why not???
	BSP_SystemInit();
	OS_TRACE_INIT();

	OSInit(&err);							//Initialize kernel
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	OSTaskCreate(&startTaskTCB,     		//Create start task
				 "Start Task",
				 StartTask,
				 DEF_NULL,
				 START_TASK_PRIO,
				 &startTaskStack[0],
				 (START_STACK_SIZE / 10u),
				 START_STACK_SIZE,
				 10u,
				 0u,
				 DEF_NULL,
				 OS_OPT_TASK_STK_CLR,
				 &err);

	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	OSStart(&err);                       		//Start the kernel, LET'S DO THIS!!!!
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	while(1);
}


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
	//Create semaphore used to signal a button press occurred to the speed setpoint task
	OSSemCreate(&setptFifoSem, "Button Press Signal Semaphore", CNT_ZERO, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);


	/**** Create all Flag groups used ****/
	//Create event flag group to communicate with the vehicle monitor task
	OSFlagCreate(&vehMonFlags, "Vehicle Monitor Event Flags", VEH_MON_CLR_FLAGS, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	//Create the event flag to notify the LED Driver task of LED State change
	OSFlagCreate(&LEDDriverEvent, "Vehicle Warning Event Flag", LED_WARN_CLR_FLAGS, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);


	/**** Create all mutexs used *****/
	//Create mutex to protect the speed setpoint data
	OSMutexCreate(&setptDataMutex, "Speed Setpoint Data Mutex", &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	//Create mutex to protect the vehicle direction state variable
	OSMutexCreate(&vehDirMutex, "Vehicle Direction Mutex", &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);


	/**** Create all timers used ****/
	//Create the timer to schedule the vehicle direction task
	OSTmrCreate(&vehDirTimer,
				"Vehicle Direction Task Timer",
				NO_DLY,
				VEH_DIR_TMR_CNT,
				OS_OPT_TMR_PERIODIC,
				&SLD_TimerCallback,
				DEF_NULL,
				&err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	//Create vehicle turning monitor timeout timer
	OSTmrCreate(&vehTurnTimeout,
				"Vehicle Turn Timeout Timer",
				VEH_TURN_TIMEOUT,
				0,
				OS_OPT_TMR_ONE_SHOT,
				&VehicleTurnTimeout,
				DEF_NULL,
				&err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	//Create timer to signal the LCD Display to move to the ready queue
	OSTmrCreate(&LCDDispTmr,
				"LCD Display Task Timer",
				0,
				LCD_TMR_PERIOD,
				OS_OPT_TMR_PERIODIC,
				&LCDTmrCallback,
				DEF_NULL,
				&err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);


	/**** Create all tasks ******/
	//Create button input task
	OSTaskCreate(&speedSetPTTaskTCB,
				 "Speed Setpoint Task",
				 SpeedSetpointTask,
				 DEF_NULL,
				 SPD_SETPT_TASK_PRIO,
				 &speedSetPTTaskStack[0],
				 (SPD_SETPT_STACK_SIZE / 2u),
				 SPD_SETPT_STACK_SIZE,
				 0u,
				 0u,
				 DEF_NULL,
				 OS_OPT_TASK_STK_CLR,
				 &err);

	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	//Create LED driver task
	OSTaskCreate(&LEDDriverTaskTCB,
				 "LED Driver Task",
				 LEDDriverTask,
				 DEF_NULL,
				 LED_DRV_TASK_PRIO,
				 &LEDDriverTaskStack[0],
				 (LED_DRV_STACK_SIZE / 2u),
				 LED_DRV_STACK_SIZE,
				 0u,
				 0u,
				 DEF_NULL,
				 OS_OPT_TASK_STK_CLR,
				 &err);

	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	//Create slider input task
    OSTaskCreate(&vehicleDirTaskTCB,
				 "Vehicle Direction Monitor Task",
				 VehicleDirectionTask,
				 DEF_NULL,
				 VEH_DIR_TASK_PRIO,
				 &vehicleDirTaskStack[0],
				 (VEH_DIR_STACK_SIZE / 2u),
				 VEH_DIR_STACK_SIZE,
				 0u,
				 0u,
				 DEF_NULL,
				 OS_OPT_TASK_STK_CLR,
				 &err);

    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

    //Create vehicle monitor task
    OSTaskCreate(&vehicleMonTaskTCB,
				 "Vehicle Monitor Task",
				 VehicleMonitorTask,
				 DEF_NULL,
				 VEH_MON_TASK_PRIO,
				 &vehicleMonTaskStack[0],
				 (VEH_MON_STACK_SIZE / 2u),
				 VEH_MON_STACK_SIZE,
				 0u,
				 0u,
				 DEF_NULL,
				 OS_OPT_TASK_STK_CLR,
				 &err);

	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	//Create LCD Display Task
	OSTaskCreate(&LCDDispTaskTCB,
				 "LCD Display Task",
				 LCDDisplayTask,
				 DEF_NULL,
				 LCD_DISP_TASK_PRIO,
				 &LCDDisplayTaskStack[0],
				 (LCD_DISP_STACK_SIZE / 2u),
				 LCD_DISP_STACK_SIZE,
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

/* Idle Task */
void IdleTask(void* p_args) {

	RTOS_ERR err;

	OSTaskDel(&startTaskTCB, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	while(1) {
		EMU_EnterEM1();
	}
}

void VehicleMonitorTask(void* p_args) {
	RTOS_ERR err;
	CPU_TS timestamp;
	OS_FLAGS flags;

	bool speedWarn = false;		//LED Currently signaling a spped warning
	bool turnWarn = false;		//LED Currently signaling a turn warning
	Direction_t currDir = Straight;
	int currSpeed = 40;

	//Start the timer
	OSTmrStart(&vehTurnTimeout, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	while(1) {
		//Wait for speed change, direction change, or hard left/right timeout
		flags = OSFlagPend(&vehMonFlags, VEH_MON_SET_FLAGS, 0, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
		OSFlagPost(&vehMonFlags, flags, OS_OPT_POST_FLAG_CLR, &err);

		if(flags & SPD_SETPT_FLAG) {																		//Speed change occurred
			//Critical section
			OSMutexPend(&setptDataMutex, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
			currSpeed = setptData.speed;
			OSMutexPost(&setptDataMutex, OS_OPT_POST_NONE, &err);
			if(currSpeed > 70 && !speedWarn) {														//Send Speed violation if speed is greater than 70 for any drection
				OSFlagPost(&LEDDriverEvent, LED_WARN_SPD_VIOLATION, OS_OPT_POST_FLAG_SET, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
				speedWarn = true;
			}
			else if(currSpeed > 50 && !speedWarn && currDir != Straight) {							//Send speed warning if speed is greater than 50 and the drive is turning
				OSFlagPost(&LEDDriverEvent, LED_WARN_SPD_VIOLATION, OS_OPT_POST_FLAG_SET, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
				speedWarn = true;
			}
			else if(setptData.speed < 75 && speedWarn && currDir == Straight) {						//Clear speed violation if driver not turning and speed less than 75
				OSFlagPost(&LEDDriverEvent, LED_WARN_CLR_SPD_VIOLATION, OS_OPT_POST_FLAG_SET, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
				speedWarn = false;
			}
			else if(setptData.speed < 55 && speedWarn) {											//Clear speed violation if speed is less than 55 for any turn direction
				OSFlagPost(&LEDDriverEvent, LED_WARN_CLR_SPD_VIOLATION, OS_OPT_POST_FLAG_SET, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
				speedWarn = false;
			}
		}

		if(flags & VEH_DIR_FLAG) {																			//Vehicle direction changed
			OSMutexPend(&vehDirMutex, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
			currDir = vehicleDir.dir;																		//Get Current direction
			OSMutexPost(&vehDirMutex, OS_OPT_POST_NONE, &err);
			if(turnWarn) {																					//turn off warning
				turnWarn = false;
				OSFlagPost(&LEDDriverEvent, LED_WARN_CLR_TRN_VIOLATION, OS_OPT_POST_FLAG_SET, &err);		//Signal LED task to turn off turn warning light
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

			}
			else {																							//Only need to stop timer if it hasn't already expired
				OSTmrStop(&vehTurnTimeout, OS_OPT_TMR_NONE, DEF_NULL, &err);								//Restart the timer
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
			}
			OSTmrStart(&vehTurnTimeout, &err);
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);


			if(currSpeed > 70 && !speedWarn) {															//Speed greater than or equal to 75, speed violation
				OSFlagPost(&LEDDriverEvent, LED_WARN_SPD_VIOLATION, OS_OPT_POST_FLAG_SET, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
				speedWarn = true;
			}
			else if(currSpeed < 75 && speedWarn && currDir == Straight) {								//Speed less than or equal to 70, and direction straight clear speed violation
				OSFlagPost(&LEDDriverEvent, LED_WARN_CLR_SPD_VIOLATION, OS_OPT_POST_FLAG_SET, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
				speedWarn = false;
			}

			else if(currSpeed > 50 && !speedWarn && currDir != Straight) {								//Speed greater than or equal to 55 and direction is not straigt
				OSFlagPost(&LEDDriverEvent, LED_WARN_SPD_VIOLATION, OS_OPT_POST_FLAG_SET, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
				speedWarn = true;
			}
			else if(currSpeed < 55 && speedWarn) {														//Any speed less than 55, no speed violation
				OSFlagPost(&LEDDriverEvent, LED_WARN_CLR_SPD_VIOLATION, OS_OPT_POST_FLAG_SET, &err);
				APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
				speedWarn = false;
			}
		}

		if(flags & VEH_TURNTM_FLAG) {																		//Hard left/right timeout expired
			turnWarn = true;																				//Set turn warning state variable
			OSFlagPost(&LEDDriverEvent, LED_WARN_TRN_VIOLATION, OS_OPT_POST_FLAG_SET, &err);				//Signal LED task to turn on turn warning light
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
		}
	}
}

void VehicleTurnTimeout(void* tmr, void* p_args) {
	RTOS_ERR err;

	OSFlagPost(&vehMonFlags, VEH_TURNTM_FLAG, OS_OPT_POST_FLAG_SET, &err);	//Vehicle turn hard left/right timeout, notify Vehicle Monitor task
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
}

void KeithGInit(void) {
	EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_STK_DEFAULT;
	CMU_HFXOInit_TypeDef hfxoInit = CMU_HFXOINIT_STK_DEFAULT;

	/* Chip errata */
	CHIP_Init();

	/* Init DCDC regulator and HFXO with kit specific parameters */
	EMU_DCDCInit(&dcdcInit);
	CMU_HFXOInit(&hfxoInit);

	/* Switch HFCLK to HFXO and disable HFRCO */
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
	CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);
}
