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
#include "tasks.h"
#include "game.h"

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

#define HFRCO_FREQ 		40000000

/* Main */
int main(void)
{
	KeithGInit();

	CMU_RouteGPIOClock();       			//Enable GPIO Clock

	road.waypoints = (WayPtFIFO_t) { .head = NULL, .tail = NULL, .currWayPts = 0, .totalWayPts = 10 };
	usedRoad.waypoints = (WayPtFIFO_t) { .head = NULL, .tail = NULL, .currWayPts = 0, .totalWayPts = 10 };
	vehState = (VehSt_T){ .vehDir = Straight,
						  .xPos = 0,
						  .yPos = 0,
	                      .circX = 0,
	                      .circY = 0,
	                      .radius = 0,
						  .angle = 90,
	                      .prcntSlip = 0 };
	vehSpecs = (VehSpecs_t) { .vehicleName = "Car",
	                          .mass = 20,
	                          .maxPower = 20,
	                          .turnRadius = 10,
	                          .vehicleWidth = 1,
	                          .dragArea = 1,
	                          .tireType = Truck};
	vehPhys = (VehPhys_t) { .accelSd = 0,
	                        .accelFwd = 0,
	                        .velocity = 0,
	                        .power = 0,
	                        .zAccel = 0,
	                        .bankAngle = 0,
	                        .roll = 0 };
	gameStats = (GameStats_t) { .distance = 0,
	                            .sumOfSpeeds = 0,
	                            .numSums = 0,
	                            .gameResult = Finished };
	gameState = GameStart;

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
