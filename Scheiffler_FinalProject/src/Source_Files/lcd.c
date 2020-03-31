/*
 * lcd.c
 *
 *  Created on: Feb 28, 2020
 *      Author: Jacob S
 */

#include "lcd.h"
#include "gpio.h"
#include "slider.h"

#include "display.h"
#include "textdisplay.h"
#include "retargettextdisplay.h"

#include  <common/include/rtos_utils.h>
#include <stdio.h>
#include <stdlib.h>

OS_TCB LCDDispTaskTCB;
CPU_STK LCDDisplayTaskStack[LCD_DISP_STACK_SIZE];

OS_TMR LCDDispTmr;

// ----- LCD Display Task -----
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


// ----- LCD Task Timer Callback -----
void LCDTmrCallback(void* p_tmr, void* p_args) {
	RTOS_ERR err;

	//Move LCD Display task to the ready queue
	OSTaskResume(&LCDDispTaskTCB, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}
