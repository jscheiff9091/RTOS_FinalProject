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

OS_TMR LCDDispTmr;


/* LCD Task Timer Callback */
void LCDTmrCallback(void* p_tmr, void* p_args) {
	RTOS_ERR err;

	//Move LCD Display task to the ready queue
	OSTaskResume(&LCDDispTaskTCB, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}
