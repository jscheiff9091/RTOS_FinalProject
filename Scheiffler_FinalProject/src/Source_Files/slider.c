/*
 * slider.c
 *
 *  Created on: Jan 23, 2020
 *      Author: Jacob S
 */
#include "slider.h"
#include "capsense.h"
#include <kernel/include/os.h>
#include  <common/include/rtos_utils.h>

SLD_Direction_t vehicleDir;

/* Slider Initialize */
void SLD_Init(void) {
	CAPSENSE_Init();			//Enable capctive sensing slider
}

/* Slider is Pressed? */
SLD_SliderPressedState_t SLD_IsPressed(SLD_SliderSide_t side) {

	if(side != SLD_LeftSide && side != SLD_RightSide) {
		return SLD_Released;
	}

	CAPSENSE_Sense();                  																//Read Sliders

	if(side == SLD_LeftSide && (CAPSENSE_getPressed(POS0) || CAPSENSE_getPressed(POS1))) {         	//Checking left side and one or both of the two left side positions are pressed
		return SLD_Pressed;
	}
	else if(side == SLD_RightSide && (CAPSENSE_getPressed(POS2) || CAPSENSE_getPressed(POS3))) {   	//Checking right side and one or both of the right side positions are pressed
		return SLD_Pressed;
	}

	return SLD_Released;
}

/* Slider - Get Slider Action */
LED_Action_t SLD_GetSLDAction(SLD_SliderPressedState_t sld_leftSideState, SLD_SliderPressedState_t sld_rightSideState) {
	//Determine which combination of left and right side is selected (or not)
	if(sld_leftSideState == SLD_Pressed && sld_rightSideState == SLD_Pressed) { 					//Both side selected, turn off LEDs
		return LED_BOTH_OFF;
	}
	else if(sld_leftSideState == SLD_Pressed && sld_rightSideState == SLD_Released) {				//Left side pressed, right side released, turn on LED0
		return LED0_ON;
	}
	else if(sld_leftSideState == SLD_Released && sld_rightSideState == SLD_Pressed) {				//LEft side released, right side pressed, turn on LED1
		return LED1_ON;
	}
	else {																//Neither side is pressed, turn both off
		return LED_BOTH_OFF;
	}
}

/* Slider - Get Current Diection */
Direction_t SLD_GetDirection(void) {
	//CAPSENSE_Init();									//Re-initialize CAPSENSE -> CAPSENSE readings more reliable when I added this?
	RTOS_ERR err;
	Direction_t ret;
	OSSchedLock(&err);
	CAPSENSE_Sense();									//Read the touch slider
	OSSchedUnlock(&err);

	if(CAPSENSE_getPressed(POS0)){						//Position 0selected?
		ret = HardLeft;
	}
	else if(CAPSENSE_getPressed(POS1)) {				//Position 1 selected?
		ret = Left;
	}
	else if(CAPSENSE_getPressed(POS2)) {				//Position 2 selected?
		ret = Right;
	}
	else if(CAPSENSE_getPressed(POS3)) {				//Position 3 selected?
		ret = HardRight;
	}
	else {
		ret = Straight;
	}
	return ret;
}

/* Touch slider signal timer */
void SLD_TimerCallback(void* p_tmr, void* p_args) {
	RTOS_ERR err;

	OSTaskResume(&vehicleDirTaskTCB, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
}
