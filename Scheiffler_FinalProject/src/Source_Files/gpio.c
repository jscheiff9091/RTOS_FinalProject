/*
 * gpio.c
 *
 *  Created on: Jan 23, 2020
 *      Author: Jacob S
 */
#include "em_gpio.h"
#include "em_core.h"
#include "slider.h"
#include "gpio.h"
#include "fifo.h"
#include "tasks.h"
#include <common/include/rtos_utils.h>
#include <stdlib.h>

//----- Function Definitions -----


/* GPIO - Init LEDs */
void GPIO_InitLEDs() {
	//LED0
	GPIO_DriveStrengthSet(LED0_PORT, gpioDriveStrengthStrongAlternateStrong);	//Set drive strength
	GPIO_PinModeSet(LED0_PORT, LED0_PIN, gpioModePushPull, LED_DEFAULT);		//Set LED0 pin Mode														//Initialize LED0 state

	//LED1
	GPIO_DriveStrengthSet(LED1_PORT, gpioDriveStrengthStrongAlternateStrong);	//Set drive strength
	GPIO_PinModeSet(LED1_PORT, LED1_PIN, gpioModePushPull, LED_DEFAULT);		//Set LED1 pin mode														//Initialize LED1 state
}


/* GPIO - Initialize Buttons */
void GPIO_InitBTNs() {
	GPIO_PinModeSet(BTN0_PORT, BTN0_PIN, gpioModeInputPullFilter, PULLUP);		//Set buttons to input with pullup resistor (pressed means bit is low)
	GPIO_PinModeSet(BTN1_PORT, BTN1_PIN, gpioModeInputPullFilter, PULLUP);

	GPIO_ExtIntConfig(BTN0_PORT, BTN0_PIN, BTN0_INT, BTN_RISE_DIS, BTN_FALL_EN, true); //Config rising edge interrupt for both buttons
	GPIO_ExtIntConfig(BTN1_PORT, BTN1_PIN, BTN1_INT, BTN_RISE_DIS, BTN_FALL_EN, true);

	NVIC_EnableIRQ(GPIO_EVEN_IRQn);												//Enable interrupt in the NVIC
}


/* GPIO - Get Button State */
GPIO_BTNState_t GPIO_GetBTNState(uint8_t btn) {
	if(btn != BTN0 && btn != BTN1) {    					//Ensure btn passed is BTN1 or BTN0
		return GPIO_BTNReleased;
	}

	uint32_t GPIO_PortFVal = GPIO_PortInGet(BTN_PORT); 		//Get value of DIN Register for port F
	if(btn == BTN0 && (~GPIO_PortFVal & BTN0_BIT)) {   		//If btn is BTN0 and BTN0 bit is a zero, BTN 0 Pressed
		return GPIO_BTNPressed;
	}
	else if(btn == BTN1 && (~GPIO_PortFVal & BTN1_BIT)) {  	//If btn is BTN1 and BTN1 bit is a zero, BTN1 Pressed
		return GPIO_BTNPressed;
	}

	return GPIO_BTNReleased;                              	//Return default value, that button is released
}


/* GPIO - Get Button Action */
LED_Action_t GPIO_GetButtonAction(GPIO_BTNState_t btn0State, GPIO_BTNState_t btn1State) {
	//Determine the LEDs that should be on based on the button state
	if(btn0State == GPIO_BTNPressed && btn1State == GPIO_BTNPressed) {				//Both BTN0 and BTN1 are pressed
		return LED_BOTH_OFF;
	}
	else if(btn0State == GPIO_BTNPressed && btn1State != GPIO_BTNPressed) {			//BTN0 pressed and BTN1 is not pressed
		return LED0_ON;
	}
	else if(btn0State != GPIO_BTNPressed && btn1State == GPIO_BTNPressed) {			//BTN0 is not pressed and BTN1 pressed
		return LED1_ON;
	}
	else {																	//Neither BTN0 nor BTN1 are pressed
		return LED_BOTH_OFF;
	}
}


/* Set LEDs */
void SetLEDs(LED_Action_t btn_action, LED_Action_t sld_action) {
	if(btn_action == LED_BOTH_OFF && sld_action == LED_BOTH_OFF) {          	//Both the buttons and slider want both LEDs off
		GPIO_PinOutClear(LED0_PORT, LED0_PIN);
		GPIO_PinOutClear(LED1_PORT, LED1_PIN);
	}
	else if(btn_action == LED0_ON && sld_action == LED_BOTH_OFF) {				//Button 0 pressed, Slider is either both pressed or neither
		GPIO_PinOutSet(LED0_PORT, LED0_PIN);
		GPIO_PinOutClear(LED1_PORT, LED1_PIN);
	}
	else if(btn_action == LED1_ON && sld_action == LED_BOTH_OFF) {				//Button 1 pressed, Slider is either both pressed or neither
		GPIO_PinOutClear(LED0_PORT, LED0_PIN);
		GPIO_PinOutSet(LED1_PORT, LED1_PIN);
	}
	else if(btn_action == LED_BOTH_OFF && sld_action == LED0_ON) {				//Buttons either both pressed or neither, Left side of slider pressed down
		GPIO_PinOutSet(LED0_PORT, LED0_PIN);
		GPIO_PinOutClear(LED1_PORT, LED1_PIN);
	}
	else if(btn_action == LED_BOTH_OFF && sld_action == LED1_ON) {				//Buttons either both pressed or neither, Right side of slider pressed down
		GPIO_PinOutClear(LED0_PORT, LED0_PIN);
		GPIO_PinOutSet(LED1_PORT, LED1_PIN);
	}
	else if(btn_action == LED0_ON && sld_action == LED0_ON) {					//Button 0 pressed and left side of slider pressed
		GPIO_PinOutSet(LED0_PORT, LED0_PIN);
		GPIO_PinOutClear(LED1_PORT, LED1_PIN);
	}
	else if(btn_action == LED0_ON && sld_action == LED1_ON) {                   //Button 0 pressed and right side of slider pressed
		GPIO_PinOutSet(LED0_PORT, LED0_PIN);
		GPIO_PinOutSet(LED1_PORT, LED1_PIN);
	}
	else if(btn_action == LED1_ON && sld_action == LED0_ON) { 					//Button 1 pressed and left side of slider pressed
		GPIO_PinOutSet(LED0_PORT, LED0_PIN);
		GPIO_PinOutSet(LED1_PORT, LED1_PIN);
	}
	else if(btn_action == LED1_ON && sld_action == LED1_ON) {					//Button 1 pressed and right side of slider pressed
		GPIO_PinOutClear(LED0_PORT, LED0_PIN);
		GPIO_PinOutSet(LED1_PORT, LED1_PIN);
	}
}


/* GPIO Even IRQ Handler */
void GPIO_EVEN_IRQHandler(void) {
	OSIntEnter();                                   //Let OS know it is currently executing an ISR

	uint32_t gpioInt = GPIO_IntGet();				//Read IF register
	GPIO_IntClear(gpioInt);							//Set IFC register
	for(int i = 0; i < 40000; i++);     		 	//Button Debounce

	RTOS_ERR err;

	if(gpioInt & (1 << BTN0_INT)) {					//Set button 0 pressed and released
		OSFlagPost(&btnEventFlags, BTN0_PRESS, OS_OPT_POST_FLAG_SET, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
	}
	else if(gpioInt & (1 << BTN1_INT)) {			//Set button 1 pressed and released
		OSFlagPost(&btnEventFlags, BTN1_PRESS, OS_OPT_POST_FLAG_SET, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
	}

	OSIntExit();									//Exit ISR
}

void LEDToggleTmrCallback(void* tmr, void* args) {
	RTOS_ERR err;
	OSFlagPost(&ledWarnFlags, TOGGLE_WARN_LED, OS_OPT_POST_FLAG_SET, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
}

