/*
 * gpio.h
 *
 *  Created on: Jan 23, 2020
 *      Author: Jacob S
 */

#ifndef GPIO_H_
#define GPIO_H_

// ----- Included Files ------
#include <stdint.h>
#include <kernel/include/os.h>
#include "main.h"

// ----- Macros ------
#define LED0_PORT 					gpioPortF
#define LED0_PIN					4u
#define LED1_PORT					gpioPortF
#define LED1_PIN					5u

#define BTN0_PORT					gpioPortF
#define BTN0_PIN					6u
#define BTN1_PORT					gpioPortF
#define BTN1_PIN					7u
#define BTN_PORT 					gpioPortF
#define BTN0						0u
#define BTN1						1u
#define BTN0_BIT					0x40
#define BTN1_BIT					0x80
#define BTN0_INT					4
#define BTN1_INT					6
#define BTN_RISE_EN					true
#define	BTN_RISE_DIS				false
#define BTN_FALL_EN					true
#define BTN_FALL_DIS				false

#define LED_DEFAULT					0
#define PULLUP						1

#define LED_DRV_TASK_PRIO 			19u
#define LED_DRV_STACK_SIZE			1000u
#define	LED_DRV_TASK_DLY			100u

#define LED_DRV_Q_SIZE				10u

#define BTN0_PRESS					(1 << 0u)
#define BTN1_PRESS					(1 << 1u)
#define BTN_FLG_ANY					(BTN0_PRESS | BTN1_PRESS)
#define BTN_FLG_NONE				0

#define LED_WARN_CLR_FLAGS			0
#define LED_WARN_SPD_VIOLATION		(1 << 0u)
#define LED_WARN_CLR_SPD_VIOLATION	(1 << 1u)
#define LED_WARN_TRN_VIOLATION		(1 << 2u)
#define	LED_WARN_CLR_TRN_VIOLATION	(1 << 3u)
#define LED_WARN_ALL				(LED_WARN_SPD_VIOLATION | LED_WARN_CLR_SPD_VIOLATION | LED_WARN_TRN_VIOLATION | LED_WARN_CLR_TRN_VIOLATION)

#define NO_TIMEOUT				0

// ----- Typedefs ------
/// @brief enumeration to track possible button states
typedef enum
{
	GPIO_BTNReleased,
	GPIO_BTNPressed
}GPIO_BTNState_t;

/// @brief enumeration to track possible LED actions from buttons and touch slider
typedef enum
{
	LED_BOTH_OFF,
	LED0_ON,
	LED1_ON
}LED_Action_t;

/// @brief enumeration to track states of the LEDs
typedef enum
{
	LED_OFF,
	LED_ON
}LED_State_t;

/// @brief enumeration to identify the source of the LED message
typedef enum
{
	SliderTaskMessage,
	ButtonTaskMessage
}GPIO_LEDTaskMsgSource_t;

/// @brief structure which holds LED Driver Task Messages
typedef struct
{
	GPIO_LEDTaskMsgSource_t msgSource;
	LED_Action_t ledAction;
}GPIO_LEDTaskMsg_t;

/// @brief structure to hold speed setpoint data
typedef struct
{
	int speed;
	int num_inc;
	int num_dec;
}GPIO_SpeedSetPT_t;


// ----- Global Variables ------


// ----- Function Prototypes ------
/// @brief Initialize LED0 and LED1 on the Pearl Gecko starter kit
///
void GPIO_InitLEDs();


/// @brief Initialize BTN0 and BTN1 on the Pearl Gecko starter kit
///
void GPIO_InitBTNs();


/// @brief Get State of external button switch
///
/// @param[in] external switch number
///
/// @return the state of the external switch
GPIO_BTNState_t GPIO_GetBTNState(uint8_t btn);


/// @brief Get desired action of the button selections
///
/// @param[in] state of button 0
/// @param[in] state of button 1
///
/// @return the selection of LEDs the buttons wish to drive
LED_Action_t GPIO_GetButtonAction(GPIO_BTNState_t btn0State, GPIO_BTNState_t btn1State);


/// @brief Turn LEDs on/off based off of desired actions of slider and buttons
///
/// @param[in] desired LED state indicated by the external button switches
/// @param[in] desired LED state indicated by the touch slider
void SetLEDs(LED_Action_t btn_action, LED_Action_t sld_action);


/// @brief ISR for the even gpio interrupts
///
void GPIO_EVEN_IRQHandler(void);


#endif /* SRC_HEADER_FILES_GPIO_H_ */
