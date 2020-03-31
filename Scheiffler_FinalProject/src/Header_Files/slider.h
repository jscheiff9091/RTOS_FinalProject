/*
 * slider.h
 *
 *  Created on: Jan 23, 2020
 *      Author: Jacob S
 */

#ifndef SLIDER_H_
#define SLIDER_H_

// ----- Included Files ------
#include <kernel/include/os.h>
#include "gpio.h"

// ----- Macros ------
#define POS0					0
#define POS1					1
#define POS2					2
#define POS3					3

#define CNT_ZERO			    0
#define NO_DLY           		0
#define SLD_TIMER_CNT			2


// ----- Typedefs ------
/// @brief data type to keep track of slider state
typedef enum
{
	SLD_Released,
	SLD_Pressed
}SLD_SliderPressedState_t;

/// @brief data type to indicate slide of interest on the slider
typedef enum
{
	SLD_LeftSide,
	SLD_RightSide
}SLD_SliderSide_t;

/// @brief datat type to holde the current direction of the vehicle
typedef enum
{
	HardLeft,
	Left,
	Straight,
	Right,
	HardRight
}Direction_t;

typedef struct
{
	Direction_t dir;
	int leftCnt;
	int rightCnt;
}SLD_Direction_t;

// ----- Global Variables ------
//extern SLD_SliderPressedState_t sld_rightSideState;			/**< Variable to hold the state of the right side of the slider */
//extern SLD_SliderPressedState_t sld_leftSideState;   		/**< Variable to hold the state of the left side of the slider */


extern SLD_Direction_t vehicleDir;							/**< State variable to keep track of the current vehicle direction */

// ----- Function Prototypes ------
/// @brief Initialize the slider and slider state variables
void SLD_Init(void);


/// @brief Check if side of the slider is pressed
///
/// @param[in] side of slider which is to be monitored
///
/// @return state of the side of the slider of interest
SLD_SliderPressedState_t SLD_IsPressed(SLD_SliderSide_t side);


/// @brief Determine LED action desired from the slider input
///
/// @param[in] left side of touch slider state variable
/// @param[in] right side of touch slider state variable
///
/// @return Desired state of the LEDs based on slider states
LED_Action_t SLD_GetSLDAction(SLD_SliderPressedState_t sld_leftSideState, SLD_SliderPressedState_t sld_rightSideState);


/// @brief Function called when the timer used to trigger touch slider readings expires
///
/// @param[in] pointer to the timer
/// @param[in] pointer to any arguments
void SLD_TimerCallback(void* p_tmr, void* p_args);


/// @brief Get direction of the vehicle as indicated by the touch slider
///
/// @return	Direction of the vehicle
Direction_t SLD_GetDirection(void);

#endif /* SRC_HEADER_FILES_SLIDER_H_ */
