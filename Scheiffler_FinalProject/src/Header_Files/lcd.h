/*
 * lcd.h
 *
 *  Created on: Feb 28, 2020
 *      Author: Jacob S
 */

#ifndef LCD_H_
#define LCD_H_
#include  <kernel/include/os.h>

// ----- Macros -----
#define LCD_TMR_PERIOD			1u
#define NEW_SHIFT_MESSAGE		1 << 0
#define GAME_OVER				1 << 1

#define DIRECTION_STRINGS		{"Hard Left", \
								 "Left", \
								 "Straight", \
								 "Right", \
								 "Hard Right" }

// ----- Global Variables -----


// ----- Function Prototypes -----
/// @brief Timer callback for the LCD Display task timer
///
/// @param[in] pointer to the LCD task timer
/// @param[in] pointer to cal back arguments
void LCDTmrCallback(void* p_tmr, void* p_args);

#endif /* LCD_H_ */
