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
#define LCD_DISP_TASK_PRIO		21u
#define LCD_DISP_STACK_SIZE		1000u

#define LCD_TMR_PERIOD			1u

#define DIRECTION_STRINGS		{"Hard Left", \
								 "Left", \
								 "Straight", \
								 "Right", \
								 "Hard Right" }

// ----- Global Variables -----
extern OS_TCB LCDDispTaskTCB;                          		/**< Task control block for the LCD Display Task  */
extern CPU_STK LCDDisplayTaskStack[LCD_DISP_STACK_SIZE];	/**< LCD Display task stack  */

extern OS_TMR LCDDispTmr;									/**< LCD Display task timer */

// ----- Function Prototypes -----

/// @brief Task to update LCD display with speed and direction information
///
/// @param[in] Task arguments
void LCDDisplayTask(void* p_args);

/// @brief Timer callback for the LCD Display task timer
///
/// @param[in] pointer to the LCD task timer
/// @param[in] pointer to cal back arguments
void LCDTmrCallback(void* p_tmr, void* p_args);

#endif /* LCD_H_ */
