/*
 * fifo.h
 *
 *  Created on: Feb 28, 2020
 *      Author: Jacob S
 */

#ifndef FIFO_H_
#define FIFO_H_

#include "gpio.h"
#include <stdbool.h>


// ----- Data types -----
/// @brief data type stored in the Setpoint fifo
struct FIFO_SetptNode_t
{
	GPIO_BTNState_t btn0_state;
	GPIO_BTNState_t btn1_state;
	struct FIFO_SetptNode_t* next;
};

/// @brief data type to store Setpoint fifo
typedef struct
{
	struct FIFO_SetptNode_t* head;
	struct FIFO_SetptNode_t* tail;
}FIFO_SetptFIFO_t;


// ----- Function Prototypes -----
/// @brief append entry to a setpoint fifo
///
/// @param[in] pointer to fifo
/// @param[in] state of button 0
/// @param[in] state of button 1
void FIFO_Append(FIFO_SetptFIFO_t* fifo, GPIO_BTNState_t btn0, GPIO_BTNState_t btn1);


/// @brief look at the first entry in the queue
///
/// @param[in] pointer to the fifo
///
/// @return structure of the first entry in the queue
struct FIFO_SetptNode_t* FIFO_Peek(FIFO_SetptFIFO_t* fifo);


/// @brief remove first entry in the queue
///
/// @param[in] pointer to the fifo
void FIFO_Pop(FIFO_SetptFIFO_t* fifo);

/// @brief check if fifo is empty
///
/// @param[in] pointer to the fifo
bool FIFO_IsEmpty(FIFO_SetptFIFO_t* fifo);


#endif /* FIFO_H_ */
