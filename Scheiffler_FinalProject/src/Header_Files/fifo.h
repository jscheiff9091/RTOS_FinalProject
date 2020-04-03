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
#include <stdint.h>

// ----- Macros -----
#define FIFO_CAPACITY 		30


// ----- Data types -----
/// @brief data type stored in the Setpoint fifo
struct WayPt_t
{
	int xPos;
	int yPos;
	struct WayPt_t* next;
	//struct WayPt_t* prev;  //?
};

/// @brief data type to store Setpoint fifo
typedef struct
{
	struct WayPt_t* head;
	struct WayPt_t* tail;
	uint8_t currWayPts;
	uint8_t totalWayPts;
}WayPtFIFO_t;


// ----- Function Prototypes -----
/// @brief append entry to a setpoint fifo
///
/// @param[in] pointer to fifo
/// @param[in] state of button 0
/// @param[in] state of button 1
void FIFO_Append(WayPtFIFO_t* fifo, int xDiff);


/// @brief look at the first entry in the queue
///
/// @param[in] pointer to the fifo
///
/// @return structure of the first entry in the queue
struct WayPt_t* FIFO_Peek(WayPtFIFO_t* fifo);


/// @brief remove first entry in the queue
///
/// @param[in] pointer to the fifo
void FIFO_Pop(WayPtFIFO_t* fifo);

/// @brief check if fifo is empty
///
/// @param[in] pointer to the fifo
bool FIFO_IsEmpty(WayPtFIFO_t* fifo);


#endif /* FIFO_H_ */
