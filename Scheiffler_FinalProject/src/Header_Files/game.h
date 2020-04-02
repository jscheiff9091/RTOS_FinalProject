/*
 * game.h
 *
 *  Created on: Mar 30, 2020
 *      Author: Jacob S
 */

#ifndef GAME_H_
#define GAME_H_

#include "slider.h"

// ----- Macros -----
#define VEHST_FLG_CLR		0

// ----- Type Definitions -----
typedef struct {
	Direction_t vehDir;
	uint16_t xPos;
	uint16_t yPos;
	uint16_t speed;
	//uint16_t angle; //????
}VehSt_T;

// ----- Global Variables
extern VehSt_T vehState;

// ----- Function Prototypes -----
/// @brief 		Callback for the timer which signals for the road generation task to unblock
///
/// @param[in] 	Pointer to the timer which has expired
/// @param[in] 	Pointer to the timer arguments
void Game_RoadGenerationCallback(void* tmr, void* t_args);

#endif /* GAME_H_ */
