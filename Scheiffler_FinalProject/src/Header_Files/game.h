/*
 * game.h
 *
 *  Created on: Mar 30, 2020
 *      Author: Jacob S
 */

#ifndef GAME_H_
#define GAME_H_

#include "slider.h"
#include "fifo.h"

// ----- Macros -----
#define DIR_FLG_CLR			0
#define	DIR_FLG_NONE		0

#define WAYPT_YDIFF			5

// ----- Type Definitions -----
typedef enum {
	Truck,
	Tourism,
	Performance
}Tire_t;

typedef struct {
	Direction_t vehDir;
	uint16_t xPos;
	uint16_t yPos;
	uint16_t speed;
	//uint16_t angle; //????
}VehSt_T;

typedef struct {
	char vehicleName[20];
	uint16_t mass;
	uint16_t maxPower;
	uint16_t turnRadius;
	uint16_t vehicleWidth;

	uint16_t dragArea;
	Tire_t tireType;
}VehSpecs_t;

typedef struct {
	uint16_t acceleration;
	uint16_t velocity;
	uint16_t power;
	uint8_t zAccel;
	uint16_t bankAngle;
	uint8_t roll;
}VehPhys_t;

typedef struct {
	char RoadName[20];
	uint8_t roadWidth;
	WayPtFIFO_t waypoints;
}Road_t;

// ----- Global Variables
extern VehSt_T vehState;             	/**< Variable to hold the current vehicle state */
extern VehSpecs_t vehSpecs;				/**< Variable to hold the data related to the vehicle selected */
extern VehPhys_t vehPhys;				/**< Variable to hold the vehicle physics data */
extern uint8_t vehSlip;					/**< Slip parameter of the vehicle */

// ----- Function Prototypes -----
/// @brief 		Callback for the timer which signals for the road generation task to unblock
///
/// @param[in] 	Pointer to the timer which has expired
/// @param[in] 	Pointer to the timer arguments
void Game_RoadGenerationCallback(void* tmr, void* t_args);

#endif /* GAME_H_ */
