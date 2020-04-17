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

#define XDIFF_HIGH			5
#define XDIFF_LO			-5
#define GET_XDIFF			rand() % (XDIFF_HIGH - XDIFF_LO + 1) + XDIFF_LO
#define WAYPT_YDIFF			5

#define PHYS_UPDATE_RATE	.25
#define VEHST_UPDATE_RATE   .1

#define STD_MU				2
#define VEH_SLIP_TOLERANCE	.85

// ----- Type Definitions -----
typedef enum {
	Performace = 1,
	Tourism = 2,
	Truck = 3
}Tire_t;

typedef enum {
	Finished,
	SpunOut,
	LeftRoad
}GameResult_t;

typedef struct {
	Direction_t vehDir;
	int16_t xPos;
	int16_t yPos;
	int16_t circX;
	int16_t circY;
	uint16_t radius;
	uint8_t angle;
	double prcntSlip;
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
	uint16_t accelSd;
	uint16_t accelFwd;
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

typedef struct {
	uint16_t distance;
	uint16_t sumOfSpeeds;
	uint16_t numSums;
	GameResult_t gameResult;
}GameStats_t;

// ----- Global Variables -----
extern VehSt_T vehState;             	/**< Variable to hold the current vehicle state */
extern VehSpecs_t vehSpecs;				/**< Variable to hold the data related to the vehicle selected */
extern VehPhys_t vehPhys;				/**< Variable to hold the vehicle physics data */
extern uint8_t vehSlip;					/**< Slip parameter of the vehicle */
extern Road_t road;						/**< Variable to hold information about the road */
extern Road_t usedRoad;					/**< Road FIFO which holds used waypoints for boundary testing */
extern GameStats_t gameStats;			/**< Variable to track stats to be printed at the end of the game */

// ----- Function Prototypes -----
/// @brief Update the position of the vehicle
void UpdateVehiclePosition(void);

/// @brief Update the angle of the vehicle travel vector
void UpdateVehicleAngle(void);

/// @brief Update the slip percentage of the vehicle
void UpdateSlip(void);

/// @brief Create the circular footprint for a vehicle turn
void CalculateCircle(void);

/// @brief Check if the vehicle has left the road
///
/// @param[in] Vehicle x position
/// @param[in] Vehicle y position
///
/// @return true if off road, false if still on
bool OutsideBoundary(int16_t xPos, int16_t yPos);

/// @brief Check if the vehicle will cross over be off road within the next 30m
///
/// @param[in] Vehicle x position
/// @param[in] Vehicle y position
///
/// @return true will be off road, false will still be on
bool TrendingOut(int16_t xPos, int16_t yPos);
#endif /* GAME_H_ */
