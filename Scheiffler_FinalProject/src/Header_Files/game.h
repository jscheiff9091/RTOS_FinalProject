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
#include "math.h"
#include "glib.h"

// ----- Macros -----
#define DIR_FLG_CLR			0
#define	DIR_FLG_NONE		0
#define XDIFF_HIGH			10
#define XDIFF_LO			-10
#define R_WIDTH				20
#define GET_XDIFF(x)		rand() % ((x*XDIFF_HIGH) - (x*XDIFF_LO) + 1) + (x*XDIFF_LO)
#define WAYPT_YDIFF			30
#define PI					3.1415926535
#define deg2rad(X)			((X)/180.0)*PI
#define rad2deg(X)			((X)/PI)*180.0
#define NUM_GAME_MODES		2
#define NUM_DIFFICULTIES	3
#define NUM_VEHICLES		3
#define NUM_RESTART_OPTS	2
#define PHYS_UPDATE_RATE	.25
#define VEHST_UPDATE_RATE   .1
#define STD_MU				1
#define VEH_SLIP_TOLERANCE	.9
#define SPORTS_CAR_SPECS	{ .vehicleName = "Sports Car", 	\
	                          .mass = 20,					\
	                          .maxPower = 40,				\
	                          .turnRadius = 20,				\
	                          .vehicleWidth = 1,			\
	                          .dragArea = 1,				\
	                          .tireType = SportsCar }
#define MINI_VAN_SPECS		{ .vehicleName = "Mini Van", 	\
	                          .mass = 40,					\
	                          .maxPower = 20,				\
	                          .turnRadius = 25,				\
	                          .vehicleWidth = 1,			\
	                          .dragArea = 1,				\
	                          .tireType = MiniVan }
#define SEMI_SPECS			{ .vehicleName = "Semi", 		\
	                          .mass = 100,					\
	                          .maxPower = 10,				\
	                          .turnRadius = 30,				\
	                          .vehicleWidth = 1,			\
	                          .dragArea = 1,				\
	                          .tireType = Truck }
// ----- Type Definitions -----
typedef enum {
	SportsCar = 1,
	MiniVan = 2,
	Truck = 3
}Tire_t;

typedef enum {
	Finished,
	SpunOut,
	LeftRoad
}GameResult_t;

typedef enum {
	Easy,
	Medium,
	Hard
}GameDifficulty_t;

typedef enum {
	TimeTrial,
	Survival
}GameMode_t;

typedef enum {
	GameStart,
	GamePlay,
	GameEnd
}GameState_t;

typedef struct {
	Direction_t vehDir;
	double xPos;
	double yPos;
	double circX;
	double circY;
	uint16_t radius;
	double angle;
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
	int16_t accelSd;
	int16_t accelFwd;
	double velocity;
	uint16_t power;
	uint8_t zAccel;
	uint16_t bankAngle;
	uint8_t roll;
}VehPhys_t;

typedef struct {
	char RoadName[20];
	uint8_t roadWidth;
	uint16_t numWayPts;
	WayPtFIFO_t waypoints;
}Road_t;

typedef struct {
	double distance;
	uint16_t sumOfSpeeds;
	uint16_t numSums;
	GameResult_t gameResult;
	OS_TICK startTime;
	uint16_t wayPtsPassed;
}GameStats_t;

typedef struct {
	Tire_t vehicle;
	GameDifficulty_t difficulty;
	GameMode_t gameMode;
}GameSelections_t;

// ----- Global Variables -----
extern VehSt_T vehState;             	/**< Variable to hold the current vehicle state */
extern VehSpecs_t vehSpecs;				/**< Variable to hold the data related to the vehicle selected */
extern VehPhys_t vehPhys;				/**< Variable to hold the vehicle physics data */
extern uint8_t vehSlip;					/**< Slip parameter of the vehicle */
extern Road_t road;						/**< Variable to hold information about the road */
extern Road_t usedRoad;					/**< Road FIFO which holds used waypoints for boundary testing */
extern GameStats_t gameStats;			/**< Variable to track stats to be printed at the end of the game */
extern GameState_t	gameState;			/**< Variable to keep track of the state of the game */
extern GameSelections_t selections;		/**< Variable to hold user selections for the game setup */

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
bool OutsideBoundary(double xPos, double yPos);

/// @brief Check if the vehicle will cross over be off road within the next 30m
///
/// @param[in] Vehicle x position
/// @param[in] Vehicle y position
///
/// @return true will be off road, false will still be on
bool TrendingOut(double xPos, double yPos);

/// @brief user selected their desired game mode
///
/// @param[in] LCD context
void SelectGameMode(GLIB_Context_t* lcdContext);

/// @brief User selects the difficulty of their game
///
/// @param[in] LCD context
void SelectDifficulty(GLIB_Context_t* lcdContext);

/// @brief User select the vehicle they would like to use
///
/// @param[in] LCD context
void SelectVehicle(GLIB_Context_t* lcdContext);

/// @brief Select game restart option
///
/// @param[in] LCD Context
GameState_t GameResetSelect(GLIB_Context_t* lcdContext);

/// @brief Initalize road and and used road FIFOs
void InitRoadFIFOs(void);

/// @brief Restore state variables to power on state
void ResetStateVars(void);

/// @bried Reset state variable to state at beginning of previous game
void ReloadStateVars(void);

#endif /* GAME_H_ */
