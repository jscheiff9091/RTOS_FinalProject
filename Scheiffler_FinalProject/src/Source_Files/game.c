/*
 * game.c
 *
 *  Created on: Mar 30, 2020
 *      Author: Jacob S
 */

// ----- Included Files -----
#include "game.h"
#include "tasks.h"
#include "lcd.h"
#include "glib.h"

#include <kernel/include/os.h>
#include <common/include/rtos_utils.h>
#include <math.h>
//#include <tgmath.h>


// ----- Declaration of Global Variables ------
VehSt_T vehState;
VehSpecs_t vehSpecs;
VehPhys_t vehPhys;
Road_t road;
Road_t usedRoad;
GameStats_t gameStats;
GameState_t	gameState;
GameSelections_t selections;


// ----- Function definitions -----
/* Update position of the vehicle */
void UpdateVehiclePosition(void) {
	RTOS_ERR err;
	CPU_TS timestamp;
	double distance, deltaX, deltaY;


	OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	distance = vehPhys.velocity * VEHST_UPDATE_RATE;							//Calculate the distance traveled along the vehicle vector since last update
	gameStats.sumOfSpeeds += (uint16_t) vehPhys.velocity;
	OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	gameStats.distance += distance;
	gameStats.numSums += 1;

	OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);		//Lock vehicle state variable
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	if(vehState.angle < 90) {
		deltaX = distance * cos(deg2rad(vehState.angle));					//Calculate and add change in x Position
		vehState.xPos += deltaX;
		deltaY = distance * sin(deg2rad(vehState.angle));					//Calculate and add change in y position
		vehState.yPos += deltaY;
	}
	else if(vehState.angle > 90) {
		deltaX = distance * cos(deg2rad(180-vehState.angle));			//Calculate and subtract change in x position
		vehState.xPos -=  deltaX;
		deltaY = distance * sin(deg2rad(180-vehState.angle));			//Calculate and add change in y position
		vehState.yPos += deltaY;
	}
	else {
		vehState.yPos += distance;								//Skip unnecessary floating point ops, since change only in y
	}

	OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);						//Release vehicle state lock
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}

/* Update angle of the vehicle vector */
void UpdateVehicleAngle(void) {
	RTOS_ERR err;
	CPU_TS timestamp;
	double deltaX, deltaY;

	OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);		//Acquire vehicle state lock
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	deltaX =  vehState.xPos - vehState.circX;						//Get distance from center point of turn footprint to vehicle position
	deltaY =  vehState.yPos - vehState.circY;

	if((deltaX > 0 && deltaY > 0) || (deltaX < 0 && deltaY < 0)) {		    //1st and 3rd quadrants
		vehState.angle = rad2deg((PI/2) + atan(deltaY/deltaX));
	}
	else if((deltaX > 0 && deltaY < 0) || (deltaX < 0 && deltaY > 0)) {     //2nd and 4th quadrants
		vehState.angle = rad2deg((PI/2) - atan(-1.0*(deltaY/deltaX)));
	}

	if(vehState.angle > 180) {                                              //Vehicle cannot go backwards in my game. Just like my philosophy of life: Never look back.
		vehState.angle = 180;
	}
	else if(vehState.angle < 0) {
		vehState.angle = 0;
	}
	OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);						//Release vehicle state lock
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}
/* Update the slip percentage */
void UpdateSlip(void) {
	RTOS_ERR err;
	CPU_TS timestamp;

	double force, slip;

	OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	if(vehState.radius == 0) {												//Car is moving straight
		vehState.prcntSlip = 0;
	}
	else {																	//Vehicle turning, compute percent slip
		OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		force = STD_MU * vehSpecs.tireType * vehSpecs.mass * 9.8 * .05;
		slip = sqrt(pow((double)(vehPhys.power/vehPhys.velocity), 2) + pow((double)(vehPhys.velocity/vehState.radius) , 2));
		OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		vehState.prcntSlip = slip / force;									//Record fraction of slip
	}

	OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}

/* Calculate the circular footprint of the vehicle turn */
void CalculateCircle(void) {
	RTOS_ERR err;
	CPU_TS timestamp;

	OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	if(vehState.vehDir == Straight) {																	//Clear circular footprint if car is not turning
		vehState.radius = 0;
		vehState.circX = 0;
		vehState.circY = 0;
	}
	else {
		if(vehState.vehDir == Left || vehState.vehDir == Right) {										//Update vehicle turn circular footprint radius
			vehState.radius = 2 * vehSpecs.tireType * vehSpecs.turnRadius;
		}
		else if(vehState.vehDir == HardLeft || vehState.vehDir == HardRight) {
			vehState.radius = vehSpecs.tireType * vehSpecs.turnRadius;
		}

		if((vehState.vehDir == Left || vehState.vehDir == HardLeft) && vehState.angle > 90) {			//Quadrant 1, calculate center point of circular footprint
			vehState.circX = vehState.xPos - vehState.radius * cos(deg2rad(vehState.angle - ANGLE90));
			vehState.circY = vehState.yPos - vehState.radius * sin(deg2rad(vehState.angle - ANGLE90));
		}
		else if((vehState.vehDir == Right || vehState.vehDir == HardRight) && vehState.angle <= 90) {   //Quadrant 2
			vehState.circX = vehState.xPos + vehState.radius * cos(deg2rad(ANGLE90 - vehState.angle));
			vehState.circY = vehState.yPos - vehState.radius * sin(deg2rad(ANGLE90 - vehState.angle));
		}
		else if((vehState.vehDir == Right || vehState.vehDir == HardRight) && vehState.angle > 90) {	//Quadrant 3
			vehState.circX = vehState.xPos + vehState.radius * cos(deg2rad(vehState.angle - ANGLE90));
			vehState.circY = vehState.yPos + vehState.radius * sin(deg2rad(vehState.angle - ANGLE90));
		}
		else if((vehState.vehDir == Left || vehState.vehDir == HardLeft) && vehState.angle <= 90) {		//Quadrant 4
			vehState.circX = vehState.xPos - vehState.radius * cos(deg2rad(ANGLE90 - vehState.angle));
			vehState.circY = vehState.yPos + vehState.radius * sin(deg2rad(ANGLE90 - vehState.angle));
		}
	}
	OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}

/* Check if the vehicle is off the road */
bool OutsideBoundary(double xPos, double yPos) {
	RTOS_ERR err;
	CPU_TS timestamp;

	double x1, y1, x2, y2, ySol;
	struct WayPt_t* temp;

	OSMutexPend(&usedRdLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);		//Get position of the previous and next waypoints
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	x1 = usedRoad.waypoints.head->xPos;
	y1 = usedRoad.waypoints.head->yPos;
	x2 = usedRoad.waypoints.head->next->xPos;
	y2 = usedRoad.waypoints.head->next->yPos;
	temp = usedRoad.waypoints.head->next;
	OSMutexPost(&usedRdLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	if(temp == NULL) {
		return false;
	}

	double slope = (y2 - y1) / (x2 - x1);
	//Check if the vehicle has crossed the left boundary
	ySol = slope * (xPos - (x1-(road.roadWidth/2))) + y1;	//(equation of the left road boundary)
	if(slope > 0 && yPos > ySol) {
		return true;
	}
	else if(slope < 0 && yPos < ySol) {
		return true;
	}

	//Check if the vehicle has crossed the right boundary
	ySol = slope * (xPos - (x1+(road.roadWidth/2))) + y1;	//(equation of the right road boundary)
	if(slope > 0 && yPos < ySol) {
		return true;
	}
	else if(slope < 0 && yPos > ySol) {
		return true;
	}

	return false;
}

/* Check if vehicle trending off the road */
bool TrendingOut(double xPos, double yPos) {
	RTOS_ERR err;
	CPU_TS timestamp;

	double angle, xSol, x2, y2, slope;

	OSMutexPend(&usedRdLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);		//Get position of the previous and next waypoints
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	x2 = (double)usedRoad.waypoints.head->next->xPos;
	y2 = (double)usedRoad.waypoints.head->next->yPos;
	OSMutexPost(&usedRdLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);		//Get current angle of the vehicle path vector
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	angle = vehState.angle;
	OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	if(angle > ANGLE90) {
		angle = 180 - angle;
		slope = -1*tan(deg2rad(angle));
	}
	else {
		slope = tan(deg2rad(angle));
	}
	xSol = (y2 - yPos) / slope + xPos;													//Calculate position of the vehicle if it were to continue to the next waypoint along this vector

	if(xSol < (x2 - (road.roadWidth/2)) || xSol > (x2 + (road.roadWidth/2))) {
		return true;
	}
	return false;
}

/* Let user select their desired game mode */
void SelectGameMode(GLIB_Context_t* lcdContext) {
	RTOS_ERR err;
	OS_FLAGS flags;
	CPU_TS timestamp;

	PrintGameModeSelection(lcdContext);
	while(1) {
		flags = OSFlagPend(&btnEventFlags, BTN_FLG_ANY, 0u, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		OSFlagPost(&btnEventFlags, flags, OS_OPT_POST_FLAG_CLR, &err);

		if(flags & BTN0_PRESS) {
			selections.gameMode = (selections.gameMode + 1) % NUM_GAME_MODES;
			PrintGameModeSelection(lcdContext);
		}
		else if(flags & BTN1_PRESS) {
			break;
		}
	}
}

/* Let user select the difficulty of their game */
void SelectDifficulty(GLIB_Context_t* lcdContext) {
	RTOS_ERR err;
	OS_FLAGS flags;
	CPU_TS timestamp;

	PrintGameDifficultySelect(lcdContext);
	while(1) {
		flags = OSFlagPend(&btnEventFlags, BTN_FLG_ANY, 0u, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		OSFlagPost(&btnEventFlags, flags, OS_OPT_POST_FLAG_CLR, &err);

		if(flags & BTN0_PRESS) {
			selections.difficulty = (selections.difficulty + 1) % NUM_DIFFICULTIES;
			PrintGameDifficultySelect(lcdContext);
		}
		else if(flags & BTN1_PRESS) {
			break;
		}
	}

	//Waypoint offsets
	OSMutexPend(&usedRdLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);	//Get Used waypoints lock
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	OSMutexPend(&wayPtLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);			//Get upcoming waypoints lock
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	road.roadWidth  = (NUM_DIFFICULTIES - selections.difficulty - 1) * 5 + 15;
	if(selections.gameMode == TimeTrial) {
		usedRoad.waypoints.totalWayPts = (selections.difficulty + 1) * 25;
		road.waypoints.totalWayPts = (selections.difficulty + 1) * 25;
		road.numWayPts = road.waypoints.totalWayPts + 5;
	}
	else {
		usedRoad.waypoints.totalWayPts = 0xFFFF;
		road.waypoints.totalWayPts = 0xFFFF;
		road.numWayPts = 0xFFFF;
	}
	OSMutexPost(&usedRdLock, OS_OPT_POST_NONE, &err);								//Release locks
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	OSMutexPost(&wayPtLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}

/* Let user select their car */
void SelectVehicle(GLIB_Context_t* lcdContext) {
	uint8_t ctr = 0;
	RTOS_ERR err;
	CPU_TS timestamp;
	OS_FLAGS flags;

	PrintVehicleSelect(lcdContext, ctr);
	while(1) {
		flags = OSFlagPend(&btnEventFlags, BTN_FLG_ANY, 0u, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		OSFlagPost(&btnEventFlags, flags, OS_OPT_POST_FLAG_CLR, &err);

		if(flags & BTN0_PRESS) {
			ctr = (ctr + 1) % NUM_VEHICLES;
			PrintVehicleSelect(lcdContext, ctr);
		}
		else if(flags & BTN1_PRESS) {
			break;
		}
	}

	selections.vehicle = ctr + 1;
	switch(selections.vehicle) {
		case SportsCar:
			vehSpecs = (VehSpecs_t) SPORTS_CAR_SPECS;
			break;
		case MiniVan:
			vehSpecs = (VehSpecs_t) MINI_VAN_SPECS;
			break;
		case Truck:
			vehSpecs = (VehSpecs_t) SEMI_SPECS;
			break;
	}
 }

GameState_t GameResetSelect(GLIB_Context_t* lcdContext) {
	CPU_TS timestamp;
	RTOS_ERR err;
	OS_FLAGS flags;
	uint8_t ctr;

	PrintGameRestartSelect(lcdContext, ctr);
	while(1) {
		flags = OSFlagPend(&btnEventFlags, BTN_FLG_ANY, 0u, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		OSFlagPost(&btnEventFlags, flags, OS_OPT_POST_FLAG_CLR, &err);

		if(flags & BTN0_PRESS) {
			ctr = (ctr + 1) % NUM_RESTART_OPTS;
			PrintGameRestartSelect(lcdContext, ctr);
		}
		else if(flags & BTN1_PRESS) {
			break;
		}
	}

	if(ctr) {
		ResetStateVars();

		OSMutexPend(&gameStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		gameState = GameStart;
		OSMutexPost(&gameStLock, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		return GameStart;
	}
	else {
		ReloadStateVars();

		OSMutexPend(&gameStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		gameState = GamePlay;
		OSMutexPost(&gameStLock, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		return GamePlay;
	}
}

// Initialize the waypoint FIFOs
void InitRoadFIFOs(void) {
	RTOS_ERR err;
	CPU_TS timestamp;

	OSMutexPend(&usedRdLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	OSMutexPend(&wayPtLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	road.waypoints.totalWayPts = 5;
	usedRoad.waypoints.totalWayPts = 5;

	for(int i = 0; i < 5; i++) {
		FIFO_Append(&road.waypoints, 0, true);
	}

	OSMutexPost(&usedRdLock, OS_OPT_POST_NONE, &err);								//Release locks
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	OSMutexPost(&wayPtLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}

// Initialize waypoint FIFOs
void ResetStateVars(void) {
	vehState = (VehSt_T){ .vehDir = Straight,
						  .xPos = 0,
						  .yPos = 0,
	                      .circX = 0,
	                      .circY = 0,
	                      .radius = 0,
						  .angle = 90,
	                      .prcntSlip = 0 };
	vehPhys = (VehPhys_t) { .accelSd = 0,
	                        .accelFwd = 0,
	                        .velocity = 0,
	                        .power = 0,
	                        .zAccel = 0,
	                        .bankAngle = 0,
	                        .roll = 0 };
	gameStats = (GameStats_t) { .distance = 0,
	                            .sumOfSpeeds = 0,
	                            .numSums = 0,
	                            .gameResult = Finished,
								.startTime = 0,
								.wayPtsPassed = 0};

	RTOS_ERR err;
	CPU_TS timestamp;
	OSMutexPend(&wayPtLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	FIFO_Clear(&road.waypoints);
	OSMutexPost(&wayPtLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	InitRoadFIFOs();
}

void ReloadStateVars(void) {
	RTOS_ERR err;
	CPU_TS timestamp;

	OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	vehState = (VehSt_T){ .vehDir = Straight,
						  .xPos = 0,
						  .yPos = 0,
						  .circX = 0,
						  .circY = 0,
						  .radius = 0,
						  .angle = 90,
						  .prcntSlip = 0 };
	OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	vehPhys = (VehPhys_t) { .accelSd = 0,
							.accelFwd = 0,
							.velocity = 0,
							.power = 0,
							.zAccel = 0,
							.bankAngle = 0,
							.roll = 0 };
	OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	gameStats = (GameStats_t) { .distance = 0,
								.sumOfSpeeds = 0,
								.numSums = 0,
								.gameResult = Finished,
								.startTime = 0,
								.wayPtsPassed = 0};

	OSMutexPend(&wayPtLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	FIFO_Clear(&road.waypoints);
	OSMutexPost(&wayPtLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	InitRoadFIFOs();

	OSMutexPend(&usedRdLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	OSMutexPend(&wayPtLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	road.roadWidth  = (NUM_DIFFICULTIES - selections.difficulty - 1) * 5 + 15;
	if(selections.gameMode == TimeTrial) {
		usedRoad.waypoints.totalWayPts = (selections.difficulty + 1) * 25;
		road.waypoints.totalWayPts = (selections.difficulty + 1) * 25;
		road.numWayPts = road.waypoints.totalWayPts + 5;
	}
	else {
		usedRoad.waypoints.totalWayPts = 0xFFFF;
		road.waypoints.totalWayPts = 0xFFFF;
		road.numWayPts = 0xFFFF;
	}

	OSMutexPost(&usedRdLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	OSMutexPost(&wayPtLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}
