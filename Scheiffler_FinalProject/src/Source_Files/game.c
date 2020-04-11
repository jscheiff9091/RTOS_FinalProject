/*
 * game.c
 *
 *  Created on: Mar 30, 2020
 *      Author: Jacob S
 */

// ----- Included Files -----
#include "game.h"
#include "tasks.h"

#include <kernel/include/os.h>
#include <common/include/rtos_utils.h>
#include <math.h>
#include <tgmath.h>


// ----- Declaration of Global Variables ------
VehSt_T vehState;
VehSpecs_t vehSpecs;
VehPhys_t vehPhys;
uint8_t vehSlip;
Road_t road;
Road_t usedRoad;
GameStats_t gameStats;


// ----- Function definitions -----
/* Update position of the vehicle */
void UpdateVehiclePosition(void) {
	RTOS_ERR err;
	CPU_TS timestamp;
	double distance, deltaX, deltaY;

	OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);		//Lock vehicle state variable
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	distance = vehPhys.velocity * VEHST_UPDATE_RATE;							//Calculate the distance traveled along the vehicle vector since last update
	gameStats.sumOfSpeeds += (uint16_t) vehPhys.velocity;
	OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	gameStats.distance += (uint16_t) distance;
	gameStats.numSums += 1;

	if(vehState.angle < 90) {
		deltaX = distance * cos((double)vehState.angle);					//Calculate and add change in x Position
		vehState.xPos += (int16_t) deltaX;
		deltaY = distance * sin((double)vehState.angle);					//Calculate and add change in y position
		vehState.yPos += (int16_t) deltaY;
	}
	else if(vehState.angle > 90) {
		deltaX = distance * cos((double)(180 - vehState.angle));			//Calculate and subtract change in x position
		vehState.xPos -= (int16_t) deltaX;
		deltaY = distance * sin((double)(180 - vehState.angle));			//Calculate and add change in y position
		vehState.yPos += (int16_t) deltaY;
	}
	else {
		vehState.yPos += (int16_t) distance;								//Skip unnecessary floating point ops, since change only in y
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

	deltaX = vehState.xPos - vehState.circX;								//Get distance from center point of turn footprint to vehicle position
	deltaY = vehState.yPos - vehState.circY;

	if((deltaX > 0 && deltaY > 0) || (deltaX < 0 && deltaY < 0)) {		    //1st and 3rd quadrants
		vehState.angle = 90 + atan(deltaX/deltaY);
	}
	else if((deltaX > 0 && deltaY < 0) || (deltaX < 0 && deltaY > 0)) {     //2nd and 4th quadrants
		vehState.angle = 90 - atan(-1*(deltaX/deltaY));
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
		force = STD_MU * vehSpecs.tireType * vehSpecs.mass * 9.8;
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

		if((vehState.vehDir == Left || vehState.vehDir == HardLeft) & vehState.angle > 90) {			//Quadrant 1, calculate center point of circular footprint
			vehState.circX = vehState.xPos - vehState.radius * cos(180 - vehState.angle);
			vehState.circY = vehState.yPos - vehState.radius * sin(180 - vehState.angle);
		}
		else if((vehState.vehDir == Right || vehState.vehDir == HardRight) & vehState.angle <= 90) {    //Quadrant 2
			vehState.circX = vehState.xPos + vehState.radius * cos(90 - vehState.angle);
			vehState.circY = vehState.yPos - vehState.radius * sin(90 - vehState.angle);
		}
		else if((vehState.vehDir == Right || vehState.vehDir == HardRight) & vehState.angle > 90) {		//Quadrant 3
			vehState.circX = vehState.xPos + vehState.radius * cos(vehState.angle - 90);
			vehState.circY = vehState.yPos + vehState.radius * sin(vehState.angle - 90);
		}
		else if((vehState.vehDir == Left || vehState.vehDir == HardLeft) & vehState.angle <= 90) {		//Quadrant 4
			vehState.circX = vehState.xPos - vehState.radius * cos(90 - vehState.angle);
			vehState.circY = vehState.yPos + vehState.radius * sin(90 - vehState.angle);
		}
	}
	OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}
