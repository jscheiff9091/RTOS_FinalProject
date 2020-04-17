/*
 * lcd.c
 *
 *  Created on: Feb 28, 2020
 *      Author: Jacob S
 */

#include "lcd.h"
#include "slider.h"
#include "tasks.h"
#include "game.h"
#include "fifo.h"

#include "glib.h"

#include <kernel/include/os.h>
#include <common/include/rtos_utils.h>
#include <stdio.h>
#include <string.h>
#include <math.h>


//---- Function definitions -----
//Draw direction vector
void DrawVehicleDirLine(GLIB_Context_t* lcdContext) {
	RTOS_ERR err;
	CPU_TS timestamp;
	uint8_t angle;
	int8_t xChng, yChng;

	OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);  	//Get copy of the current vehicle angle
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	angle = vehState.angle;
	OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	if(angle > ANGLE90) {													//Set change variables
		angle -= ANGLE90;
		xChng = -1 * cos((double)angle);
		yChng = sin((double)angle);
	}
	else {
		xChng = cos((double)angle);
		yChng = sin((double)angle);
	}

	//Draw line
	GLIB_drawLine(lcdContext, LCD_CAR_X, LCD_CAR_Y, LCD_CAR_X + 10*xChng, LCD_CAR_Y - 10*yChng);
}

//Update the Waypoint array and waypoints if requested
uint8_t DrawWaypoints(GLIB_Context_t* lcdContext, struct WayPt_t* wayPtArray, uint8_t size, bool draw) {
	RTOS_ERR err;
	CPU_TS timestamp;
	int16_t xPos, yPos, deltaX, deltaY;
	uint8_t cpSize = size;
	uint8_t sub = 0;
	struct WayPt_t* headPtr;

	OSMutexPend(&vehStLock, PEND_TIMEOUT, OS_OPT_PEND_BLOCKING, &timestamp, &err);	//Get copies of current vehicle position
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	xPos = vehState.xPos;
	yPos = vehState.yPos;
	OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	for(int i = 0; i < cpSize; i++) {												//Update current values in the array
		if(wayPtArray[i].yPos <= yPos) {											//Check if car has passed that waypoint
			sub += 1;
			size--;
		}
		else {
			wayPtArray[i-sub] = wayPtArray[i];										//Shift waypoint if car has passed any waypoints
			if(draw) {																//If drawing
				deltaX = wayPtArray[i].xPos - xPos;									//get difference between car and waypoint positions
				deltaY = wayPtArray[i].yPos - yPos;
				GLIB_drawLineH(lcdContext, LCD_CAR_X+deltaX-XDIFF_HIGH, LCD_CAR_Y-deltaY, LCD_CAR_X+deltaX+XDIFF_HIGH);	//draw waypoint
			}
		}
	}

	OSMutexPend(&usedRdLock, PEND_TIMEOUT, OS_OPT_PEND_BLOCKING, &timestamp, &err);	//Get Used waypoints lock
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	OSMutexPend(&wayPtLock, PEND_TIMEOUT, OS_OPT_PEND_BLOCKING, &timestamp, &err);	//Get upcoming waypoints lock
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	headPtr = FIFO_Peek(&road.waypoints);
	while(headPtr != NULL && (headPtr->yPos - yPos) < LCD_CAR_Y) {					//While waypoints still fit on the screen add them to the waypoint array
		deltaX = headPtr->xPos - xPos;												//get difference between car and waypoint positions
		deltaY = headPtr->yPos - yPos;
		GLIB_drawLineH(lcdContext, LCD_CAR_X+deltaX-XDIFF_HIGH, LCD_CAR_Y-deltaY, LCD_CAR_X+deltaX+XDIFF_HIGH);	//draw waypoint

		wayPtArray[size] = *headPtr;												//Insert waypoint into current waypoints array
		size++;
		FIFO_Append(&usedRoad.waypoints, headPtr->xPos, false);						//Add waypoint to used waypoint fifo
		FIFO_Pop(&road.waypoints);													//Take it out of upcoming waypoint fifo
		headPtr = FIFO_Peek(&road.waypoints);										//Check next waypoint
	}

	OSMutexPost(&wayPtLock, OS_OPT_POST_NONE, &err);								//Release locks
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	OSMutexPost(&usedRdLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	return size;
}

//Print Velocity
void PrintVehicleState(GLIB_Context_t* lcdContext) {
	RTOS_ERR err;
	CPU_TS timestamp;
	uint16_t velocity;
	char* velStr;

	OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	velocity = vehPhys.velocity;
	OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	sprintf(velStr, "Vel: %ud", velocity);
	GLIB_drawString(lcdContext, velStr, strlen(velStr), STR_XPOS, STR_YPOS, false);
}
