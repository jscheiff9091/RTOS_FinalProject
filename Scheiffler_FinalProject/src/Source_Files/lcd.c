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
	double angle, xChng, yChng;

	OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);  	//Get copy of the current vehicle angle
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	angle = vehState.angle;
	OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	if(angle > ANGLE90) {													//Set change variables
		angle = 180.0 - angle;
		xChng = -1.0 * cos(deg2rad(angle));
		yChng = sin(deg2rad(angle));
	}
	else {
		xChng = 1.0*cos(deg2rad(angle));
		yChng = 1.0*sin(deg2rad(angle));
	}

	//Draw line
	GLIB_drawLine(lcdContext, LCD_CAR_X, LCD_CAR_Y, LCD_CAR_X + 10*xChng, LCD_CAR_Y - 10*yChng);
}

//Update the Waypoint array and waypoints if requested
uint8_t DrawWaypoints(GLIB_Context_t* lcdContext, struct WayPt_t* wayPtArray, uint8_t size, bool draw) {
	RTOS_ERR err;
	CPU_TS timestamp;
	double xPos, yPos, deltaX, deltaY;
	uint8_t cpSize = size;
	uint8_t sub = 0;
	struct WayPt_t* headPtr;

	OSMutexPend(&vehStLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);	//Get copies of current vehicle position
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	xPos = vehState.xPos;
	yPos = vehState.yPos;
	OSMutexPost(&vehStLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	for(int i = 0; i < cpSize; i++) {												//Update current values in the array
		if(wayPtArray[i].yPos < yPos) {											//Check if car has passed that waypoint
			sub += 1;
			size--;
		}
		else {
			wayPtArray[i-sub] = wayPtArray[i];										//Shift waypoint if car has passed any waypoints
			if(draw) {																//If drawing
				deltaX = wayPtArray[i].xPos - xPos;									//get difference between car and waypoint positions
				deltaY = wayPtArray[i].yPos - yPos;
				GLIB_drawLineH(lcdContext, (int32_t)LCD_CAR_X+deltaX-(road.roadWidth/2), (int32_t)LCD_CAR_Y-deltaY, (int32_t)LCD_CAR_X+deltaX+(road.roadWidth/2));	//draw waypoint
			}
		}
	}

	OSMutexPend(&usedRdLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);	//Get Used waypoints lock
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	OSMutexPend(&wayPtLock, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);	//Get upcoming waypoints lock
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	headPtr = FIFO_Peek(&road.waypoints);
	while(headPtr != NULL && (headPtr->yPos - yPos) < LCD_CAR_Y) {					//While waypoints still fit on the screen add them to the waypoint array
		deltaX = headPtr->xPos - xPos;												//get difference between car and waypoint positions
		deltaY = headPtr->yPos - yPos;
		GLIB_drawLineH(lcdContext, (int32_t)LCD_CAR_X+deltaX-(road.roadWidth/2), (int32_t)LCD_CAR_Y-deltaY, (int32_t)LCD_CAR_X+deltaX+(road.roadWidth/2));  //draw waypoint

		wayPtArray[size] = *headPtr;												//Insert waypoint into current waypoints array
		size++;
		FIFO_Append(&usedRoad.waypoints, headPtr->xPos, false);						//Add waypoint to used waypoint fifo
		FIFO_Pop(&road.waypoints);													//Take it out of upcoming waypoint fifo
		headPtr = FIFO_Peek(&road.waypoints);										//Check next waypoint
	}

	OSMutexPost(&usedRdLock, OS_OPT_POST_NONE, &err);								//Release locks
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	OSMutexPost(&wayPtLock, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	return size;
}

//Print Velocity
void PrintVehicleState(GLIB_Context_t* lcdContext) {
	RTOS_ERR err;
	CPU_TS timestamp;
	double velocity;
	char velStr[20];

	OSMutexPend(&physTupLk, 0, OS_OPT_PEND_BLOCKING, &timestamp, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	velocity = vehPhys.velocity;
	OSMutexPost(&physTupLk, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	sprintf(velStr, "Vel: %d", ((uint16_t)velocity));
	GLIB_drawString(lcdContext, velStr, strlen(velStr), STR_XPOS, STR_YPOS, false);
}

// Print game over status message
void PrintGameOverStatus(GLIB_Context_t* lcdContext) {
	RTOS_ERR err;
	char msg[100];
	char* gameEndStatusMsg[] = GAME_OVER_STATUS_MESSAGES;

	sprintf(msg, "GAME OVER!");
	GLIB_drawString(lcdContext, msg, strlen(msg), GO_STR_XPOS, GO_STR_YPOS, false);

	if(gameStats.gameResult != Finished) {
		sprintf(msg, gameEndStatusMsg[gameStats.gameResult]);
		GLIB_drawString(lcdContext, msg, strlen(msg), GR_STR_XPOS, GR_STR_YPOS, false);
	}
	else {
		sprintf(msg, gameEndStatusMsg[gameStats.gameResult]);
		GLIB_drawString(lcdContext, msg, strlen(msg), FINISHED_STR_XPOS, FINISHED_STR_YPOS, false);
	}

	sprintf(msg, "Total Distance: %d", (int)gameStats.distance);
	GLIB_drawString(lcdContext, msg, strlen(msg), DIS_STR_XPOS, DIS_STR_YPOS, false);

	sprintf(msg, "Avg. Speed: %d", gameStats.sumOfSpeeds/gameStats.numSums);
	GLIB_drawString(lcdContext, msg, strlen(msg), SPD_STR_XPOS, SPD_STR_YPOS, false);

	if(selections.gameMode == TimeTrial) {
		OS_TICK endTime = OSTimeGet(&err);
		sprintf(msg, "Time Elapsed: %d", (int)((endTime - gameStats.startTime)*.001));
		GLIB_drawString(lcdContext, msg, strlen(msg), TIME_STR_XPOS, TIME_STR_YPOS, false);
	}
	else {
		sprintf(msg, "WayPts Passed: %d", gameStats.wayPtsPassed);
		GLIB_drawString(lcdContext, msg, strlen(msg), NUMWAYPT_STR_XPOS, NUMWAYPT_STR_YPOS, false);
	}
}

//Print game details at the beginning of the game
void PrintGameInit(GLIB_Context_t* lcdContext) {
	char msg[100];
	char* gameDiffStr[] = DIFFICULTY_STRINGS;

	GLIB_clear(lcdContext);
	sprintf(msg, "Press B1 to Start");
	GLIB_drawString(lcdContext, msg, strlen(msg), START_STR_XPOS, START_STR_YPOS, false);

	sprintf(msg, "Car: %s", vehSpecs.vehicleName);
	GLIB_drawString(lcdContext, msg, strlen(msg), STATS_XPOS, CAR_TYPE_Y_POS, false);

	sprintf(msg, "Car Power: %d", vehSpecs.maxPower);
	GLIB_drawString(lcdContext, msg, strlen(msg), STATS_XPOS, CAR_POWER_Y_POS, false);

	sprintf(msg, "Turn Radius: %d", vehSpecs.turnRadius);
	GLIB_drawString(lcdContext, msg, strlen(msg), STATS_XPOS, CAR_RADIUS_Y_POS, false);

	sprintf(msg, "Car Mass: %d", vehSpecs.mass);
	GLIB_drawString(lcdContext, msg, strlen(msg), STATS_XPOS, CAR_MASS_Y_POS, false);

	sprintf(msg, "Num. WayPts: %d", road.numWayPts);
	GLIB_drawString(lcdContext, msg, strlen(msg), STATS_XPOS, NUM_WAYPTS_Y_POS, false);

	sprintf(msg, "Difficulty: %s", gameDiffStr[selections.difficulty]);
	GLIB_drawString(lcdContext, msg, strlen(msg), STATS_XPOS, GM_DIFF_Y_POS, false);
	DMD_updateDisplay();
}

// Print game mode options
void PrintGameModeSelection(GLIB_Context_t* lcdContext) {
	char msg[100];

	GLIB_clear(lcdContext);
	sprintf(msg, "Select Game Mode!");
	GLIB_drawString(lcdContext, msg, strlen(msg), GM_SELECT_XPOS, GM_SELECT_YPOS, false);

	sprintf(msg, "Time Trial");
	GLIB_drawString(lcdContext, msg, strlen(msg), TT_STR_XPOS, TT_STR_YPOS, false);

	sprintf(msg, "Survival");
	GLIB_drawString(lcdContext, msg, strlen(msg), SURVIVE_STR_XPOS, SURVIVE_STR_YPOS, false);

	GLIB_drawCircle(lcdContext, SELECT_DOT_XPOS, SELECT_DOT_YPOS + selections.gameMode*10, SELECT_DOT_RADIUS);
	DMD_updateDisplay();
}

// Print game difficulty selection screen
void PrintGameDifficultySelect(GLIB_Context_t* lcdContext) {
	char msg[100];

	GLIB_clear(lcdContext);
	sprintf(msg, "Select Difficulty!");
	GLIB_drawString(lcdContext, msg, strlen(msg), DIFF_SELECT_XPOS, DIFF_SELECT_YPOS, false);

	sprintf(msg, "Easy");
	GLIB_drawString(lcdContext, msg, strlen(msg), EASY_STR_XPOS, EASY_STR_YPOS, false);

	sprintf(msg, "Medium");
	GLIB_drawString(lcdContext, msg, strlen(msg), MED_STR_XPOS, MED_STR_YPOS, false);

	sprintf(msg, "Hard");
	GLIB_drawString(lcdContext, msg, strlen(msg), HARD_STR_XPOS, HARD_STR_YPOS, false);

	GLIB_drawCircle(lcdContext, SELECT_DOT_XPOS, SELECT_DOT_YPOS + selections.difficulty*10, SELECT_DOT_RADIUS);
	DMD_updateDisplay();
}

// Print vehicle select screen
void PrintVehicleSelect(GLIB_Context_t* lcdContext, uint8_t ctr) {
	char msg[100];

	GLIB_clear(lcdContext);
	sprintf(msg, "Select Vehicle!");
	GLIB_drawString(lcdContext, msg, strlen(msg), VEH_SELECT_XPOS, VEH_SELECT_YPOS, false);

	sprintf(msg, "Sports Car");
	GLIB_drawString(lcdContext, msg, strlen(msg), SPORTS_STR_XPOS, SPORTS_STR_YPOS, false);

	sprintf(msg, "Mini Van");
	GLIB_drawString(lcdContext, msg, strlen(msg), VAN_STR_XPOS, VAN_STR_YPOS, false);

	sprintf(msg, "Semi");
	GLIB_drawString(lcdContext, msg, strlen(msg), SEMI_STR_XPOS, SEMI_STR_YPOS, false);

	GLIB_drawCircle(lcdContext, SELECT_DOT_XPOS, SELECT_DOT_YPOS + ctr*10, SELECT_DOT_RADIUS);
	DMD_updateDisplay();
}

//Print restart options
void PrintGameRestartSelect(GLIB_Context_t* lcdContext, uint8_t ctr) {
	char msg[100];

	GLIB_clear(lcdContext);
	sprintf(msg, "Retry Level");
	GLIB_drawString(lcdContext, msg, strlen(msg), RESRT_OPT1_XPOS, RESRT_OPT1_YPOS, false);

	sprintf(msg, "Chg. Gm. Settings");
	GLIB_drawString(lcdContext, msg, strlen(msg), RESRT_OPT2_XPOS, RESRT_OPT2_YPOS, false);

	GLIB_drawCircle(lcdContext, SELECT_DOT_XPOS, SELECT_DOT_YPOS + ctr*10, SELECT_DOT_RADIUS);
	DMD_updateDisplay();
}









