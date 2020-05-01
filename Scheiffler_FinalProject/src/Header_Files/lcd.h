/*
 * lcd.h
 *
 *  Created on: Feb 28, 2020
 *      Author: Jacob S
 */

#ifndef LCD_H_
#define LCD_H_

// ----- Included Files -----
#include <stdbool.h>
#include <stdint.h>
#include "glib.h"
#include "game.h"

// ----- Macros -----
#define LCD_TMR_PERIOD				1u
#define NEW_SHIFT_MESSAGE			1 << 0
#define GAME_OVER					1 << 1
#define LCD_FLAG_ANY				(NEW_SHIFT_MESSAGE | GAME_OVER)
#define ANGLE90						90.0
#define CAR_XMIN					59
#define CAR_XMAX					69
#define CAR_YMIN					113
#define CAR_YMAX					128
#define LCD_CAR_X					64
#define LCD_CAR_Y					113
#define STR_XPOS					5
#define STR_YPOS					120
#define START_STR_XPOS          	20
#define START_STR_YPOS				20
#define GAME_OVER_STATUS_MESSAGES	{ "Finished!", "Spun out :(", "Off course :(" }
#define GO_STR_XPOS					36
#define GO_STR_YPOS					40
#define GR_STR_XPOS					28
#define GR_STR_YPOS                 56
#define DIS_STR_XPOS                10
#define DIS_STR_YPOS                72
#define SPD_STR_XPOS				30
#define SPD_STR_YPOS                88
#define CAR_TYPE_STRINGS			{ "NULL", "Performance", "Tourism", "Truck" }
#define STATS_XPOS					5
#define CAR_TYPE_Y_POS				30
#define CAR_POWER_Y_POS				40
#define CAR_RADIUS_Y_POS			50
#define CAR_MASS_Y_POS				60
#define NUM_WAYPTS_Y_POS			70
#define GM_DIFF_Y_POS				80

// ----- Global Variables -----


// ----- Function Prototypes -----
/// @brief Draw the vehicle direction line
///
/// @param[in] GLIB context which defines the state of the LCD
void DrawVehicleDirLine(GLIB_Context_t* lcdContext);

/// @brief Darw upcoming waypoints
///
/// @param[in] GLIB context which defines the state of the LCD
/// @param[in] Array of waypoints currently on the screen
/// @param[in] Number of elements in the array
/// @param[in] Set to true if drawing waypoints, false if waypoint array just needs to be updated
///
/// @return New size of the waypoint array
uint8_t DrawWaypoints(GLIB_Context_t* lcdContext, struct WayPt_t* wayPtArray, uint8_t size, bool draw);

/// @brief Print velocity and acceleration on the screen
///
/// @param[in] GLIB context which defines the state of the LCD
void PrintVehicleState(GLIB_Context_t* lcdContext);


/// @brief Print Game over status message
///
/// @param[in] GLIB context which defines the state of the LCD
void PrintGameOverStatus(GLIB_Context_t* lcdContext);

/// @brief Print the vehicle and course attributes before starting the game
///
/// @param[in] GLIB context which defines the state of the LCD
void PrintGameInit(GLIB_Context_t* lcdContext);

#endif /* LCD_H_ */
