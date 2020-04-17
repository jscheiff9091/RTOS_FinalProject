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
#define LCD_TMR_PERIOD			1u
#define NEW_SHIFT_MESSAGE		1 << 0
#define GAME_OVER				1 << 1
#define LCD_FLAG_ANY			(NEW_SHIFT_MESSAGE | GAME_OVER)
#define ANGLE90					90
#define CAR_XMIN				59
#define CAR_XMAX				69
#define CAR_YMIN				113
#define CAR_YMAX				128
#define LCD_CAR_X				64
#define LCD_CAR_Y				113
#define STR_XPOS				5
#define STR_YPOS				120

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

#endif /* LCD_H_ */
