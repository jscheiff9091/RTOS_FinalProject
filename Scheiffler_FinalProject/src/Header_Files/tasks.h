/*
 * tasks.h
 *
 *  Created on: Mar 30, 2020
 *      Author: Jacob S
 */

#ifndef TASKS_H_
#define TASKS_H_

#include  <kernel/include/os.h>

//----- Macros ----
// Task Macros
#define START_TASK_PRIO 		20u
#define START_STACK_SIZE		1000u

#define IDLE_TASK_PRIO			19u   //Low
#define IDLE_STACK_SIZE			1000u

#define RDGEN_TASK_PRIO			18u   //Low
#define RDGEN_STACK_SIZE		1000u
#define RDGEN_TMR_CNT			100u

#define DIR_TASK_PRIO 			13u
#define DIR_STACK_SIZE			1000u
#define DIR_TIME_DLY			100u
#define DIR_TMR_CNT				1

#define PHYSMOD_TASK_PRIO		12u
#define PHYSMOD_STACK_SIZE		1000u

#define SPD_TASK_PRIO 			11u
#define SPD_STACK_SIZE			1000u
#define SPD_TASK_DLY			100u

#define LED_TASK_PRIO 			17u    //Low
#define LED_STACK_SIZE			1000u
#define	LED_TASK_DLY			100u

#define VEHST_TASK_PRIO			14u
#define VEHST_STACK_SIZE		1000u

#define GMMON_TASK_PRIO			15u
#define GMMON_STACK_SIZE		1000u

#define LCD_TASK_PRIO			16u     //Higher than LED
#define LCD_STACK_SIZE			1000u

// Miscellaneous macros
#define CNT_ZERO			    0
#define NO_DLY           		0
#define	PEND_TIMEOUT			10u
#define PEND_NB_TIMEOUT			1u


//----- Global Variables -----
extern OS_TCB startTaskTCB;                      	/**< Task control block for the start task */
extern CPU_STK startTaskStack[START_STACK_SIZE];    /**< Task stack for the start task */
extern OS_FLAG_GRP ledWarnFlags;					/**< Flags to trigger the LED driver task */
extern OS_FLAG_GRP btnEventFlags;					/**< Flags to signal to the button task that a button has been pressed */
extern OS_FLAG_GRP lcdFlags;						/**< Flags to notify the LCD task of vehicle state */
extern OS_MUTEX vehStLock;							/**< Mutex to protect the vehicle state object */
extern OS_MUTEX physTupLk;							/**< Mutex to protect the physics tuple */
extern OS_MUTEX usedRdLock;							/**< Mutex used to protect the used road waypoint FIFO */
extern OS_MUTEX wayPtLock;							/**< Mutex to protect upcoming waypoints FIFO */


//----- Function Prototypes -----
/// @brief Task to launch all other tasks
///
/// @param[in] pointer to arguments
void StartTask(void* p_args);

/// @brief Task which generates waypoints for the road ahead.
///
/// @param[in] pointer to arguments
void RoadGenerateTask(void* p_args);

/// @brief Task to monitor the direction of the car turn as indicated by the touch slider
///
/// @param[in] pointer to task arguments
void DirectionUpdateTask(void* p_args);

/// @brief Task which updates the physics model of the car.
///
/// @param[in] pointer to arguments
void PhysicsModelTask(void* p_args);

/// @brief Task which updates the speed state variable.
///
/// @param[in] pointer to arguments
void SpeedUpdateTask(void* p_args);

/// @brief Task to control warning lights (direction and speed)
///
/// @param[in] pointer to arguments
void LEDWarningTask(void* p_args);

/// @brief Task to update the vehicle state for use in game monitoring
///
/// @param[in] pointer to arguments
void VehicleStateTask(void* p_args);

/// @brief Task to monitor the condition of the games, signal to the LEDs and LCD accordingly, start and stop the game as needed
///
/// @param[in] pointer to arguments
void GameMonitorTask(void* p_args);

/// @brief Task to update LCD display with speed and direction information
///
/// @param[in] Task arguments
void LCDDisplayTask(void* p_args);

/// @brief Task which is scheduled if all other tasks are blocked/waiting
///
/// @param[in] Task arguments
void IdleTask(void* p_args);

#endif /* TASKS_H_ */
