#ifndef MAIN_H
#define MAIN_H

// ----- Include files -----

// ----- Macros -----
//#define PART1
//#define PART2
//#define uCProbe


/* Start Task Macros */
#define START_TASK_PRIO 	23u
#define START_STACK_SIZE	1000u

#define IDLE_TASK_PRIO		22u
#define IDLE_STACK_SIZE		1000u

#define VEH_MON_TASK_PRIO	20u
#define VEH_MON_STACK_SIZE	1000u

#define SPD_SETPT_FLAG		(1 << 0)
#define VEH_DIR_FLAG		(1 << 1)
#define	VEH_TURNTM_FLAG		(1 << 2)
#define VEH_MON_SET_FLAGS	(SPD_SETPT_FLAG | VEH_DIR_FLAG | VEH_TURNTM_FLAG)
#define VEH_MON_CLR_FLAGS	0
#define VEH_TURN_TIMEOUT	50


// ----- Type Definitions -----


// ----- Global Variables -----
extern OS_TCB startTaskTCB;							/**< Start task control block variable */
extern CPU_STK startTaskStack[START_STACK_SIZE];	/**< Start task stack */
extern OS_FLAG_GRP vehMonFlags;						/**< Vehicle monitor task event flag group */

// ----- Function Prototypes -----
/// @brief Task to create all other tasks and initialize kernel
void StartTask(void* p_args);


/// @brief Task which is scheduled if all other tasks are blocked/waiting
void IdleTask(void* p_args);

/// @brief 	Task which monitors the speed and direction of the vehicle
///			to allow LCD display task and LED driver task to output
///			accurate information
///
/// @param[in] Task arguments
void VehicleMonitorTask(void* p_args);

/// @brief Task which is called when the vehicle turn timeout expires
///
/// @param[in] pointer to the timer which expired
/// @param[in] arguments
void VehicleTurnTimeout(void* tmr, void* p_args);

/// @brief this task performs initialization tasks for the chip
///	Including initializing clock sources and reference voltages.
void KeithGInit(void);

#endif
