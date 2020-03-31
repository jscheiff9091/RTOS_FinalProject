#ifndef _CMU_H
#define _CMU_h

// Include files
#include "em_cmu.h"

// Macros


//Type definitions


// Global Variables


// Function Prototypes
/// @brief this function enables the high frequency clock
///
void CMU_InitHFClocks();

/// @brief this function enables the clock to the gpio peripheral
///
void CMU_RouteGPIOClock();

///@brief this clock enables and configures the low energy timer
///
void CMU_LETIMERClockInit();

#endif
