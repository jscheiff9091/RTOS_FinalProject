#ifndef CMU_H
#define CMU_H
//***********************************************************************************
// Include files
//***********************************************************************************
#include "cmu.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************

//Initialize
void CMU_InitHFClocks() {

	CMU_HFXOInit_TypeDef hfxoInit = CMU_HFXOINIT_DEFAULT;  //Not used
	CMU_HFXOInit(&hfxoInit);

	/* Switch HFCLK to HFRCO and disable HFRCO */
	CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
	CMU_OscillatorEnable(cmuOsc_HFXO, false, false);
}

//Enable clock to GPIO peripheral
void CMU_RouteGPIOClock() {
	CMU_ClockEnable(cmuClock_GPIO, true);		//Enable clock to GPIO periheral
}

//Enable Clock to the low energy timer
void CMU_LETIMERClockInit() {					//Not Used
	return;
}

#endif
