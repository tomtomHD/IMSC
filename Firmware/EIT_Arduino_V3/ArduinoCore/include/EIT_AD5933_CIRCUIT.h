/* 
* EIT_AD5933_CIRCUIT.h
*
* Created: 19.02.2021 09:56:45
* Author: Thurner Thomas
*/


#ifndef __EIT_AD5933_CIRCUIT_H__
#define __EIT_AD5933_CIRCUIT_H__

#include "MUX16_CIRCUIT.h"
#include "AD5933s.h"
#include <EEPROM.h>

#define START_FREQ		(99000)
#define FREQ_INCR		(0)
#define NUM_INCR		(10)
#define SINGLESTIMULATIONDATALENGTH (13)

#define LEAKAGE_CORRECTION (true)

// Voltages:
#define SIGNAL_VPP		(1.98)			// V
// Currents:
#define STIMULATION_CURRENT (1)			// mA
// Resistors:
#define REF_RESIST		(3300)			// Ohm
#define SHUNT_RESIST	(330)			// Ohm
#define REF_RESIST		(22000)			// Ohm
#define REF_RESIST_1	(22000)			// Ohm
#define REF_RESIST_2	(22000)			// Ohm
// Gain-factors:
#define GAIN_R			(1.035e-07)						// Gain factor for calculating Impedance with standard circuit, calibrated with 'REF_RESIST = 3300 Ohm'


class EIT_AD5933_CIRCUIT
{
//variables
private:
	
	// Max. measured DFT-value:
	float maxMeasuredDFT;
	
	// Gain factor for DFT_value(max)/Ue(max)  [DFT_value..calculated result of DFT in the AD5933; Ue..Output voltage of AD5933]:
	float gain_u;
	
	// Last stimulation current measured:
	float lastMeasuredStimCurrent;
	
	// Array to detect leakage currents:
	float leakageCurrentCompensationFactorA[ELECTRODE_NUMBER][ELECTRODE_NUMBER] = {{1}};
	/*	At these measures, there is always only one pin/electrode with voltage (all others are highZ):
		All voltages get measured for each single pin stimulation and a Factor f is created to compensate this impact and saved here:
	   {	{ [@Pin0 -> high] f0,f1,f2,f3,f4,...fN},
			{ [@Pin1 -> high] f0,f1,f2,f3,f4,...fN},
				...,
			{ [@PinN -> high] f0,f1,f2,f3,f4,...fN}	}
	*/
		
public:
	// Trinity MUX-object pointer:
	MUX16_CIRCUIT* trinityMUX;
	
	// AD5933-object pointer:
	AD5933s* ad5933ciruit;

//functions
public:
	// Constructor:
	EIT_AD5933_CIRCUIT();
	
	// Destructor:
	~EIT_AD5933_CIRCUIT();
	
	// Initialize Module for EIT:
	bool initEITmodule();
	
	// Set electrode Initial conditions (separate as far as possible)
	void setElectrodeInitCond();
	
	// Refresh max. measurable DFT-Value int 'maxMeasuredDFT'
	void getMaxMeasuredDFT();
	
	// Set Module in Measurement-Mode:
	void beginMeasure();
	
	// Measure all electrodes, results are going to be written into result:
	void measureAll(float result[ELECTRODE_NUMBER], bool debugMode = false );
	
	// Set Module in Standby-Mode:
	void endMeasure();
	
	// Prepare stimulation-electrodes:
	void prepareStimulation(int8_t stimPattern[ELECTRODE_NUMBER]);
	
	// Acquire data via Stimulation pattern and Measurement pattern:
	void acquireData(int8_t stimPattern[ELECTRODE_NUMBER], int8_t measPattern[SINGLESTIMULATIONDATALENGTH][ELECTRODE_NUMBER], float result[SINGLESTIMULATIONDATALENGTH], bool debugMode = false );
	
	// Refresh leakageCurrentCompensationFactorA [Array] to compensate leakage currents / their impact:
	void getLeakageImpact(int8_t stimPattern[ELECTRODE_NUMBER]);
	
	// Leakage correction:
	void correctLeakingData(int8_t stimPattern[ELECTRODE_NUMBER], float result[SINGLESTIMULATIONDATALENGTH]);
		
	// Get last measured Stimulation current:
	float getLastMeasuredStimCurrent();
	


}; //EIT_AD5933_CIRCUIT

#endif //__EIT_AD5933_CIRCUIT_H__
