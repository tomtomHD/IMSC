/*
 * CircuitDataAndCorrectionValues.h
 *
 * Created: 01.04.2022 13:19:00
 *  Author: Thomas Thurner
 */ 


#ifndef CIRCUITDATAANDCORRECTIONVALUES_H_
#define CIRCUITDATAANDCORRECTIONVALUES_H_


//----------------------------------------------------
// Correction of Measurements:
																				// (this value corresponds to the imaginary part of the measured values (DFT value) with the measuring resistance extrapolated to infinity (does NOT correspond to disabled MUX) )
#define SHUNT_RESIST_IN_REAL					470//(22000)								// 470 Ohm in real
#define SHUNT_RESIST_CORRECTION_FACTOR			(1.0)//(1.4)								// Factor for the correction of the shunt resistance, which appears to the circuit [old@22k resistors: (1.15)]
																					// (is likely caused by the suboptimal (non ideal) current-voltage (Operational)Amplifier (--> virtual ground is not held exactly) )
#define SHUNT_RESIST	(SHUNT_RESIST_IN_REAL * SHUNT_RESIST_CORRECTION_FACTOR)		// Shunt resistor, which appears to the circuit

//----------------------------------------------------


#endif /* CIRCUITDATAANDCORRECTIONVALUES_H_ */