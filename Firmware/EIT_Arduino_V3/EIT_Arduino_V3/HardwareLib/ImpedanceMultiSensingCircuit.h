/* 
* ImpedanceMultiSensingCircuit.h
*
* Created: 08.03.2022 23:06:42
* Author: Thomas Thurner
*/


#ifndef __IMPEDANCEMULTISENSINGCIRCUIT_H__
#define __IMPEDANCEMULTISENSINGCIRCUIT_H__

#include "MUX16_CIRCUIT.h"
#include "AD5933s.h"
#include "MCP23017.h"
#include "CircuitDataAndCorrectionValues.h"


//#define DEBUG_MODE




// AD5933 Options:
#define USE_INTERNAL_CLOCK		(false)
#define START_FREQ		(100000)
#define FREQ_INCR		(0)
#define NUM_INCR		(10)
#define MEASURE_NUMBER_PER_STIMULATION (13)
#define SETTLING_TIME_CYCLES (5)	// Waiting cycles until data is collected for the DFT
#define HP_SETTLING_TIME	(3)		// Wait ? ms until Signal is settled to steady state (HP)

// MCP23017 Addresses:
#define MCP23017_MUX_ADDR  (0)		// Address of MCP23017 controlling the Multiplexer
#define MCP23017_MODE_ADDR (1)		// Address of MCP23017 controlling the measurement-Mode, LEDs and GPIOs

// Voltages:
#define SIGNAL_VPP		(1.98)			// V

// Currents:
#define STIMULATION_CURRENT (1)			// mA

// Compensation tolerance:
#define COMPENSATIONTOLERANCE	(10)	// Tolerance; Outside this tolerance, compensation Values get updated and restored into EEPROM
#define COMPENSATION_DETERMINATION_SAFETY_REPETITIONS (3)		// Defines how often the same Result for compensation values has to be measured until they get accepted as valid

// Modes
#define MODE_Data_acquisition_finished	(0)
#define MODE_Magnitude_of_Voltage		(1)
#define MODE_Complex_Voltage			(2)
#define MODE_Magnitude_of_Impedance		(3)
#define MODE_Complex_Impedance			(4)
#define MODE_Magnitude_of_other_Data	(5)
#define MODE_Complex_other_Data			(6)
#define MODE_Magnitude_EIT_Data			(7)

// Gain-factors:
#define GAIN_R			(1.035e-07)						// Gain factor for calculating Impedance with standard circuit, calibrated with 'REF_RESIST = 3300 Ohm'

// Maximum HIGH-Z Values:
#define HIGH_Z_REAL_neg		(-50)
#define HIGH_Z_IMAG_pos		(300)

// ADG1611 Analog Switch:
#define ADG1611_LOG_ON		(0)
#define ADG1611_LOG_OFF		(HIGH)

// IO-Expander MODES PINOUT:
#define PIN_MODE_Z_MEAS_IN	(0)
#define PIN_MODE_V_MEAS_IN	(3)
#define PIN_MODE_NEEDLE_IN	(2)
#define PIN_MODE_NEEDLE_OUT	(1)
#define PIN_LED_RED			(4)
#define PIN_LED_YELLOW		(5)
#define PIN_LED_GREEN		(6)
#define PIN_LED_BLUE		(7)
// IO-Expander GPIO-Port-3 PINOUT:
#define GPIO_PORT_3_PIN_0   (0)
#define GPIO_PORT_3_PIN_1   (1)
#define GPIO_PORT_3_PIN_2   (2)
#define GPIO_PORT_3_PIN_3   (3)
#define GPIO_PORT_3_PIN_4   (4)
#define GPIO_PORT_3_PIN_5   (5)
#define GPIO_PORT_3_PIN_6   (6)
#define GPIO_PORT_3_PIN_7   (7)

// Needle Placeholder:
#define NEEDLE				(16)
//--------------------------------------------------------


//--------------------------------------------------------
// Class for LEDs:
class imsc_LED{
private:
	uint8_t pin;
	bool loglow;
	MCP23017* ioExpander;
	bool state;
public:
	imsc_LED(MCP23017* io_Expander, uint8_t pinIdx, bool logicalLow = LOW);
	~imsc_LED();
	void on();
	void off();
	bool getState();
	void overwrite(bool _state, byte* manipulateByteToSet = nullptr);
};

//--------------------------------------------------------

enum directPrintMode { DirectPRINT_NONE, DirectPRINT_MAGNITUDE, DirectPRINT_COMPLEX };
	
//--------------------------------------------------------
class ImpedanceMultiSensingCircuit
{
//variables

public:
	// Measurement-Pattern:
	uint8_t meas_pattern[MEASURE_NUMBER_PER_STIMULATION];
	
	// Stimulation-Pattern:
	uint8_t stim_pattern;	// Pattern
	uint8_t stim_current;	// Value
	
	// Result Vector of EIT (Voltage) measurement:
	float resultVsEIT[MEASURE_NUMBER_PER_STIMULATION] = {0};
		
	// Results (Temporary storage, can be both, Voltage or Impedance, depending of last measurement):
	float result_Real;
	float result_Imag;
	
	// Compensation Values:
	int16_t compensation_Z_Re;
	int16_t compensation_Z_Im;
	
	int16_t compensation_V_Re;
	int16_t compensation_V_Im;
	
	// Measurement Frequency:
	unsigned long measureFrequency;
		
private:
	// Max. measured DFT-value:
	float maxMeasuredDFT;
	
	// Gain factor for Ue(max)/DFT_value(max)  [ Ue..Output voltage of AD5933; DFT_value..calculated result of DFT in the AD5933]:
	float gain_U;
	// Gain factor for Impedance (1/Referenz_Impedance)/Reference_Magnitude:
	float gain_Z;
	
	// Indicator if signal generator is enabled:
	bool sigGenEnabled;
	
	// Last stimulation current measured:
	float lastMeasuredStimCurrent;
	
	// Temporary raw Data:
	float tempRawData[ELECTRODE_NUMBER+NEEDLE_NUMBER]; // (Electrodes + Needle)
	
	// Array to detect leakage currents (temporary):
	float leakageCurrentCompensationFactorA[ELECTRODE_NUMBER+NEEDLE_NUMBER] = {1};
	/*	At these measures, there is always only one pin/electrode with voltage (all others are highZ):
		All voltages get measured for a single pin stimulation and a Factor f is created to compensate the impact of leakage currents:
	    @Pin0 -> high:   {f0,f1,f2,f3,f4,...fN},
	*/
	
	
//objects
public:
		// Trinity MUX-object pointer:
		MUX16_CIRCUIT* trinityMUX; 
		
		// AD5933-object pointer:
		AD5933s* ad5933ciruit;
		
		// IO-Expander for Modes, LED and GPIO:
		MCP23017* mcp23017_ModeExpander;

private:
		// LED- and Modes:
		imsc_LED* redLED;
		imsc_LED* yellowLED;
		imsc_LED* greenLED;
		imsc_LED* blueLED;
		imsc_LED* analogSwitch_SIG_OUT_NEEDLE;
		imsc_LED* analogSwitch_V_MEAS_NEEDLE;
		imsc_LED* analogSwitch_V_MEAS;
		imsc_LED* analogSwitch_Z_MEAS;
		
		
//functions
public:
	// Constructor:
	ImpedanceMultiSensingCircuit();
	
	// Destructor:
	~ImpedanceMultiSensingCircuit();
	
	// Initialize Module for EIT:
	int8_t initIMSC();
	
	// Set electrode Initial conditions (separate as far as possible)
	void setElectrodeInitCond();
	
	// Self Calibration if nothing is connected (checked between Pin 0 and 1):
	void calibrationRoutine();
	
	// Write Modes and LED simultaneously:
	void writeModesAndLeds(byte byteToSet);
	
	// Send Data binary:
	void sendBinaryData(int8_t mode, int8_t fromEl = -1, int8_t toEl = -1, int8_t measEl = -1, float fData1 = NAN, float fData2 = NAN);
	
	// Set Module in Measurement-Mode:
	void beginMeasure();
	
	// Set Module in Standby-Mode (all Disabled):
	void endMeasure();
	
	// Main Measure Parts (Mind to "beginMeasure()" before and "endMeasure()" afterwards):
	float measureImpedance(int8_t fromEl, int8_t toEl, directPrintMode measureImpedance_directPrint_MODE = DirectPRINT_NONE);	// Return Magnitude of Impedance, Resistance and Reactance get stored in result_Real and result_Imag (including shunt Resistance and small Phase shift of System)
	float measureReactance(int8_t fromEl, int8_t toEl, directPrintMode measureReactance_directPrint_MODE = DirectPRINT_NONE);	// Return Reactance,			  Resistance and Reactance get stored in result_Real and result_Imag (including shunt Resistance and small Phase shift of System)
	float measureResistance(int8_t fromEl, int8_t toEl,directPrintMode measureResistance_directPrint_MODE= DirectPRINT_NONE);	// Return Resistance,			  Resistance and Reactance get stored in result_Real and result_Imag (including shunt Resistance and small Phase shift of System)
	
	float measureVoltage(int8_t measuringEl, int8_t stimSig, int8_t stimGnd, directPrintMode measureVoltage_directPrint_MODE, bool correctLeakage = false);
	
	// Disable all Modes (disable all analog switches):
	void disableModes();
	
	// Measure all electrodes, results are going to be written into result (Stimulation or similar must be prepared before):
	void measureAllPrepreparedV(directPrintMode measureAllVolt_directPrint_MODE = DirectPRINT_NONE);

	// Acquire data via Stimulation pattern and Measurement pattern (binary):
	void acquireEITdata(directPrintMode aquireData_directPrint_MODE = DirectPRINT_NONE, bool compensateLeakage = true, bool measureStimCurrent = true);
	
	// Prepare stimulation-electrodes (binary):
	void prepareStimulation();
	
	// Get last measured Stimulation current:
	float getLastMeasuredStimCurrent();
	
	// Get gain_U:
	float getGainU();
	
	// Get gain_Z
	float getGainZ();
	
protected:
private:


	// Measure Compensation Values !IF! nothing is connected:
	void determineCompensationValues();
	
	// Refresh max. measurable DFT-Value int 'maxMeasuredDFT'
	void getMaxMeasuredDFT();

	// Determine GAIN for Impedance Measurement:
	void determineGainZ();
	

	// Refresh leakageCurrentCompensationFactorA [Array] to compensate leakage currents / their impact  (binary):
	void getLeakageImpactVs();
	
	// Leakage correction (binary):
	void correctLeakingDataVs();
	
	ImpedanceMultiSensingCircuit( const ImpedanceMultiSensingCircuit &c );
	ImpedanceMultiSensingCircuit& operator=( const ImpedanceMultiSensingCircuit &c );
	

}; //ImpedanceMultiSensingCircuit



#endif //__IMPEDANCEMULTISENSINGCIRCUIT_H__
