/* 
* ImpedanceMultiSensingCircuit.cpp
*
* Created: 08.03.2022 23:06:42
* Author: Thomas Thurner
*/


#include "ImpedanceMultiSensingCircuit.h"

// ----------------------------------------------------------
imsc_LED::imsc_LED(MCP23017* io_Expander, uint8_t pinIdx, bool logicalLow){
	pin = pinIdx;
	loglow = logicalLow;
	ioExpander = io_Expander;
	ioExpander->pinMode(pin, OUTPUT);
	ioExpander->digitalWrite(pin, logicalLow);
	state = logicalLow;
}

imsc_LED::~imsc_LED(){}
	
void imsc_LED::on(){
	ioExpander->digitalWrite(pin, HIGH != loglow);
	state = HIGH;
}

void imsc_LED::off(){
	ioExpander->digitalWrite(pin, LOW != loglow);
	state = LOW;
}

bool imsc_LED::getState(){
	return state;
}

void imsc_LED::overwrite(bool _state, byte* manipulateByteToSet){
	state = _state;
	if (manipulateByteToSet != nullptr){// Something to Manipulate
		if(loglow == HIGH){
			if(state == HIGH){ *manipulateByteToSet &= ~(1<<pin); }
			else{			   *manipulateByteToSet |=  (1<<pin); }
		}else{
			if(state == HIGH){ *manipulateByteToSet |=  (1<<pin); }
			else{			   *manipulateByteToSet &= ~(1<<pin); }
		}
	}
}
// ----------------------------------------------------------

// Write Modes and LED simultaneously:
void ImpedanceMultiSensingCircuit::writeModesAndLeds(byte byteToSet){
	// Byte to be adopted to get correct states regarding to logical HITH/LOW levels:
	byte adoptedByteToSet = byteToSet;
	// Overwrite LEDs:
	if(byteToSet & 1<<PIN_LED_RED)   {redLED->overwrite(HIGH, &adoptedByteToSet);}   else{redLED->overwrite(LOW, &adoptedByteToSet);}
	if(byteToSet & 1<<PIN_LED_YELLOW){yellowLED->overwrite(HIGH, &adoptedByteToSet);}else{yellowLED->overwrite(LOW, &adoptedByteToSet);}
	if(byteToSet & 1<<PIN_LED_GREEN) {greenLED->overwrite(HIGH, &adoptedByteToSet);} else{greenLED->overwrite(LOW, &adoptedByteToSet);}
	if(byteToSet & 1<<PIN_LED_BLUE)  {blueLED->overwrite(HIGH, &adoptedByteToSet);}  else{blueLED->overwrite(LOW, &adoptedByteToSet);}
	// Overwrite Modes:
	if(byteToSet & 1<<PIN_MODE_NEEDLE_OUT){analogSwitch_SIG_OUT_NEEDLE->overwrite(HIGH, &adoptedByteToSet);}else{analogSwitch_SIG_OUT_NEEDLE->overwrite(LOW, &adoptedByteToSet);}
	if(byteToSet & 1<<PIN_MODE_NEEDLE_IN) {analogSwitch_V_MEAS_NEEDLE->overwrite(HIGH, &adoptedByteToSet);} else{analogSwitch_V_MEAS_NEEDLE->overwrite(LOW, &adoptedByteToSet);}
	if(byteToSet & 1<<PIN_MODE_V_MEAS_IN) {analogSwitch_V_MEAS->overwrite(HIGH, &adoptedByteToSet);}        else{analogSwitch_V_MEAS->overwrite(LOW, &adoptedByteToSet);}
	if(byteToSet & 1<<PIN_MODE_Z_MEAS_IN) {analogSwitch_Z_MEAS->overwrite(HIGH, &adoptedByteToSet);}        else{analogSwitch_Z_MEAS->overwrite(LOW, &adoptedByteToSet);}
	// Send to IO-Expander:
	mcp23017_ModeExpander->writeRegister(MCP23017_GPIOA, adoptedByteToSet);
}

// default constructor
ImpedanceMultiSensingCircuit::ImpedanceMultiSensingCircuit()
{
	trinityMUX = new MUX16_CIRCUIT;
	
	ad5933ciruit = new AD5933s;
	
	mcp23017_ModeExpander = new MCP23017;
	
	maxMeasuredDFT = 32767;
	
	lastMeasuredStimCurrent = 0;
	// Meas-Pattern Initialization:
	meas_pattern[0]  = 3<<4  | 2;  // {0,0,1,-1,0,0,0,0,0,0,0,0,0,0,0,0},
	meas_pattern[1]  = 4<<4  | 3;  // {0,0,0,1,-1,0,0,0,0,0,0,0,0,0,0,0},
	meas_pattern[2]  = 5<<4  | 4;  // {0,0,0,0,1,-1,0,0,0,0,0,0,0,0,0,0},
	meas_pattern[3]  = 6<<4  | 5;  // {0,0,0,0,0,1,-1,0,0,0,0,0,0,0,0,0},
	meas_pattern[4]  = 7<<4  | 6;  // {0,0,0,0,0,0,1,-1,0,0,0,0,0,0,0,0},
	meas_pattern[5]  = 8<<4  | 7;  // {0,0,0,0,0,0,0,1,-1,0,0,0,0,0,0,0},
	meas_pattern[6]  = 9<<4  | 8;  // {0,0,0,0,0,0,0,0,1,-1,0,0,0,0,0,0},
	meas_pattern[7]  = 10<<4 | 9;  // {0,0,0,0,0,0,0,0,0,1,-1,0,0,0,0,0},
	meas_pattern[8]  = 11<<4 | 10; // {0,0,0,0,0,0,0,0,0,0,1,-1,0,0,0,0},
	meas_pattern[9]  = 12<<4 | 11; // {0,0,0,0,0,0,0,0,0,0,0,1,-1,0,0,0},
	meas_pattern[10] = 13<<4 | 12; // {0,0,0,0,0,0,0,0,0,0,0,0,1,-1,0,0},
	meas_pattern[11] = 14<<4 | 13; // {0,0,0,0,0,0,0,0,0,0,0,0,0,1,-1,0},
	meas_pattern[12] = 15<<4 | 14; // {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,-1};
	// Stimulation-Pattern Initialization:
	stim_pattern = 0<<4 | 1; // {-10,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	stim_current = 10; // mA
	
	// Compensation Values:
	compensation_Z_Re = 0;
	compensation_Z_Im = 0;
	
	compensation_V_Re = 0;
	compensation_V_Im = 0;
	
	// Indicator if signal generator is enabled:
	sigGenEnabled = false;
	
	// Measure Frequency:
	measureFrequency = START_FREQ;
	
} //ImpedanceMultiSensingCircuit

// default destructor
ImpedanceMultiSensingCircuit::~ImpedanceMultiSensingCircuit()
{
		delete trinityMUX;
		delete ad5933ciruit;
		delete mcp23017_ModeExpander;
} //~ImpedanceMultiSensingCircuit

// Initialize Module for EIT:
int8_t ImpedanceMultiSensingCircuit::initIMSC(){
	// Initialize Mode Expander:
	if(mcp23017_ModeExpander->begin(MCP23017_MODE_ADDR)){
		// Return Error:
		return 1;
	}
	
	// Initialize Mode- and LED- Pins:
	redLED    = new imsc_LED(mcp23017_ModeExpander, PIN_LED_RED   );
	yellowLED = new imsc_LED(mcp23017_ModeExpander, PIN_LED_YELLOW);
	greenLED  = new imsc_LED(mcp23017_ModeExpander, PIN_LED_GREEN );
	blueLED   = new imsc_LED(mcp23017_ModeExpander, PIN_LED_BLUE  );
	analogSwitch_SIG_OUT_NEEDLE = new imsc_LED(mcp23017_ModeExpander, PIN_MODE_NEEDLE_OUT, ADG1611_LOG_OFF);
	analogSwitch_V_MEAS_NEEDLE  = new imsc_LED(mcp23017_ModeExpander, PIN_MODE_NEEDLE_IN,  ADG1611_LOG_OFF);
	analogSwitch_V_MEAS 		= new imsc_LED(mcp23017_ModeExpander, PIN_MODE_V_MEAS_IN,  ADG1611_LOG_OFF);
	analogSwitch_Z_MEAS 		= new imsc_LED(mcp23017_ModeExpander, PIN_MODE_Z_MEAS_IN,  ADG1611_LOG_OFF);
	// LED flash on:
	redLED->on(); yellowLED->on(); greenLED->on(); blueLED->on(); 
	// Initialize Trinity-MUX
	if(trinityMUX->begin(MCP23017_MUX_ADDR)){
		// ERROR:
		// Red LED:
		redLED->on(); yellowLED->off(); greenLED->off(); blueLED->off();
		return 1;
	}
	// Initialize Pin Electrode configuration by disabling all:
	setElectrodeInitCond();
	// Perform initial configuration. Fail if any one of these fail.
	if (!(ad5933ciruit->reset() &&
		ad5933ciruit->setInternalClock(USE_INTERNAL_CLOCK) &&
		ad5933ciruit->setStartFrequency(measureFrequency) &&
		ad5933ciruit->setIncrementFrequency(FREQ_INCR) &&
		ad5933ciruit->setNumberIncrements(NUM_INCR) &&
		ad5933ciruit->setPGAGain(PGA_GAIN_X1) &&
		ad5933ciruit->setSettlingTimeCycles(SETTLING_TIME_CYCLES)))
	{	
		// ERROR:
		Serial.println("ERROR @ AD5933"); 
		// Red LED:
		redLED->on(); yellowLED->off(); greenLED->off(); blueLED->off();
		return 1;
	}
	// Self Calibration if nothing is connected (checked between Pin 0 and 1):
	calibrationRoutine();
	// Disable Modes:
	disableModes();
	// LED flash on:
	redLED->on(); yellowLED->on(); greenLED->on(); blueLED->on();
	// Wait some time:
	delay(500);
	// LED flash off:
	redLED->off(); yellowLED->on(); greenLED->off(); blueLED->off();
	// Return Success:
	return 0;
}

// Set electrode Initial conditions (separate as far as possible)
void ImpedanceMultiSensingCircuit::setElectrodeInitCond(){
	trinityMUX->prepareSignalMux(-1);
	trinityMUX->prepareMeasureMux(-1);
	trinityMUX->prepareGroundMux(-1);
	trinityMUX->writePreparedChannelsToMux();
}

// Self Calibration if nothing is connected (checked between Pin 0 and 1):
void ImpedanceMultiSensingCircuit::calibrationRoutine(){
	// Start Measurement:
	beginMeasure();
	// Measure Compensation Values !IF! nothing is connected:
	determineCompensationValues();
	// Refresh max. measurable DFT-Value int 'maxMeasuredDFT':
	getMaxMeasuredDFT();
	// Gain factor for Voltage Ue(max)/DFT_value(max)  [Ue..Output voltage of AD5933; DFT_value..calculated result of DFT in the AD5933]:
	gain_U = (SIGNAL_VPP/2)/maxMeasuredDFT;
	// Gain factor for Impedance:
	determineGainZ();
}

// Send Data binary:
void ImpedanceMultiSensingCircuit::sendBinaryData(int8_t mode, int8_t fromEl, int8_t toEl, int8_t measEl, float fData1, float fData2){
	Serial.write((byte) mode);
	if(mode == 0){return;} // Finished to acquire Data? Return!
	Serial.write((byte) fromEl);
	Serial.write((byte) toEl);
	Serial.write((byte) measEl);
	Serial.write((byte*) &fData1, 4);
	if(mode%2 == 1){return;} // Only magnitude? Return!
	Serial.write((byte*) &fData2, 4); // Second part of Complex Data
}

// Measure Impedance:
float ImpedanceMultiSensingCircuit::measureImpedance(int8_t fromEl, int8_t toEl, directPrintMode measureImpedance_directPrint_MODE){
	// Check input:
	//if(fromEl == toEl  ||  ((fromEl < 0 || fromEl >= NELECTRODES) && fromEl != NEEDLE)  ||  ((toEl < 0 || toEl >= NELECTRODES) && toEl != NEEDLE) ){
	if(((fromEl < 0 || fromEl >= NELECTRODES) && fromEl != NEEDLE && fromEl != DISABLE_MUX)  ||  ((toEl < 0 || toEl >= NELECTRODES) && toEl != NEEDLE && toEl != DISABLE_MUX) ){
		// DISABLE ALL in case of invalid input arguments:
		writeModesAndLeds(HIGH<<PIN_LED_RED);
		// Set MUX:
		trinityMUX->prepareSignalMux(DISABLE_MUX);
		trinityMUX->prepareMeasureMux(DISABLE_MUX);
		trinityMUX->prepareGroundMux(DISABLE_MUX);
		trinityMUX->writePreparedChannelsToMux();
		// Delay to wait twice the time while false electrodes may be active (caused by finite I2C writing speed combined with sequential writing of two separate registers)
		delayMicroseconds(2*I2C_8BIT_DELAY_US);
		return -1;
	}
	// Start Measurement:
	beginMeasure();
	  // Needle to Electrode:
	if(fromEl == NEEDLE || toEl == NEEDLE){
		// Set Mode:
		int8_t toEl_ = (fromEl == NEEDLE ? toEl : fromEl);
		#ifdef DEBUG_MODE
			Serial.print("toEl_= ");
			Serial.println(toEl_);
		#endif
		writeModesAndLeds(HIGH<<PIN_LED_GREEN | HIGH<<PIN_MODE_NEEDLE_OUT | HIGH<<PIN_MODE_Z_MEAS_IN);
		// Set MUX:
		trinityMUX->prepareSignalMux(DISABLE_MUX);
		trinityMUX->prepareMeasureMux(DISABLE_MUX);
		trinityMUX->prepareGroundMux(toEl_);
	}
	  // Electrode to Electrode:
	else{
		// Set Mode:
		writeModesAndLeds(HIGH<<PIN_LED_GREEN | HIGH<<PIN_MODE_Z_MEAS_IN);
		// Set MUX:
		trinityMUX->prepareSignalMux(fromEl);
		trinityMUX->prepareMeasureMux(DISABLE_MUX);
		trinityMUX->prepareGroundMux(toEl);
	}
	trinityMUX->writePreparedChannelsToMux();
	// Delay to wait twice the time while false electrodes may be active (caused by finite I2C writing speed combined with sequential writing of two separate registers)
	delayMicroseconds(2*I2C_8BIT_DELAY_US);
	// Measure:
	ad5933ciruit->getOneFrequency(nullptr, nullptr);
	#ifdef DEBUG_MODE
		Serial.print("resistance_re = ");
		Serial.println(ad5933ciruit->tempRealData);
		Serial.print("reactance_im = ");
		Serial.println(ad5933ciruit->tempImagData);
	#endif
	// Calculate Magnitude of corrected DFT-Values:
	float rawMag = (float) sqrt(pow(ad5933ciruit->tempRealData - compensation_Z_Re, 2) + pow(ad5933ciruit->tempImagData - compensation_Z_Im, 2));
	// Calculate Impedance:
	float impedanceMagnitude = 1/(gain_Z*rawMag);
	// Since the Phase of the Impedance (Phi(Z)) is the same as the Phase between Real and Imaginary Part of the DFT (Re, Im) (corresponding to Data sheet of AD5933, page 19),
	// Phase(rads) = arctan(Im/Re)
	// Thereby, "Resistance" and "Reactance" can be calculated out of Re and Im
	// Since "Z_complex = |Z| * (Re - j*Im)/sqrt(Re^2 + Im^2)",  calculate zDivRawMagFactor = impedanceMagnitude/rawMag:
	float negZdivRawMagFactor = -impedanceMagnitude/rawMag;	// negative Sign '-' since additional inverting amplifier
	// Calculate "Resistance":
	result_Real = negZdivRawMagFactor*(ad5933ciruit->tempRealData - compensation_Z_Re); // Scale real DFT-value (Re) to the size of the Magnitude of Impedance
	// Calculate Reactance:
	result_Imag = negZdivRawMagFactor*(ad5933ciruit->tempImagData - compensation_Z_Im); // Scale imaginary DFT-value (Re) to the size of the Magnitude of Impedance
	#ifdef DEBUG_MODE
		Serial.print("result_Real = ");
		Serial.println(result_Real);
		Serial.print("result_Imag = ");
		Serial.println(result_Imag);
	#endif
	// Direct Send if desired:
	if(measureImpedance_directPrint_MODE == DirectPRINT_COMPLEX){
		sendBinaryData(MODE_Complex_Impedance, analogSwitch_SIG_OUT_NEEDLE->getState() ? NEEDLE : trinityMUX->getSignalMuxChannel(), trinityMUX->getGroundMuxChannel(), DISABLE_MUX, result_Real, result_Imag);
		}else if (measureImpedance_directPrint_MODE == DirectPRINT_MAGNITUDE){
		sendBinaryData(MODE_Magnitude_of_Impedance, analogSwitch_SIG_OUT_NEEDLE->getState() ? NEEDLE : trinityMUX->getSignalMuxChannel(), trinityMUX->getGroundMuxChannel(), DISABLE_MUX, impedanceMagnitude);
	}
	// Convert DFT-Values into impedance:
	return impedanceMagnitude;  // Mind that SHUNT_RESIST is included in this result!
}


// Measure Reactance:
float ImpedanceMultiSensingCircuit::measureReactance(int8_t fromEl, int8_t toEl, directPrintMode measureReactance_directPrint_MODE){
	measureImpedance(fromEl, toEl, DirectPRINT_NONE);
	if(measureReactance_directPrint_MODE == DirectPRINT_MAGNITUDE || measureReactance_directPrint_MODE == DirectPRINT_COMPLEX){
		sendBinaryData(MODE_Magnitude_of_Impedance, analogSwitch_SIG_OUT_NEEDLE->getState() ? NEEDLE : trinityMUX->getSignalMuxChannel(), trinityMUX->getGroundMuxChannel(), DISABLE_MUX, result_Imag);
	}
	return result_Imag;
}
// Measure Resistance:
float ImpedanceMultiSensingCircuit::measureResistance(int8_t fromEl, int8_t toEl, directPrintMode measureResistance_directPrint_MODE){
	measureImpedance(fromEl, toEl, DirectPRINT_NONE);
	if(measureResistance_directPrint_MODE == DirectPRINT_MAGNITUDE || measureResistance_directPrint_MODE == DirectPRINT_COMPLEX){
		sendBinaryData(MODE_Magnitude_of_Impedance, analogSwitch_SIG_OUT_NEEDLE->getState() ? NEEDLE : trinityMUX->getSignalMuxChannel(), trinityMUX->getGroundMuxChannel(), DISABLE_MUX, result_Real);
	}
	return result_Real;
}

// Measure Voltage:
float ImpedanceMultiSensingCircuit::measureVoltage(int8_t measuringEl, int8_t stimSig, int8_t stimGnd, directPrintMode measureVoltage_directPrint_MODE, bool correctLeakage){
	// Correct Leakage:
	// !!EXPERIMENTAL!!
	float leakageCurrentCompensationFactorA;
	if (correctLeakage){
		// leakageCurrentCompensationFactorA = maxRefV@SignalPin / LeakedVoltage_withoutGNDpin:
		// Faster Variant with only one additional Voltage measurement:
			//leakageCurrentCompensationFactorA = (maxMeasuredDFT * gain_U) / measureVoltage(measuringEl, stimSig, DISABLE_MUX, DirectPRINT_NONE, false);
		leakageCurrentCompensationFactorA = measureVoltage(measuringEl, measuringEl, DISABLE_MUX, DirectPRINT_NONE, false) / measureVoltage(measuringEl, stimSig, DISABLE_MUX, DirectPRINT_NONE, false);
		// Even better, but a third Voltage MEasurement is required... :
		//float leakageCurrentCompensationFactorA = measureVoltage(stimSig, stimSig, DISABLE_MUX, false) / measureVoltage(measuringEl, stimSig, DISABLE_MUX, false); // Even better, but a third Voltage MEasurement is required...
	}
	// Set Analog Switch:
	uint8_t mode4bit;
	uint8_t led4bit = HIGH<<PIN_LED_BLUE;	// Activate blue LED;
	int8_t stimGnd_ = stimGnd;
	// Check input:
	//if(stimSig == stimGnd  ||  ((stimSig < 0 || stimSig >= NELECTRODES) && stimSig != NEEDLE)  ||  ((stimGnd < 0 || stimGnd >= NELECTRODES) && stimGnd != NEEDLE) ){
	if(((stimSig < 0 || stimSig >= NELECTRODES) && stimSig != NEEDLE && stimSig != DISABLE_MUX) || ((stimGnd < 0 || stimGnd >= NELECTRODES) && stimGnd != NEEDLE && stimGnd != DISABLE_MUX) || ((measuringEl < 0 || measuringEl >= NELECTRODES) && measuringEl != NEEDLE && measuringEl != DISABLE_MUX)){
		// DISABLE ALL in case of invalid input arguments:
		writeModesAndLeds(HIGH<<PIN_LED_RED);
		// Set MUX:
		trinityMUX->prepareSignalMux(DISABLE_MUX);
		trinityMUX->prepareMeasureMux(DISABLE_MUX);
		trinityMUX->prepareGroundMux(DISABLE_MUX);
		trinityMUX->writePreparedChannelsToMux();
		// Delay to wait twice the time while false electrodes may be active (caused by finite I2C writing speed combined with sequential writing of two separate registers)
		delayMicroseconds(2*I2C_8BIT_DELAY_US);
		return -1;
	}
	// Start Measurement:
	beginMeasure();
	if(stimSig == NEEDLE || stimGnd == NEEDLE){
		stimGnd_ = (stimSig == NEEDLE ? stimGnd : stimSig);
		if(measuringEl == NEEDLE){ // this case is not relevant for usual measuring
			writeModesAndLeds(HIGH<<PIN_LED_BLUE | HIGH<<PIN_MODE_NEEDLE_OUT | HIGH<<PIN_MODE_NEEDLE_IN);
			// Set MUX:
			trinityMUX->prepareSignalMux(DISABLE_MUX);
			trinityMUX->prepareMeasureMux(DISABLE_MUX);
			trinityMUX->prepareGroundMux(stimGnd_);
		}
		else{
			writeModesAndLeds(HIGH<<PIN_LED_BLUE | HIGH<<PIN_MODE_NEEDLE_OUT | HIGH<<PIN_MODE_V_MEAS_IN);
			// Set MUX:
			trinityMUX->prepareSignalMux(DISABLE_MUX);
			trinityMUX->prepareMeasureMux(measuringEl);
			trinityMUX->prepareGroundMux(stimGnd_);
		}
	}
	else{
		if(measuringEl == NEEDLE){ // this case is not relevant for usual measuring
			writeModesAndLeds(HIGH<<PIN_LED_BLUE | HIGH<<PIN_MODE_NEEDLE_IN);
			// Set MUX:
			trinityMUX->prepareSignalMux(stimSig);
			trinityMUX->prepareMeasureMux(DISABLE_MUX);
			trinityMUX->prepareGroundMux(stimGnd);
		}
		else{
			writeModesAndLeds(HIGH<<PIN_LED_BLUE | HIGH<<PIN_MODE_V_MEAS_IN);
			// Set MUX:
			trinityMUX->prepareSignalMux(stimSig);
			trinityMUX->prepareMeasureMux(measuringEl);
			trinityMUX->prepareGroundMux(stimGnd);
		}
}
	trinityMUX->writePreparedChannelsToMux();
	// Delay to wait twice the time while false electrodes may be active (caused by finite I2C writing speed combined with sequential writing of two separate registers)
	delayMicroseconds(2*I2C_8BIT_DELAY_US);
	// Measure:
	ad5933ciruit->getOneFrequency(nullptr, nullptr);
	result_Real = gain_U*(ad5933ciruit->tempRealData - compensation_V_Re);
	result_Imag = gain_U*(ad5933ciruit->tempImagData - compensation_V_Im);
	#ifdef DEBUG_MODE
		Serial.print("result_Real = ");
		Serial.println(result_Real);
		Serial.print("result_Imag = ");
		Serial.println(result_Imag);
	#endif
	// Correct Leakage:
	if (correctLeakage){
		result_Real *= leakageCurrentCompensationFactorA;
		result_Imag *= leakageCurrentCompensationFactorA;
	}
	// Magnitude calculation:
	float magnitude = sqrt(pow(result_Real, 2) + pow(result_Imag, 2));
	if(measureVoltage_directPrint_MODE == DirectPRINT_COMPLEX){
		sendBinaryData(MODE_Complex_Voltage, analogSwitch_SIG_OUT_NEEDLE->getState() ? NEEDLE : trinityMUX->getSignalMuxChannel(), trinityMUX->getGroundMuxChannel(), measuringEl, result_Real, result_Imag);
	}else if (measureVoltage_directPrint_MODE == DirectPRINT_MAGNITUDE){
		sendBinaryData(MODE_Magnitude_of_Voltage, analogSwitch_SIG_OUT_NEEDLE->getState() ? NEEDLE : trinityMUX->getSignalMuxChannel(), trinityMUX->getGroundMuxChannel(), measuringEl, magnitude);
	}
	// Return L2 of Voltages:
	return magnitude;
}

// Measure Nothing:
void ImpedanceMultiSensingCircuit::disableModes(){
	setElectrodeInitCond();
	// write the new GPIOA
	writeModesAndLeds(HIGH<<PIN_LED_YELLOW);
}

// Refresh max. measurable DFT-Value int 'maxMeasuredDFT':
void ImpedanceMultiSensingCircuit::getMaxMeasuredDFT(){
	// Set correct Mode:
	writeModesAndLeds(HIGH<<PIN_LED_YELLOW | HIGH<<PIN_LED_GREEN | HIGH<<PIN_MODE_V_MEAS_IN);
	// Begin Measurement to get 'maxMeasuredDFT';
	beginMeasure();
	// Prepare Measure-Electrode to be set:
	trinityMUX->prepareMeasureMux(0);
	// Prepare Signal-Electrode to be set:
	trinityMUX->prepareSignalMux(0);
	// Prepare GND-Electrode to not be set:
	trinityMUX->prepareGroundMux(DISABLE_MUX);
	// Set MUXs by writing Registers of GPIO-Expander:
	trinityMUX->writePreparedChannelsToMux();
	// Delay to wait twice the time while false electrodes may be active (caused by finite I2C writing speed combined with sequential writing of two separate registers)
	delayMicroseconds(2*I2C_8BIT_DELAY_US);
	// Measurement it selves:
	ad5933ciruit->getOneFrequency(nullptr, nullptr);
	maxMeasuredDFT = sqrt(pow(ad5933ciruit->tempRealData, 2) + pow(ad5933ciruit->tempImagData, 2));
	// End Measurement (goto Standby)
	endMeasure();
	// Initialize Pin Electrode configuration by disabling all:
	setElectrodeInitCond();
}

// Measure Shunt DFT-value:
void ImpedanceMultiSensingCircuit::determineGainZ(){
	// !Reset! GAIN for Impedance:
	gain_Z = 1;
	// Determine GAIN for Impedance (with reseted GAIN = 1):
	beginMeasure();
	delay(10);
	gain_Z = measureImpedance(0, 0)/SHUNT_RESIST;
	endMeasure();
	#ifdef DEBUG_MODE
		Serial.print("gain_Z = ");
		Serial.print(gain_Z, 12);
	#endif
}

// Measure Compensation Values !IF! nothing is connected:
void ImpedanceMultiSensingCircuit::determineCompensationValues(){
	// !!!!! ----- TODO ----- !!!!!
	// !!!!! ----- TODO ----- !!!!!
	// Kompensationsmessungen für jeden einzelnen Fall erstellen (das zu und weg schalten elektronischer komponenten führt immer zu änderungen!!)
	// Auch für nadelmessungen extra kompensationswerte vorsehen
	// Zusätzliche kompensationsmessungen sind auch notwendig, wenn der Analogschalter in eine andere schaltstellung schaltet, da dann die Impedanzwandler und mux abgekoppelt werden.
	// !!!!! ----- TODO ----- !!!!!
	// !!!!! ----- TODO ----- !!!!!
	// Begin Measuring:
	beginMeasure();
	// Check if something is connected:
	// Measured Impedance between 0 and 1:
	measureImpedance(0, 1);
	// Check if nothing is connected (only Valid for exactly THIS used PCB):
	if(ad5933ciruit->tempRealData > HIGH_Z_REAL_neg && ad5933ciruit->tempImagData < HIGH_Z_IMAG_pos){
		// Nothing connected!
		// Voltage correction:
		setElectrodeInitCond();
		disableModes();
		// Safety loop to ensure valid results:
		uint8_t similarResults = 0;
		while(true){
			// Wait and re-measure:
			delay(10);
			//ad5933ciruit->getOneFrequency(nullptr, nullptr);
			measureVoltage(1,0,1, DirectPRINT_NONE, false); // Sould deliver 0 V
			// Check if similar to last measurement:
			if(abs(compensation_V_Re - ad5933ciruit->tempRealData) + abs(compensation_V_Im - ad5933ciruit->tempImagData) < 2*COMPENSATION_DETERMINATION_SAFETY_REPETITIONS){
				similarResults++;
			}else{
				similarResults = 0;
			}
			// Update Values:
			compensation_V_Re = ad5933ciruit->tempRealData;
			compensation_V_Im = ad5933ciruit->tempImagData;
			// Enough similar results - accept as valid:
			if(similarResults >= COMPENSATION_DETERMINATION_SAFETY_REPETITIONS){
				break;
			}
		}
		// Print:
		#ifdef DEBUG_MODE
			Serial.print("compensation_V_Re = ");
			Serial.println(compensation_V_Re);
			Serial.print("compensation_V_Im = ");
			Serial.println(compensation_V_Im);
		#endif
		// Impedance Correction:
		// Measured Impedance between 0 and 1:
		// Safety loop to ensure valid results:
		similarResults = 0;
		while(true){
			// Wait and re-measure:
			delay(10);
			measureImpedance(0, 1);
			// Check if similar to last measurement:
			if(abs(compensation_Z_Re - ad5933ciruit->tempRealData) + abs(compensation_Z_Im - ad5933ciruit->tempImagData) < 2*COMPENSATION_DETERMINATION_SAFETY_REPETITIONS){
				similarResults++;
			}else{
				similarResults = 0;
			}
			// Update Values:
			// Use raw Data from AD5933:
			compensation_Z_Re = ad5933ciruit->tempRealData;
			compensation_Z_Im = ad5933ciruit->tempImagData;
			// Enough similar results - accept as valid:
			if(similarResults >= COMPENSATION_DETERMINATION_SAFETY_REPETITIONS){
				break;
			}
		}
		// Print:
		#ifdef DEBUG_MODE
			Serial.print("compensation_Z_Re = ");
			Serial.println(compensation_Z_Re);
			Serial.print("compensation_Z_Im = ");
			Serial.println(compensation_Z_Im);
		#endif
	}else{
		// Default Values:
		compensation_V_Re = 0;
		compensation_V_Im = 0;
		compensation_Z_Re = 0;
		compensation_Z_Im = 0;
	}
	// End Measuring:
	endMeasure();
}

// Set Module in Measurement-Mode:
void ImpedanceMultiSensingCircuit::beginMeasure(){
	if (sigGenEnabled) { return;}	// Already begun
	ad5933ciruit->beginOneFrequency();
	delay(HP_SETTLING_TIME); // Wait ? ms until Signal is settled to steady state (HP)
	sigGenEnabled = true;
}

// Measure all electrodes (Stimulation or similar must be prepared before):
void ImpedanceMultiSensingCircuit::measureAllPrepreparedV(directPrintMode measureAllVolt_directPrint_MODE){
	// Start Measurement:
	beginMeasure();
	float magnitude = 0;	// Absolute value of DFT via measured data
	#ifdef DEBUG_MODE
		unsigned long timestamp = micros();
		unsigned long timeConsumption = 0;
	#endif
	// Measure all electrodes:
	byte optionalNeedleMode = (analogSwitch_SIG_OUT_NEEDLE->getState() ? HIGH<<PIN_MODE_NEEDLE_OUT : LOW);
	writeModesAndLeds( HIGH<<PIN_LED_BLUE | HIGH<<PIN_MODE_V_MEAS_IN | optionalNeedleMode);
	for(int i = 0; i<ELECTRODE_NUMBER+NEEDLE_NUMBER; i++){
		if(i == ELECTRODE_NUMBER){	// Switch to needle-measuring
			writeModesAndLeds( HIGH<<PIN_LED_BLUE | HIGH<<PIN_MODE_NEEDLE_IN | optionalNeedleMode);
		}
		// Prepare Measure-Electrode to be set:
		trinityMUX->prepareMeasureMux(i); // @NEEDLE == 16, MUX gets disabled anyway...
		// Set MUXs by writing Registers of GPIO-Expander:
		trinityMUX->writePreparedChannelsToMux();
		// Delay to wait twice the time while false electrodes may be active (caused by finite I2C writing speed combined with sequential writing of two separate registers)
		delayMicroseconds(2*I2C_8BIT_DELAY_US);
		// Measurement it selves:
		ad5933ciruit->getOneFrequency(nullptr, nullptr);
		result_Real = gain_U*(ad5933ciruit->tempRealData - compensation_V_Re);
		result_Imag = gain_U*(ad5933ciruit->tempImagData - compensation_V_Im);
		magnitude = sqrt(pow(result_Real, 2) + pow(result_Imag, 2));
		// Save Results:
		tempRawData[i] = magnitude;
		#ifdef DEBUG_MODE // Debug Mode:
			// Print:
			timeConsumption = micros() - timestamp;
			Serial.print("result_Real = ");Serial.print(result_Real, 4);
			Serial.print(",  result_Imag = ");Serial.print(result_Imag, 4);
			Serial.print(",  magnitude = ");Serial.print(magnitude, 4);
			Serial.print(",  time-consumption = ");Serial.println(timeConsumption); 
			timestamp = micros();
		#endif
		// Send binary Data if requested:
		if(measureAllVolt_directPrint_MODE == DirectPRINT_COMPLEX){
			//sendBinaryData(MODE_Complex_Voltage, stim_pattern&0x0F, stim_pattern>>4, i, result_Real, result_Imag);
			sendBinaryData(MODE_Complex_Voltage, analogSwitch_SIG_OUT_NEEDLE->getState() ? NEEDLE : trinityMUX->getSignalMuxChannel(), trinityMUX->getGroundMuxChannel(), i, result_Real, result_Imag);
		}else if (measureAllVolt_directPrint_MODE == DirectPRINT_MAGNITUDE){
			sendBinaryData(MODE_Magnitude_of_Voltage, analogSwitch_SIG_OUT_NEEDLE->getState() ? NEEDLE : trinityMUX->getSignalMuxChannel(), trinityMUX->getGroundMuxChannel(), i, magnitude);		
		}
	}
}


// Set Module in Standby-Mode (all Disabled):
void ImpedanceMultiSensingCircuit::endMeasure(){
	if (!sigGenEnabled) { return;}	// Already begun
	ad5933ciruit->endOneFrequency();
	setElectrodeInitCond();
	writeModesAndLeds(HIGH<<PIN_LED_YELLOW);
	sigGenEnabled = false;
}


// Prepare stimulation-electrodes:
void ImpedanceMultiSensingCircuit::prepareStimulation(){
	trinityMUX->prepareSignalMux(stim_pattern & 0x0F);
	trinityMUX->prepareGroundMux(stim_pattern >> 4);
}

// Acquire data via Stimulation pattern and Measurement pattern:
void ImpedanceMultiSensingCircuit::acquireEITdata(directPrintMode aquireData_directPrint_MODE, bool compensateLeakage, bool measureStimCurrent){
	// Start Measurement:
	beginMeasure();
	// Set Analog Switch:
	writeModesAndLeds(HIGH<<PIN_LED_BLUE | HIGH<<PIN_MODE_V_MEAS_IN);
	// Leakage detention:
	if(compensateLeakage){ getLeakageImpactVs(); }
	// Prepare Electrodes for correct stimulation:
	prepareStimulation();
	// Measure all Voltages:
	measureAllPrepreparedV();
	// Leakage correction:
	if(compensateLeakage){ correctLeakingDataVs(); }
	if(measureStimCurrent){ // Measure Stimulation Current via Stimulation Resistance
		float stimulationResistance = measureImpedance(stim_pattern & 0x0F, stim_pattern>>4);
		lastMeasuredStimCurrent = (SIGNAL_VPP/2)/(stimulationResistance-SHUNT_RESIST);
	}
	else{ // Calculate Stimulation Current via Shunt-Resistor:
		// Voltage at Shunt-Resistor:
		float uShunt = tempRawData[stim_pattern>>4];     // Voltage across Shunt-Resistance#ifdef DEBUG_MODE
		#ifdef DEBUG_MODE
			Serial.print("uShunt = ");Serial.println(uShunt);
		#endif
		// Consequent Stimulation-Current [mA]:
		lastMeasuredStimCurrent = (uShunt/maxMeasuredDFT*(SIGNAL_VPP/2)/SHUNT_RESIST) * 1000;
	}
	// Convert tempRawData into result vector:
	#ifdef DEBUG_MODE
		Serial.print("maxMeasuredDFT = ");Serial.println(maxMeasuredDFT);
		Serial.print("gain_U = ");Serial.println(gain_U, 10);
		Serial.print("iStimWanted = ");Serial.println(stim_current);
		Serial.print("lastMeasuredStimCurrent = ");Serial.println(lastMeasuredStimCurrent, 10);
		Serial.print("( gain_U ) * iStimWanted / lastMeasuredStimCurrent = ");Serial.println(( gain_U ) * ((float)stim_current) / lastMeasuredStimCurrent,6);
	#endif
	for (int i = 0; i<MEASURE_NUMBER_PER_STIMULATION; i++){
		resultVsEIT[i] = (tempRawData[meas_pattern[i] & 0x0F] - tempRawData[meas_pattern[i]>>4]) * gain_U  * ((float)stim_current) / lastMeasuredStimCurrent;
		if(aquireData_directPrint_MODE == DirectPRINT_MAGNITUDE){
			sendBinaryData(MODE_Magnitude_EIT_Data, trinityMUX->getSignalMuxChannel(), trinityMUX->getGroundMuxChannel(), meas_pattern[i], resultVsEIT[i]);
		}
		//Serial.print("resultVsEIT["); Serial.print(i); Serial.print("] = (tempRawData["); Serial.print(meas_pattern[i] & 0x0F); Serial.print("] - tempRawData[");Serial.print(meas_pattern[i]>>4); Serial.print("]) * "); Serial.println(gain_U  * ((float)stim_current) / lastMeasuredStimCurrent);
		//Serial.print("resultVsEIT["); Serial.print(i); Serial.print("] = ("); Serial.print(tempRawData[meas_pattern[i] & 0x0F]); Serial.print(" - ");Serial.print(tempRawData[meas_pattern[i]>>4]); Serial.print(") * "); Serial.println(gain_U  * ((float)stim_current) / lastMeasuredStimCurrent);
	}
	writeModesAndLeds(HIGH<<PIN_LED_YELLOW);
}


// Refresh leakageCurrentCompensationFactorA [Array] to compensate leakage currents / their impact:
void ImpedanceMultiSensingCircuit::getLeakageImpactVs(){
	// Set Analog Switch:
	byte optionalNeedleMode = (analogSwitch_SIG_OUT_NEEDLE->getState() ? HIGH<<PIN_MODE_NEEDLE_OUT : LOW);
	writeModesAndLeds( HIGH<<PIN_LED_BLUE | HIGH<<PIN_MODE_V_MEAS_IN | optionalNeedleMode);
	// Create single high pin stimulation for leakage detection:
	// Prepare Electrodes for correct stimulation:
	trinityMUX->prepareSignalMux((optionalNeedleMode == LOW) ? (stim_pattern & 0x0F) : DISABLE_MUX);
	trinityMUX->prepareGroundMux(DISABLE_MUX);
	// Measure all Voltages :
	measureAllPrepreparedV();
	// Calculate compensation Array to eliminate impact of leakages:
	float maxRefV = tempRawData[stim_pattern & 0x0F];
	for (int i = 0; i < ELECTRODE_NUMBER+NEEDLE_NUMBER; i++) {
		leakageCurrentCompensationFactorA[i] = maxRefV / tempRawData[i];
	}
}

// Leakage correction:
void ImpedanceMultiSensingCircuit::correctLeakingDataVs(){
	// Correct data:
	for (int i = 0; i < ELECTRODE_NUMBER; i++) {
		tempRawData[i] *= leakageCurrentCompensationFactorA[i];
	}
}



// Get last measured Stimulation current:
float ImpedanceMultiSensingCircuit::getLastMeasuredStimCurrent(){
	return lastMeasuredStimCurrent;
}

// Get gain_U:
float ImpedanceMultiSensingCircuit::getGainU(){
	return gain_U;
}

// Get gain_Z
float ImpedanceMultiSensingCircuit::getGainZ(){
	return gain_Z;
}