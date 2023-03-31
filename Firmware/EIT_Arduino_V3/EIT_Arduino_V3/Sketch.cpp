/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */


// Thomas

#include "HardwareLib/ImpedanceMultiSensingCircuit.h"
#include <EEPROM.h>
//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio

// Function Prototypes:
void writeMeasPatternToEEPROM(int patternIdx);
void writeStimPatternToEEPROM(int patternIdx);
void writeCompensationToEEPROM();
void readMeasPatternFromEEPROM(int patternIdx);
void readStimPatternFromEEPROM(int patternIdx);
void readCompensationFromEEPROM();
void setMeasurementPattern(int patternIdx);
void setStimulationPattern(int patternIdx);
void printMeasPattern();
void printStimPattern();
uint8_t gpioRead(uint8_t port);
void gpioWrite(uint8_t port, uint8_t value);
void gpioMode(uint8_t port, uint8_t value);
void sendCalibData();
void updateCompensationValues();
void startCollectingData(bool, bool);
void startCollectingData2(bool);
void testFunction();



// EEPROM addresses:
#define MEASPATTERNADDR (0)
#define STIMPATTERNADDR (210)
#define COMPENSATIONVALUESADDR (230)
#define BYTESIZE_OF_COMPENSATIONVALUES (2)

// GPIO-Port-1:
#define GPIO_PORT_1_PIN_0   (4)
#define GPIO_PORT_1_PIN_1   (5)
#define GPIO_PORT_1_PIN_2   (6)
#define GPIO_PORT_1_PIN_3   (7)
#define GPIO_PORT_1_PIN_4   (8)
#define GPIO_PORT_1_PIN_5   (9)
// GPIO-Port-2:
#define GPIO_PORT_2_PIN_0   (18)
#define GPIO_PORT_2_PIN_1   (15)
#define GPIO_PORT_2_PIN_2   (14)
#define GPIO_PORT_2_PIN_3   (16)
#define GPIO_PORT_2_PIN_4   (10)

// GPIO PORTS:
#define GPIO_PORT_1 (1)
#define GPIO_PORT_2 (2)
#define GPIO_PORT_3 (3)

// GPIO Read/Write:
#define GPIO_READ_MODE  'r'
#define GPIO_WRITE_MODE 'w'
#define GPIO_SET_DIR    'd'

// Communication speed:
#define SERIAL_BAUDRATE (115200)
// Waiting time to get into steady state of LowPass(3rd degree) [only on analog circuit]:
#define ACQUISITION_TIME_ANALOG_US (250)
/// EIT-Constances:
#define STIMULATION_CURRENT             (1)     // [mA]
#define DIGITS_AFTER_POINT              (5)

// Incoming Byte
char incomingByte = ' ';
// Leackage Correction for Voltage Measurement - Flag
bool voltageMeasureLeakageCorrection = false;

// =======================================================================

// MUX16_CIRCUIT:
ImpedanceMultiSensingCircuit* eitModule;	// Create new ImpedanceMultiSensingCircuit object

// =======================================================================



// Write meas_pattern to EEPROM (binary):
void writeMeasPatternToEEPROM(int patternIdx){
	uint8_t dataByte = 0;
	for(int i = 0; i<MEASURE_NUMBER_PER_STIMULATION; i++){
		EEPROM.update(MEASPATTERNADDR + patternIdx*MEASURE_NUMBER_PER_STIMULATION + i, eitModule->meas_pattern[i]);
	}
}

// Write stim_pattern to EEPROM (binary):
void writeStimPatternToEEPROM(int patternIdx){
	EEPROM.update(STIMPATTERNADDR + patternIdx, eitModule->stim_pattern);
	EEPROM.update(STIMPATTERNADDR + ELECTRODE_NUMBER, eitModule->stim_current);
}

// Write compensation values to EEPROM:
void writeCompensationToEEPROM(){
	EEPROM.update(COMPENSATIONVALUESADDR                                   , ((uint16_t)eitModule->compensation_V_Re)>>8);  EEPROM.update(COMPENSATIONVALUESADDR                                 +1, ((uint16_t)eitModule->compensation_V_Re)&0xFF);
	EEPROM.update(COMPENSATIONVALUESADDR+1*BYTESIZE_OF_COMPENSATIONVALUES  , ((uint16_t)eitModule->compensation_V_Im)>>8);  EEPROM.update(COMPENSATIONVALUESADDR+1*BYTESIZE_OF_COMPENSATIONVALUES+1, ((uint16_t)eitModule->compensation_V_Im)&0xFF);
	EEPROM.update(COMPENSATIONVALUESADDR+2*BYTESIZE_OF_COMPENSATIONVALUES  , ((uint16_t)eitModule->compensation_Z_Re)>>8);  EEPROM.update(COMPENSATIONVALUESADDR+2*BYTESIZE_OF_COMPENSATIONVALUES+1, ((uint16_t)eitModule->compensation_Z_Re)&0xFF);
	EEPROM.update(COMPENSATIONVALUESADDR+3*BYTESIZE_OF_COMPENSATIONVALUES  , ((uint16_t)eitModule->compensation_Z_Im)>>8);  EEPROM.update(COMPENSATIONVALUESADDR+3*BYTESIZE_OF_COMPENSATIONVALUES+1, ((uint16_t)eitModule->compensation_Z_Im)&0xFF);
	//EEPROM.update(COMPENSATIONVALUESADDR  , (uint8_t)((byte)eitModule->compensation_V_Re));//(eitModule->compensation_V_Re < 0 ? ((uint8_t)eitModule->compensation_V_Re | 1<7) : (uint8_t)eitModule->compensation_V_Re));
	//EEPROM.update(COMPENSATIONVALUESADDR+1, (uint8_t)((byte)eitModule->compensation_V_Im));//(eitModule->compensation_V_Im < 0 ? ((uint8_t)eitModule->compensation_V_Im | 1<7) : (uint8_t)eitModule->compensation_V_Im));
	//EEPROM.update(COMPENSATIONVALUESADDR+2, (uint8_t)((byte)eitModule->compensation_Z_Re));//(eitModule->compensation_Z_Re < 0 ? ((uint8_t)eitModule->compensation_Z_Re | 1<7) : (uint8_t)eitModule->compensation_Z_Re));
	//EEPROM.update(COMPENSATIONVALUESADDR+3, (uint8_t)((byte)eitModule->compensation_Z_Im));//(eitModule->compensation_Z_Im < 0 ? ((uint8_t)eitModule->compensation_Z_Im | 1<7) : (uint8_t)eitModule->compensation_Z_Im));
}

// Read meas_pattern from EEPROM (binary):
void readMeasPatternFromEEPROM(int patternIdx){
	// Get Data:
	for(int i = 0; i<MEASURE_NUMBER_PER_STIMULATION; i++){
		eitModule->meas_pattern[i] = EEPROM.read(MEASPATTERNADDR + patternIdx*MEASURE_NUMBER_PER_STIMULATION + i);
	}
}

// Read stim_pattern from EEPROM (binary):
void readStimPatternFromEEPROM(int patternIdx){
	// Get Data:
	eitModule->stim_pattern = EEPROM.read(STIMPATTERNADDR + patternIdx);
	eitModule->stim_current = EEPROM.read(STIMPATTERNADDR + ELECTRODE_NUMBER);
}

// Read compensation values from EEPROM:
void readCompensationFromEEPROM(){
	eitModule->compensation_V_Re = (EEPROM.read(COMPENSATIONVALUESADDR                                   )<<8) +  EEPROM.read(COMPENSATIONVALUESADDR                                 +1);
	eitModule->compensation_V_Im = (EEPROM.read(COMPENSATIONVALUESADDR+1*BYTESIZE_OF_COMPENSATIONVALUES  )<<8) +  EEPROM.read(COMPENSATIONVALUESADDR+1*BYTESIZE_OF_COMPENSATIONVALUES+1);
	eitModule->compensation_Z_Re = (EEPROM.read(COMPENSATIONVALUESADDR+2*BYTESIZE_OF_COMPENSATIONVALUES  )<<8) +  EEPROM.read(COMPENSATIONVALUESADDR+2*BYTESIZE_OF_COMPENSATIONVALUES+1);
	eitModule->compensation_Z_Im = (EEPROM.read(COMPENSATIONVALUESADDR+3*BYTESIZE_OF_COMPENSATIONVALUES  )<<8) +  EEPROM.read(COMPENSATIONVALUESADDR+3*BYTESIZE_OF_COMPENSATIONVALUES+1);
}

// Receive Measurement Pattern (binary):
void setMeasurementPattern(int patternIdx){
  for (int i = 0; i<MEASURE_NUMBER_PER_STIMULATION; i++){
	  eitModule->meas_pattern[i] = 0;
	  for (int j = 0; j<ELECTRODE_NUMBER; j++){
		  while (Serial.available() <= 0){}
		  incomingByte = (Serial.read());
		  if((int8_t)incomingByte < 0){
			  eitModule->meas_pattern[i] |= j<<4;
		  }else if((int8_t)incomingByte > 0){
			  eitModule->meas_pattern[i] |= j;
		  }
	  }
  }
  // Write to EEPROM:
  writeMeasPatternToEEPROM(patternIdx);
  //printMeasPattern();  
}

// Receive Stimulation Pattern (binary):
void setStimulationPattern(int patternIdx){
	eitModule->stim_pattern = 0;
	for (int j = 0; j<ELECTRODE_NUMBER; j++){
		while (Serial.available() <= 0){}
		incomingByte = (Serial.read());
		if((int8_t)incomingByte < 0){
			eitModule->stim_pattern |= j<<4;
		}else if((int8_t)incomingByte > 0){
			eitModule->stim_pattern |= j;
			eitModule->stim_current = incomingByte;
		}
	}
	// Write to EEPROM:
	writeStimPatternToEEPROM(patternIdx);
  //printStimPattern();
}

// Print meas_pattern (binary):
void printMeasPattern(){
	uint8_t negPos;
	uint8_t posPos;
	Serial.println(F("["));
	for (int h = 0; h<MEASURE_NUMBER_PER_STIMULATION; h++){
		Serial.print(F("["));
		negPos = eitModule->meas_pattern[h]>>4;
		posPos = eitModule->meas_pattern[h] & 0x0F;
		for (int i = 0; i<ELECTRODE_NUMBER; i++){
			if(i == negPos){Serial.print(F("-1"));}
			else if(i == posPos){Serial.print(F(" 1"));}
			else {Serial.print(F(" 0"));}
			// Comma:
			if(i < ELECTRODE_NUMBER-1){Serial.print(F(", "));}
		}
		Serial.println(F("];"));
	}
	Serial.println(F("]"));
}

// Print stim_pattern (binary):
void printStimPattern(){
	Serial.print(F("["));
	uint8_t negPos = eitModule->stim_pattern>>4;
	uint8_t posPos = eitModule->stim_pattern & 0x0F;
	for (int i = 0; i<ELECTRODE_NUMBER; i++){
		if(i == negPos){Serial.print(F("-"));Serial.print(eitModule->stim_current); }
		else if(i == posPos){Serial.print(F(" "));Serial.print(eitModule->stim_current);}
		else {Serial.print(F(" 0"));}
		// Comma:
		if(i < ELECTRODE_NUMBER-1){Serial.print(F(", "));}
	}
	Serial.println(F("]"));
}

// GPIO-Read:
uint8_t gpioRead(uint8_t port){
	if(port == GPIO_PORT_1){
		return(((uint8_t)digitalRead(GPIO_PORT_1_PIN_5))<<5 |((uint8_t)digitalRead(GPIO_PORT_1_PIN_4))<<4 | ((uint8_t)digitalRead(GPIO_PORT_1_PIN_3))<<3 | ((uint8_t)digitalRead(GPIO_PORT_1_PIN_2))<<2 | ((uint8_t)digitalRead(GPIO_PORT_1_PIN_1))<<1 | ((uint8_t)digitalRead(GPIO_PORT_1_PIN_0)));
		}
	else if(port == GPIO_PORT_2){
		return(((uint8_t)digitalRead(GPIO_PORT_2_PIN_4))<<4 | ((uint8_t)digitalRead(GPIO_PORT_2_PIN_3))<<3 | ((uint8_t)digitalRead(GPIO_PORT_2_PIN_2))<<2 | ((uint8_t)digitalRead(GPIO_PORT_2_PIN_1))<<1 | ((uint8_t)digitalRead(GPIO_PORT_2_PIN_0)));
	}
	else if(port == GPIO_PORT_3){
		return eitModule->mcp23017_ModeExpander->readGPIO(1); // readGPIO of port B
	}
}
// GPIO-Write:
void gpioWrite(uint8_t port, uint8_t value){
	if(port == GPIO_PORT_1){
		digitalWrite(GPIO_PORT_1_PIN_0, (value&(1   ))!=0);
		digitalWrite(GPIO_PORT_1_PIN_1, (value&(1<<1))!=0);
		digitalWrite(GPIO_PORT_1_PIN_2, (value&(1<<2))!=0);
		digitalWrite(GPIO_PORT_1_PIN_3, (value&(1<<3))!=0);
		digitalWrite(GPIO_PORT_1_PIN_4, (value&(1<<4))!=0);
		digitalWrite(GPIO_PORT_1_PIN_5, (value&(1<<5))!=0);
	}
	else if(port == GPIO_PORT_2){
		digitalWrite(GPIO_PORT_2_PIN_0, (value&(1   ))!=0);
		digitalWrite(GPIO_PORT_2_PIN_1, (value&(1<<1))!=0);
		digitalWrite(GPIO_PORT_2_PIN_2, (value&(1<<2))!=0);
		digitalWrite(GPIO_PORT_2_PIN_3, (value&(1<<3))!=0);
		digitalWrite(GPIO_PORT_2_PIN_4, (value&(1<<4))!=0);
	}
	else if(port == GPIO_PORT_3){
		eitModule->mcp23017_ModeExpander->writeRegister(MCP23017_GPIOB, value);	// Write directly into register
	}
}
// GPIO-Write:
void gpioMode(uint8_t port, uint8_t value){
	// 0 for input, 1 for output, single bits are relevant!
	if(port == GPIO_PORT_1){
		pinMode(GPIO_PORT_1_PIN_0, (value&(1   ))!=0);
		pinMode(GPIO_PORT_1_PIN_1, (value&(1<<1))!=0);
		pinMode(GPIO_PORT_1_PIN_2, (value&(1<<2))!=0);
		pinMode(GPIO_PORT_1_PIN_3, (value&(1<<3))!=0);
		pinMode(GPIO_PORT_1_PIN_4, (value&(1<<4))!=0);
		pinMode(GPIO_PORT_1_PIN_5, (value&(1<<5))!=0);
	}
	else if(port == GPIO_PORT_2){
		pinMode(GPIO_PORT_2_PIN_0, (value&(1   ))!=0);
		pinMode(GPIO_PORT_2_PIN_1, (value&(1<<1))!=0);
		pinMode(GPIO_PORT_2_PIN_2, (value&(1<<2))!=0);
		pinMode(GPIO_PORT_2_PIN_3, (value&(1<<3))!=0);
		pinMode(GPIO_PORT_2_PIN_4, (value&(1<<4))!=0);
	}
	else if(port == GPIO_PORT_3){
		eitModule->mcp23017_ModeExpander->writeRegister(MCP23017_IODIRB, ~value);	// Write IODIR register of port B to given value (@ MCP23017, 1 corresponds to input, so the value has to be inverted)
	}
}


// Sequence of sending Calibration Data:
void sendCalibData(){
	// Begin
	eitModule->beginMeasure();
	// Send Gains:
	eitModule->sendBinaryData(MODE_Magnitude_of_other_Data, -1, -1, -1, eitModule->getGainU());
	eitModule->sendBinaryData(MODE_Magnitude_of_other_Data, -1, -1, -1, eitModule->getGainZ());
	// Send Compensation Values:
	eitModule->sendBinaryData(MODE_Complex_other_Data, -1, -1, -1, eitModule->compensation_V_Re, eitModule->compensation_V_Im);
	eitModule->sendBinaryData(MODE_Complex_other_Data, -1, -1, -1, eitModule->compensation_Z_Re, eitModule->compensation_Z_Im);
	// Measure and send Shunt:
	eitModule->measureImpedance(0,0);
	eitModule->sendBinaryData(MODE_Complex_other_Data, 0, 0, -1, eitModule->result_Real, eitModule->result_Imag);
	// End Sequence of sending Calibration Data:
	eitModule->sendBinaryData(MODE_Data_acquisition_finished);
	// End:
	eitModule->endMeasure();
}

// Get compensation Data / Config:
void updateCompensationValues(){
	// Calibrate if possible or load old Calibration Data:
	if(eitModule->compensation_V_Re == 0 && eitModule->compensation_V_Im == 0 && eitModule->compensation_Z_Re == 0 && eitModule->compensation_Z_Im == 0){
		// Calibration failed;
		readCompensationFromEEPROM();
		#ifdef DEBUG_MODE
			Serial.print(F("compensation_V_Re = "));
			Serial.println(eitModule->compensation_V_Re);
			Serial.print(F("compensation_V_Im = "));
			Serial.println(eitModule->compensation_V_Im);
			Serial.print(F("compensation_Z_Re = "));
			Serial.println(eitModule->compensation_Z_Re);
			Serial.print(F("compensation_Z_Im = "));
			Serial.println(eitModule->compensation_Z_Im);
		#endif
		Serial.println(F("Load Config..."));
	} else {
		// Save Calibration to EEPROM;
		// Check if something changed beyond the tolerance range:
		if(	abs(((EEPROM.read(COMPENSATIONVALUESADDR                                   )<<8) +  EEPROM.read(COMPENSATIONVALUESADDR                                 +1))-eitModule->compensation_V_Re) > COMPENSATIONTOLERANCE ||
			abs(((EEPROM.read(COMPENSATIONVALUESADDR+1*BYTESIZE_OF_COMPENSATIONVALUES  )<<8) +  EEPROM.read(COMPENSATIONVALUESADDR+1*BYTESIZE_OF_COMPENSATIONVALUES+1))-eitModule->compensation_V_Im) > COMPENSATIONTOLERANCE ||
			abs(((EEPROM.read(COMPENSATIONVALUESADDR+2*BYTESIZE_OF_COMPENSATIONVALUES  )<<8) +  EEPROM.read(COMPENSATIONVALUESADDR+2*BYTESIZE_OF_COMPENSATIONVALUES+1))-eitModule->compensation_Z_Re) > COMPENSATIONTOLERANCE ||
			abs(((EEPROM.read(COMPENSATIONVALUESADDR+3*BYTESIZE_OF_COMPENSATIONVALUES  )<<8) +  EEPROM.read(COMPENSATIONVALUESADDR+3*BYTESIZE_OF_COMPENSATIONVALUES+1))-eitModule->compensation_Z_Im) > COMPENSATIONTOLERANCE ){
				Serial.println(eitModule->compensation_V_Re);
				Serial.println(eitModule->compensation_V_Im);
				Serial.println(eitModule->compensation_Z_Re);
				Serial.println(eitModule->compensation_Z_Im);
				//Serial.println((EEPROM.read(COMPENSATIONVALUESADDR                                   )<<8) +  EEPROM.read(COMPENSATIONVALUESADDR                                 +1));
				//Serial.println((EEPROM.read(COMPENSATIONVALUESADDR+1*BYTESIZE_OF_COMPENSATIONVALUES  )<<8) +  EEPROM.read(COMPENSATIONVALUESADDR+1*BYTESIZE_OF_COMPENSATIONVALUES+1));
				//Serial.println((EEPROM.read(COMPENSATIONVALUESADDR+2*BYTESIZE_OF_COMPENSATIONVALUES  )<<8) +  EEPROM.read(COMPENSATIONVALUESADDR+2*BYTESIZE_OF_COMPENSATIONVALUES+1));
				//Serial.println((EEPROM.read(COMPENSATIONVALUESADDR+3*BYTESIZE_OF_COMPENSATIONVALUES  )<<8) +  EEPROM.read(COMPENSATIONVALUESADDR+3*BYTESIZE_OF_COMPENSATIONVALUES+1));
			// Update:
			writeCompensationToEEPROM();
			Serial.println(F("Measure and refresh Config..."));
		}else{
			Serial.println(F("Measure Config..."));
		}
	}
	
}


// Transmit measured Data:
void startCollectingData(bool compensateLeakage = true, bool measureStimCurrent = true){
	// Begin Measurement:
	eitModule->beginMeasure();
	for(int h = 0; h < ELECTRODE_NUMBER; h++){
		// Set Patterns:
		readMeasPatternFromEEPROM(h);
		readStimPatternFromEEPROM(h);
		// Measure:
		//eitModule->acquireEITdata(DirectPRINT_NONE, compensateLeakage, measureStimCurrent);
		eitModule->acquireEITdata(DirectPRINT_MAGNITUDE, compensateLeakage, measureStimCurrent);
		// Transmit acquired Data:
		//for (int i = 0; i<MEASURE_NUMBER_PER_STIMULATION; i++){
		//	Serial.println(eitModule->resultVsEIT[i], DIGITS_AFTER_POINT);
		//}
		//Serial.println("Stimulation current = "); Serial.println(eitModule.getLastMeasuredStimCurrent(), DIGITS_AFTER_POINT);
	}
	// End Sequence of sending EIT Data:
	eitModule->sendBinaryData(MODE_Data_acquisition_finished);
	// End Measurement:
	eitModule->endMeasure();
}

void startCollectingData2(bool complex = false){
	// Begin
	eitModule->beginMeasure();
	// Measure all Voltages for each possible stimulation case (including Needle):
	eitModule->stim_current = 10; // mA
	eitModule->writeModesAndLeds(HIGH<<PIN_LED_BLUE);
	for(int i = 0; i < ELECTRODE_NUMBER; i++){
		for(int j = i+1; j < ELECTRODE_NUMBER; j++){
			eitModule->stim_pattern = i<<4 | j; // GND-Electrode << 4, SIGNAL-Electrode
			eitModule->prepareStimulation();
			eitModule->measureAllPrepreparedV(complex ? DirectPRINT_COMPLEX : DirectPRINT_MAGNITUDE);
		}
	}
	// Measure all Voltages from Needle-SIGNAL to every possible Ground:
	eitModule->writeModesAndLeds(HIGH<<PIN_LED_BLUE | HIGH<<PIN_MODE_NEEDLE_OUT);
	for(int i = 0; i < ELECTRODE_NUMBER; i++){
		eitModule->trinityMUX->prepareSignalMux(DISABLE_MUX);
		eitModule->trinityMUX->prepareGroundMux(i);
		eitModule->measureAllPrepreparedV(complex ? DirectPRINT_COMPLEX : DirectPRINT_MAGNITUDE);
	}
	// Measure All voltages without Ground:
	eitModule->writeModesAndLeds(HIGH<<PIN_LED_BLUE);
	for(int i = 0; i < ELECTRODE_NUMBER; i++){
		eitModule->trinityMUX->prepareSignalMux(i);
		eitModule->trinityMUX->prepareGroundMux(DISABLE_MUX);
		eitModule->measureAllPrepreparedV(complex ? DirectPRINT_COMPLEX : DirectPRINT_MAGNITUDE);
	}
	// Measure Voltages from Needle-SIGNAL without Ground:
	eitModule->writeModesAndLeds(HIGH<<PIN_LED_BLUE | HIGH<<PIN_MODE_NEEDLE_OUT);
	eitModule->trinityMUX->prepareSignalMux(DISABLE_MUX);
	eitModule->trinityMUX->prepareGroundMux(DISABLE_MUX);
	eitModule->measureAllPrepreparedV(complex ? DirectPRINT_COMPLEX : DirectPRINT_MAGNITUDE);
	// ---------------------------------------------------------------
	// Measure all resistances to each other (including Needle):
	for(int i = 0; i < ELECTRODE_NUMBER; i++){
		for(int j = i+1; j < ELECTRODE_NUMBER+NEEDLE_NUMBER; j++){
			if(complex){
				eitModule->measureImpedance(i, j);
				eitModule->sendBinaryData(MODE_Complex_Impedance, i, j, -1, eitModule->result_Real, eitModule->result_Imag);
			}else{
				eitModule->sendBinaryData(MODE_Magnitude_of_Impedance, i, j, -1, eitModule->measureImpedance(i, j));
			}
		}
	}
	// End Sequence of sending Calibration Data:
	eitModule->sendBinaryData(MODE_Data_acquisition_finished);
	eitModule->writeModesAndLeds(HIGH<<PIN_LED_YELLOW);
	// End Measurement:
	eitModule->endMeasure();
}

// Transmit measured Data:
void testFunction(){  
	// ----------  Impedance-Measurement tests:  ----------
	eitModule->beginMeasure();
	Serial.println(F("-------------------------------------------"));
	eitModule->setElectrodeInitCond();
	eitModule->disableModes();
	int16_t resistance_re;
	int16_t reactance_im;
	eitModule->ad5933ciruit->getOneFrequency(&resistance_re, &reactance_im);
	Serial.print(F("resistance_re = "));
	Serial.println(resistance_re);
	Serial.print(F("reactance_im = "));
	Serial.println(reactance_im);
	Serial.println(F("-------------------------------------------"));
	Serial.print(F("gainU = "));
	Serial.println(eitModule->getGainU(), 10);
	Serial.print(F("gainZ = "));
	Serial.println(eitModule->getGainZ(), 10);
	Serial.print(F("compensation_V_Re = "));
	Serial.println(eitModule->compensation_V_Re);
	Serial.print(F("compensation_V_Im = "));
	Serial.println(eitModule->compensation_V_Im);
	Serial.print(F("compensation_Z_Re = "));
	Serial.println(eitModule->compensation_Z_Re);
	Serial.print(F("compensation_Z_Im = "));
	Serial.println(eitModule->compensation_Z_Im);
	Serial.println(F("-------------------------------------------"));
	Serial.println(eitModule->measureImpedance(DISABLE_MUX, DISABLE_MUX));	// Measured Total-Idle-Impedance
	Serial.println(eitModule->measureImpedance(0, DISABLE_MUX));			// Measured Half-Idle-Impedance 1 [signal out = 0, GND = None]
	Serial.println(eitModule->measureImpedance(DISABLE_MUX, 0));			// Measured Half-Idle-Impedance 2 [signal out = None, GND = 0]
	Serial.println(eitModule->measureImpedance(0, 1));			// Measured Half-Idle-Impedance 2 [signal out = None, GND = 0]
	Serial.println(F("Measured Impedances: "));
	for(int i = 0; i < ELECTRODE_NUMBER-1; i++){
		Serial.println(eitModule->measureImpedance(i, (i+1)));
	}
	//Serial.println(F("-------------------------------------------"));
	//Serial.println(eitModule->measureImpedance(0,0));
	//eitModule->endMeasure();
	//// ----------  EIT-Measurement tests:  ----------
	//// Measure:
	//eitModule->beginMeasure();
	//eitModule->acquireEITdata();
	//// Transmit acquired Data:
	//Serial.println(F("Results of EIT-Measurements via meas- and stimulation-patterns: "));
	//for (int i = 0; i<MEASURE_NUMBER_PER_STIMULATION; i++){
		//Serial.println(eitModule->resultVsEIT[i], DIGITS_AFTER_POINT);
	//}
	//Serial.println(F("Last measured stimulation current: "));
	//Serial.println(eitModule->getLastMeasuredStimCurrent(), DIGITS_AFTER_POINT);
	//Serial.print(F("Meas Impedance Needle to E0:	"));
	//Serial.println(eitModule->measureImpedance(0, 16));
	//Serial.print(F("Meas Voltage @ E1 (Stimulation: E0 - E2), no correction:	"));
	//Serial.println(eitModule->measureVoltage(1, 0, 2, DirectPRINT_NONE, false), 4);
	//Serial.print(F("   other way around: Meas Voltage @ E1 (Stimulation: E2 - E0), no correction:	"));
	//Serial.println(eitModule->measureVoltage(1, 2, 0, DirectPRINT_NONE, false), 4);
	//Serial.print(F("Meas Voltage @ E1 (Stimulation: Needle - E2), no correction:	"));
	//Serial.println(eitModule->measureVoltage(1, 16, 2, DirectPRINT_NONE, false), 4);
	//Serial.print(F("Meas Voltage @ Needle (Stimulation: E0 - E2), no correction:	"));
	//Serial.println(eitModule->measureVoltage(16, 0, 2, DirectPRINT_NONE, false), 4);
	//Serial.print(F("Meas Voltage @ Needle (Stimulation: E0 - E8), no correction:	"));
	//Serial.println(eitModule->measureVoltage(16, 0, 8, DirectPRINT_NONE, false), 4);
	//eitModule->endMeasure();
	int a = 0;
	int b = 3;
	Serial.print(F("Meas Voltage:"));
	Serial.println(eitModule->measureVoltage(0, a, b, DirectPRINT_NONE, false), 4);
	Serial.println(eitModule->measureVoltage(1, a, b, DirectPRINT_NONE, false), 4);
	Serial.println(eitModule->measureVoltage(2, a, b, DirectPRINT_NONE, false), 4);
	Serial.println(eitModule->measureVoltage(3, a, b, DirectPRINT_NONE, false), 4);
	Serial.println(eitModule->measureVoltage(4, a, b, DirectPRINT_NONE, false), 4);
	Serial.println(eitModule->measureVoltage(5, a, b, DirectPRINT_NONE, false), 4);
}


// =======================================================================

void setup() {
	delay(2000);
	// Begin Serial communication:
	Serial.begin(SERIAL_BAUDRATE);
	// Start up EIT-Module:
	eitModule = new ImpedanceMultiSensingCircuit;
	if(eitModule->initIMSC()){
		// ERROR:
		exit(1);
	}
	// Get compensation Data / Config:
	updateCompensationValues();
	// Short Waiting time to ensure steady state:
	delay(100);
	// Inform about finished Initialization:
	Serial.println(F("READY"));
}

void loop() {
	// Wait until character has reached:
	while (Serial.available() <= 0){}
	// Read incoming character:
	incomingByte = Serial.read();
	// Select case:
	if (incomingByte == 'S') {
		// Read incoming character:
		while (Serial.available() <= 0){}
		int patternIdx = (int)Serial.read();
		// Set Stimulation Pattern:
		setStimulationPattern(patternIdx);
	} else if (incomingByte == 'M') {
		// Read incoming character:
		while (Serial.available() <= 0){}
		int patternIdx = (int)Serial.read();
		// Set Measurement Pattern:
		setMeasurementPattern(patternIdx);
	} else if (incomingByte == 'c') {
		// Collect and transmit data (NO leakage correction):
		startCollectingData(false);
	}else if (incomingByte == 'C') {
		// Collect and transmit data (including leakage correction):
		startCollectingData();
	} else if (incomingByte == 'T') {
		// Test Hardware:
		testFunction();
	} else if (incomingByte == '+') {
		// Measure all Magnitude:
		startCollectingData2(false);
	}else if (incomingByte == '*') {
		// Measure all Complex:
		startCollectingData2(true);
	} 
	/*else if (incomingByte == 'e') {
	// Test EEPROM:
		Serial.println(F("Saved Data:"));
		printMeasPattern();
		printStimPattern();
		writeMeasPatternToEEPROM(0);
		writeStimPatternToEEPROM(0);
		delay(100);
		Serial.println(F("Reread Data:"));
		readMeasPatternFromEEPROM(0);
		readStimPatternFromEEPROM(0);
		printMeasPattern();
		printStimPattern();
	} */
	else if (incomingByte == 'r') {
		// Read PAtterns:
		for(int i = 0; i < ELECTRODE_NUMBER; i++){
			Serial.print(F("Pattern Data ["));Serial.print(i);Serial.println(F("]:"));
			readMeasPatternFromEEPROM(i);
			readStimPatternFromEEPROM(i);
			printMeasPattern();
			printStimPattern();
		}
	} else if (incomingByte == 'K') {
	// Send Calibration Data:
		sendCalibData();
	}else if (incomingByte == 'B') {
	// Begin Measurement:
	eitModule->beginMeasure();
	} else if (incomingByte == 'Z') { // Removeable?
	// MEasure Impedance:
		while (Serial.available() <= 0){}
		uint8_t from = (uint8_t)Serial.read();
		while (Serial.available() <= 0){}
		uint8_t to = (uint8_t)Serial.read();
		long impedance = (long)eitModule->measureImpedance(from, to);
		if (impedance<1e8) Serial.print('0');
		if (impedance<1e7) Serial.print('0');
		if (impedance<1e6) Serial.print('0');
		if (impedance<1e5) Serial.print('0');
		if (impedance<1e4) Serial.print('0');
		if (impedance<1e3) Serial.print('0');
		if (impedance<1e2) Serial.print('0');
		if (impedance<1e1) Serial.print('0');
		Serial.print(impedance);
	}else if (incomingByte == 'L') {
		// Set flag for voltage Leakage Correction:
		voltageMeasureLeakageCorrection = true;
	}else if (incomingByte == 'l') {
		// Reset flag for voltage Leakage Correction:
		voltageMeasureLeakageCorrection = false;
	}
	else if(incomingByte == '@'){
		while (Serial.available() <= 0){}
		int8_t mode = (int8_t)Serial.read();
		while (Serial.available() <= 0){}
		int8_t from = (int8_t)Serial.read();
		while (Serial.available() <= 0){}
		int8_t to = (int8_t)Serial.read();
		while (Serial.available() <= 0){}
		int8_t meas = (int8_t)Serial.read();
		switch(mode){
			case 1:		// Mode1-->Magnitude of Voltage
				eitModule->measureVoltage(meas, from, to, DirectPRINT_MAGNITUDE, voltageMeasureLeakageCorrection); // Leakage-correction ON/OFF
				break;
			case 2:		// Mode2-->Complex Voltage
				eitModule->measureVoltage(meas, from, to, DirectPRINT_COMPLEX, voltageMeasureLeakageCorrection); // Leakage-correction ON/OFF
				break;
			case 3:		// Mode3-->Magnitude of Impedance
				eitModule->measureImpedance(from, to, DirectPRINT_MAGNITUDE);
				break;
			case 4:		// Mode4-->Complex Impedance
				eitModule->measureImpedance(from, to, DirectPRINT_COMPLEX);
				break;
			default: 
				break;
		}
	}else if(incomingByte == '%'){
		// GPIO:
		while (Serial.available() <= 0){}
		char mode = (uint8_t)Serial.read();
		while (Serial.available() <= 0){}
		uint8_t port = (uint8_t)Serial.read();
		if (mode == GPIO_READ_MODE){ // Read
			Serial.write((byte)gpioRead(port));
		}
		else if (mode == GPIO_WRITE_MODE){// Write
			while (Serial.available() <= 0){}
			uint8_t gpioValue = (uint8_t)Serial.read();
			gpioWrite(port, gpioValue);
		}
		else if (mode == GPIO_SET_DIR){// Write
			while (Serial.available() <= 0){}
			uint8_t gpioValue = (uint8_t)Serial.read();
			gpioMode(port, gpioValue);
		}
	}else if(incomingByte == 'F'){
		while (Serial.available() <= 0){}
		incomingByte = (Serial.read());
		int8_t frequency_kHz = (int8_t)incomingByte;                    //Read Frequency in kHz (2..99)
		if(frequency_kHz < 2) {frequency_kHz = 2;}
		if(frequency_kHz > 99) {frequency_kHz = 99;}
		// Reset Chip with new frequency:
		eitModule->endMeasure();
		//eitModule->ad5933ciruit->reset();
		eitModule->ad5933ciruit->setInternalClock(USE_INTERNAL_CLOCK);
		eitModule->measureFrequency = (unsigned long)frequency_kHz*1000;
		eitModule->ad5933ciruit->setStartFrequency(eitModule->measureFrequency);
		eitModule->ad5933ciruit->setIncrementFrequency(FREQ_INCR);
		eitModule->ad5933ciruit->setNumberIncrements(NUM_INCR);
		eitModule->ad5933ciruit->setPGAGain(PGA_GAIN_X1);
		eitModule->ad5933ciruit->setSettlingTimeCycles(SETTLING_TIME_CYCLES);
		
		// Self Calibration if nothing is connected (checked between Pin 0 and 1):
		eitModule->calibrationRoutine();
		// Disable Modes:
		//eitModule->disableModes();
	}else if (incomingByte == 'X') {
	// End Measurement:
		eitModule->endMeasure();
		eitModule->setElectrodeInitCond();
	}
}
