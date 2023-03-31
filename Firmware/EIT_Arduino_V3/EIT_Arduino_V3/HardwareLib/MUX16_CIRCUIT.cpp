/* 
* MUX16_CIRCUIT.cpp
*
* Created: 17.02.2021 09:32:13
* Author: Thomas Thurner
*/


#include "MUX16_CIRCUIT.h"

// Default Constructor:
MUX16_CIRCUIT::MUX16_CIRCUIT(){
	// Set Reference to GPIO-Expander:
	mcpExpander = new MCP23017();
		
	// Set Reference to MUX-Objects:
	signalMux = new ADG1606(SIGMUX_S0, SIGMUX_S1, SIGMUX_S2, SIGMUX_S3, SIGMUX_EN, mcpExpander, electrodeAssignment);
	measureMux = new ADG1606(MEASMUX_S0, MEASMUX_S1, MEASMUX_S2, MEASMUX_S3, MEASMUX_EN, mcpExpander, electrodeAssignment);
	groundMux = new ADG1606(GNDMUX_S0, GNDMUX_S1, GNDMUX_S2, GNDMUX_S3, GNDMUX_EN, mcpExpander, electrodeAssignment);
		
	// Init GPIO-Register Template
	gpioReg = 0;
}

// Constructor:
MUX16_CIRCUIT::MUX16_CIRCUIT(MCP23017* mcp, ADG1606* sigMux, ADG1606* measMux, ADG1606* gndMux)
{
	// Set Reference to GPIO-Expander:
	mcpExpander = mcp;
	
	// Set Reference to MUX-Objects:
	signalMux = sigMux;
	measureMux = measMux;
	groundMux = gndMux;
	
	// Init GPIO-Register Template
	gpioReg = 0;
	
} //MUX16_CIRCUIT

// default destructor
MUX16_CIRCUIT::~MUX16_CIRCUIT()
{
	delete mcpExpander;
	delete signalMux;
	delete measureMux;
	delete groundMux;
} //~MUX16_CIRCUIT

// Initialize:
int8_t MUX16_CIRCUIT::begin(int addr){
	
	// Initialize GPIO Expander:
	if(mcpExpander->begin(addr)){
		// Return Error:
		return 1;
	}
	
	// Initialize MUXs:
	signalMux->init_ADG1606();
	measureMux->init_ADG1606();
	groundMux->init_ADG1606();
	
	// Get active Channels of MUX-ICs:
	signalMux_actChannel = signalMux->getChannel();
	measureMux_actChannel = measureMux->getChannel();
	groundMux_actChannel = groundMux->getChannel();
	
	// Init GPIO-Register Template
	gpioReg = 0;
	changeGPIOregWord(signalMux, signalMux->getChannel());
	changeGPIOregWord(measureMux, measureMux->getChannel());
	changeGPIOregWord(groundMux, groundMux->getChannel());
	writePreparedChannelsToMux();
	
	// I2C speed:
	Wire.setClock(I2C_CLOCKSPEED);	// Set Clock-speed for I2C to 400 kHz
	
	// Return Success:
	return 0;
}

// Prepare Signal MUX for changing Channel:
void MUX16_CIRCUIT::prepareSignalMux(int channel){
	changeGPIOregWord(signalMux, channel);
	preparedSignalMux_actChannel = channel;
}

// Prepare Signal MUX for changing Channel:
void MUX16_CIRCUIT::prepareMeasureMux(int channel){
	changeGPIOregWord(measureMux, channel);
	preparedMeasureMux_actChannel = channel;
}

// Prepare Signal MUX for changing Channel:
void MUX16_CIRCUIT::prepareGroundMux(int channel){
	changeGPIOregWord(groundMux, channel);
	preparedGroundMux_actChannel = channel;
}

// Write Channels (GPIO-Register Template) to MCP23017:
void MUX16_CIRCUIT::writePreparedChannelsToMux(){
	mcpExpander->writeGPIOAB(gpioReg);
	signalMux_actChannel =  preparedSignalMux_actChannel;
	measureMux_actChannel = preparedMeasureMux_actChannel;
	groundMux_actChannel =  preparedGroundMux_actChannel;
	signalMux->overwriteActiveChannelVar(signalMux_actChannel);
	measureMux->overwriteActiveChannelVar(measureMux_actChannel);
	groundMux->overwriteActiveChannelVar(groundMux_actChannel);
}

// Change GPIO-Register Template according to Channel-activation:
void MUX16_CIRCUIT::changeGPIOregWord(ADG1606* someMux, int activateChannel){
	#ifdef DEBUG_MODE
		Serial.print("someMux: activateChannel ");
		Serial.println(activateChannel);
		Serial.print("gpioReg (before) = ");
		Serial.println(gpioReg);
	#endif
	//Disable and Return:
	if(activateChannel < 0 || activateChannel >= ELECTRODE_NUMBER){
		gpioReg &= ~(1<<someMux->en);
		#ifdef DEBUG_MODE
			Serial.print("gpioReg (after) = ");
			Serial.println(gpioReg);
		#endif
		return;
	// Enable:
	}else{
		gpioReg |= (1<<someMux->en);
	}
	// Get pin-values:
	int internalChannel = someMux->electrodeAssignment[activateChannel];
	int pinValues[4] = {0};
	for(int i = 0; i < 4; i++){
		pinValues[i] = (internalChannel/(1<<i)) % 2;
	}
	// Write values to pin-position in 'gpioReg':
	int pinIdx[4] = {someMux->s0, someMux->s1, someMux->s2, someMux->s3};
	for(int i = 0; i < 4; i++){
		(pinValues[i] > 0) ? gpioReg |= (1<<pinIdx[i]) : gpioReg &= ~(1<<pinIdx[i]);
	}
	#ifdef DEBUG_MODE
		Serial.print("gpioReg (after) = ");
		Serial.println(gpioReg);
	#endif
	
}


// Get Channel of Signal Mux:
int8_t MUX16_CIRCUIT::getSignalMuxChannel(){
	return signalMux_actChannel;
}
// Get Channel of Measure Mux:
int8_t MUX16_CIRCUIT::getMeasureMuxChannel(){
	return measureMux_actChannel;
}
// Get Channel of Ground Mux:
int8_t MUX16_CIRCUIT::getGroundMuxChannel(){
	return groundMux_actChannel;
}