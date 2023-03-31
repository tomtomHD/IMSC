/* 
* MUX16_CIRCUIT.h
*
* Created: 17.02.2021 09:32:13
* Author: Thomas Thurner
*/


#ifndef __MUX16_CIRCUIT_H__
#define __MUX16_CIRCUIT_H__


#include "ADG1606.h"
#include "MCP23017.h"

#define MCP23017_GPIO_QUANTITY (16)
#define I2C_CLOCKSPEED (400000L)
#define I2C_8BIT_DELAY_US (8000000L/I2C_CLOCKSPEED)

#define NELECTRODES	(16)

#define DISABLE_MUX (-1)

#define SIGMUX_S0  0
#define SIGMUX_S1  1
#define SIGMUX_S2  2
#define SIGMUX_S3  3
#define SIGMUX_EN  12

#define MEASMUX_S0 4
#define MEASMUX_S1 5
#define MEASMUX_S2 6
#define MEASMUX_S3 7
#define MEASMUX_EN 13

#define GNDMUX_S0  8
#define GNDMUX_S1  9
#define GNDMUX_S2  10
#define GNDMUX_S3  11
#define GNDMUX_EN  14

#define ELECTRODE_ASSIGNMENT {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}
	
//#define DEBUG_MODE

class MUX16_CIRCUIT
{
//variables
	public:
	protected:
	private:
		int8_t signalMux_actChannel;
		int8_t measureMux_actChannel;
		int8_t groundMux_actChannel;
		int8_t preparedSignalMux_actChannel;
		int8_t preparedMeasureMux_actChannel;
		int8_t preparedGroundMux_actChannel;
		ADG1606* signalMux;
		ADG1606* measureMux;
		ADG1606* groundMux;
		MCP23017* mcpExpander;
		word gpioReg;		// GPIO-Register Template
		int electrodeAssignment[MCP23017_GPIO_QUANTITY] = ELECTRODE_ASSIGNMENT;

//functions
public:

	// Default Constructor:
	MUX16_CIRCUIT();

	// Constructor:
	MUX16_CIRCUIT(MCP23017* mcp, ADG1606* sigMux, ADG1606* measMux, ADG1606* gndMux);
	
	// Destructor:
	~MUX16_CIRCUIT();
	
	// Initialize:
	int8_t begin(int addr = 0);
	
	// Prepare Signal MUX for changing Channel:
	void prepareSignalMux(int channel);
	
	// Prepare Signal MUX for changing Channel:
	void prepareMeasureMux(int channel);
	
	// Prepare Signal MUX for changing Channel:
	void prepareGroundMux(int channel);
	
	// Write Channels (GPIO-Register Template) to MCP23017:
	void writePreparedChannelsToMux();
	
	// Change GPIO-Register Template according to Channel-activation:
	void changeGPIOregWord(ADG1606* someMux, int activateChannel);
	
	// Get Channel of Signal Mux:
	int8_t getSignalMuxChannel();
	// Get Channel of Measure Mux:
	int8_t getMeasureMuxChannel();
	// Get Channel of Ground Mux:
	int8_t getGroundMuxChannel();
	
protected:
private:
	MUX16_CIRCUIT( const MUX16_CIRCUIT &c );
	MUX16_CIRCUIT& operator=( const MUX16_CIRCUIT &c );

}; //MUX16_CIRCUIT

#endif //__MUX16_CIRCUIT_H__
