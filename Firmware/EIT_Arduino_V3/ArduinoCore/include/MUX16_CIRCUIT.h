/* 
* MUX16_CIRCUIT.h
*
* Created: 17.02.2021 09:32:13
* Author: P42164
*/


#ifndef __MUX16_CIRCUIT_H__
#define __MUX16_CIRCUIT_H__


#include "ADG1606.h"
#include "Adafruit_MCP23017.h"

#define MCP23017_GPIO_QUANTITY (16)
#define I2C_CLOCKSPEED (400000L)
#define I2C_8BIT_DELAY_US (8000000L/I2C_CLOCKSPEED)

#define SIGMUX_S0  6
#define SIGMUX_S1  14
#define SIGMUX_S2  5
#define SIGMUX_S3  13
#define SIGMUX_EN  15

#define MEASMUX_S0 12
#define MEASMUX_S1 3
#define MEASMUX_S2 11
#define MEASMUX_S3 2
#define MEASMUX_EN 4

#define GNDMUX_S0  1
#define GNDMUX_S1  9
#define GNDMUX_S2  0
#define GNDMUX_S3  8
#define GNDMUX_EN  10

#define ELECTRODE_ASSIGNMENT {1,0,3,2,5,4,7,6,9,8,11,10,13,12,15,14}

class MUX16_CIRCUIT
{
//variables
	public:
	protected:
	private:
		int signalMux_actChannel;
		int measureMux_actChannel;
		int groundMux_actChannel;
		ADG1606* signalMux;
		ADG1606* measureMux;
		ADG1606* groundMux;
		Adafruit_MCP23017* mcpExpander;
		word gpioReg;		// GPIO-Register Template
		int electrodeAssignment[MCP23017_GPIO_QUANTITY] = ELECTRODE_ASSIGNMENT;

//functions
public:

	// Default Constructor:
	MUX16_CIRCUIT();

	// Constructor:
	MUX16_CIRCUIT(Adafruit_MCP23017* mcp, ADG1606* sigMux, ADG1606* measMux, ADG1606* gndMux);
	
	// Destructor:
	~MUX16_CIRCUIT();
	
	// Initialize:
	void begin();
	
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
	
protected:
private:
	MUX16_CIRCUIT( const MUX16_CIRCUIT &c );
	MUX16_CIRCUIT& operator=( const MUX16_CIRCUIT &c );

}; //MUX16_CIRCUIT

#endif //__MUX16_CIRCUIT_H__
