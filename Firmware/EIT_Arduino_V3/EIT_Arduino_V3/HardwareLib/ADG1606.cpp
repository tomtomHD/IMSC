/* 
* ADG1606.cpp
*
* Created: 16.02.2021 16:49:34
* Author: Thomas Thurner
*/


#include "ADG1606.h"

//Constructor:
ADG1606::ADG1606(int _s0, int _s1, int _s2, int _s3, int _en, MCP23017* _mcp, int electAssignment[ELECTRODE_NUMBER]){
	/*
	MCP23017* mcp --> MCP23017-Object to which the MUX is connected
	int s0 ------------------> Pin of MCP23017 that is connected to S0 of MUX
	int s1 ------------------>  Pin of MCP23017 that is connected to S1 of MUX
	int s2 ------------------> Pin of MCP23017 that is connected to S2 of MUX
	int s3 ------------------> Pin of MCP23017 that is connected to S3 of MUX
	int en ------------------> Pin of MCP23017 that is connected to Enable of MUX (active HIGH)
	int* electAssignment-----> Assignment of the electrodes (Differences in Index can be corrected here), This is an int-array with size of ELECTRODE_NUMBER
	*/
	s0 = _s0;
	s1 = _s1;
	s2 = _s2;
	s3 = _s3;
	en = _en;
	mcp = _mcp;
	electrodeAssignment = electAssignment;
	
	activeChannel = -1;
	lastChannel = 0;
}

// Initialize (set Initial conditions of GPIO (Mode & Value @ MCP23017) )
void ADG1606::init_ADG1606(){
	mcp->pinMode(s0, OUTPUT); mcp->digitalWrite(s0, LOW);
	mcp->pinMode(s1, OUTPUT); mcp->digitalWrite(s1, LOW);
	mcp->pinMode(s2, OUTPUT); mcp->digitalWrite(s2, LOW);
	mcp->pinMode(s3, OUTPUT); mcp->digitalWrite(s3, LOW);
	mcp->pinMode(en, OUTPUT); mcp->digitalWrite(en, LOW);
	activeChannel = -1;
	lastChannel = 0;
}

// Switch to Channel (slow):
void ADG1606::channel(int8_t _channel){
	/*
	int8_t _channel --> Channel number to switch to (0...15, automatically enabled)
	*/
	if(_channel < 0){	// Disable if < 0
		disable();
		return;
	}
	if (_channel >= ELECTRODE_NUMBER){	// Correct if too high
		_channel = ELECTRODE_NUMBER-1;
	}
	int channelIntern = electrodeAssignment[_channel];
	mcp->digitalWrite(s0, (channelIntern  ) % 2);
	mcp->digitalWrite(s1, (channelIntern/2) % 2);
	mcp->digitalWrite(s2, (channelIntern/4) % 2);
	mcp->digitalWrite(s3, (channelIntern/8) % 2);
	enable();
	activeChannel = _channel;
	lastChannel = _channel;
}

// Overwrite activeChannel if channel is changed by manipulating overlying IOExpander (MCP23017):
void ADG1606::overwriteActiveChannelVar(int8_t _channel){
	activeChannel = _channel;
	if (activeChannel >= 0){lastChannel = _channel;}
}

// Enable Chip:
void ADG1606::enable(){
	mcp->digitalWrite(en, HIGH);
	activeChannel = lastChannel;
}

// Disable Chip:
void ADG1606::disable(){
	mcp->digitalWrite(en, LOW);
	activeChannel = -1;
}

// Get active Channel ( negative if Chip is disabled):
int ADG1606::getChannel(){
	return activeChannel;
}
