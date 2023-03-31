/* 
* ADG1606.h
*
* Created: 16.02.2021 16:49:34
* Author: Thomas Thurner
*/


#ifndef __ADG1606_H__
#define __ADG1606_H__

#include <Arduino.h>
#include "MCP23017.h"

#define ELECTRODE_NUMBER (16)
#define NEEDLE_NUMBER    (1)

// Class for ADG1606 MUX-IC, connected to a MCP23017-GPIO-expander:
class ADG1606{
	
	// Functions:
	public:
		
		// Constructor:
		ADG1606(int _s0, int _s1, int _s2, int _s3, int _en, MCP23017* _mcp, int electAssignment[ELECTRODE_NUMBER]);
		
		// Initialize (set Initial conditions of GPIO (Mode & Value @ MCP23017) )
		void init_ADG1606();
	
		// Switch to Channel (slow):
		void channel(int8_t _channel);
		
		// Overwrite activeChannel if channel is changed by manipulating overlying IOExpander (MCP23017):
		void overwriteActiveChannelVar(int8_t _channel);
	
		// Enable Chip:
		void enable();
	
		// Disable Chip:
		void disable();
	
		// Get active Channel ( negative if Chip is disabled):
		int getChannel();
		
	// Variables:
	private:
	
		MCP23017* mcp;			// MCP23017-Object to which the MUX is connected
		int activeChannel;				// Active Channel (0...15), negative if MUX is disabled
		int lastChannel;				// Last activated or current active Channel
								
	public:
		
		int s0;							// Pin of MCP23017 that is connected to S0 of MUX
		int s1;							// Pin of MCP23017 that is connected to S1 of MUX
		int s2;							// Pin of MCP23017 that is connected to S2 of MUX
		int s3;							// Pin of MCP23017 that is connected to S3 of MUX
		int en;							// Pin of MCP23017 that is connected to Enable of MUX (active HIGH)
		int* electrodeAssignment;
		// Assignment of the electrodes (Differences in Index can be corrected here)
	
};

#endif //__ADG1606_H__
