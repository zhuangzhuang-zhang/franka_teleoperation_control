#include "controlcan/canopen/NMT.h"
#include "controlcan/can/can.h"

int NMT_change_state(uint8_t nodeid, enum NMT_transitions state){
	uint8_t data[2];
	data[0] = state;
	data[1] = nodeid;
	
	return can_write(0x000, 2, data);
}
