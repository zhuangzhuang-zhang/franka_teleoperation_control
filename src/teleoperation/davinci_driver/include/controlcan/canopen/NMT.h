#ifndef NMT_H
#define NMT_H
#include <stdint.h>

enum NMT_transitions {
	NMT_Start_Node = 0x01,
	NMT_Stop_Node  = 0x02,
	NMT_Enter_PreOperational = 0x80,
	NMT_Reset_Node = 0x81
};

int NMT_change_state(uint8_t nodeid, enum NMT_transitions state);


#endif // NMT_H
