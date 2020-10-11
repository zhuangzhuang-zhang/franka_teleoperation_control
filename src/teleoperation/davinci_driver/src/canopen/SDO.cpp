#include "controlcan/canopen/SDO.h"
#include "controlcan/can/can.h"

int SDO_write(SDO_data* d){
	uint16_t cob;
	uint8_t ccd; //length of data
	cob = SDO_RX + d->nodeid;
	ccd = 0x22;
	
	uint8_t data[8];
	data[0] = ccd;
	data[1] = d->index & 0xFF;
	data[2] = (d->index >> 8) & 0xFF;
	data[3] = d->subindex;
	data[4] = d->data[0];
	data[5] = d->data[1];
	data[6] = d->data[2];
	data[7] = d->data[3];
	
	can_write(cob, 8, data);
	return 0;
}
