#include "controlcan/motor/epos.h"
#include "controlcan/canopen/canopen.h"



int epos_Modes_of_Operation(uint8_t node_id, enum Epos_mode mode){
	SDO_data d;
	d.nodeid = node_id;
	d.index = 0x6060;
	d.subindex = 0x00;
	d.data[0] = mode & 0xFF;
	d.data[1] = 0x00;
	d.data[2] = 0x00;
	d.data[3] = 0x00;
	
	return SDO_write(&d);
}

int epos_Controlword(uint8_t node_id, enum Epos_control control){
	SDO_data d;
	d.nodeid = node_id;
	d.index = 0x6040;
	d.subindex = 0x00;
	d.data[0] = control & 0xFF;
	d.data[1] = 0x00;
	d.data[2] = 0x00;
	d.data[3] = 0x00;
	return SDO_write(&d);
}

int epos_Set_Parameter(uint8_t node_id, uint8_t index, uint8_t subindex, int32_t value){
	SDO_data d;
	d.nodeid = node_id;
	d.index = 0x6081;
	d.subindex = 0x00;

	d.data[0] = value & 0xFF;
	d.data[1] = (value >> 8) & 0xFF;
	d.data[2] = (value >> 16) & 0xFF;
	d.data[3] = (value >> 24) & 0xFF;
	
	return SDO_write(&d);
}