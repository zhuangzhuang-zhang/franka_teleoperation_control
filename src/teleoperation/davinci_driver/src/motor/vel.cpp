#include <stdint.h>

#include "controlcan/motor/vel.h"
#include "controlcan/motor/motor.h"
#include "controlcan/canopen/SDO.h"

static int _vel_speed(int32_t vel, uint8_t node_id){
	SDO_data d;
	d.nodeid = node_id;
	d.index = 0x206B;
	d.subindex = 0x00;
	d.data[0] = vel & 0xFF;
	d.data[1] = (vel >> 8) & 0xFF;
	d.data[2] = 0x00;
	d.data[3] = 0x00;
	
	SDO_write(&d);
	return 0;
}

int vel_set_speed(int32_t vel){
	return _vel_speed(vel, MOTOR_ID);
}	
