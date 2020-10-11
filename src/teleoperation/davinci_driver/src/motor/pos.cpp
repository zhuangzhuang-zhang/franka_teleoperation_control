#include <stdint.h>
#include <iostream>

#include "controlcan/motor/pos.h"
#include "controlcan/motor/motor.h"
#include "controlcan/canopen/SDO.h"
#include "controlcan/motor/epos.h"

static int _profile_position(double angle, uint8_t node_id){
	SDO_data d;
	d.nodeid = node_id;
	d.index = 0x607A;
	d.subindex = 0x00;
	
	int32_t qt;
	//84 is the gear ratio
	//4*512 is qc
	qt = int((angle/360.0)*4*1024*100);
	//std::cout<<"target position: "<<qt<<std::endl;

	d.data[0] = qt & 0xFF;
	d.data[1] = (qt >> 8) & 0xFF;
	d.data[2] = (qt >> 16)& 0xFF;
	d.data[3] = (qt >> 24)& 0xFF;
	
	SDO_write(&d);
	return 0;
}

int pos_set_abs_position(double angle, int motor_id){
	int success=0;

	success = _profile_position(angle, motor_id);

	//std::cout<<"target position status: "<<success<<std::endl;

	if (!success)
	{
		epos_Controlword(motor_id, Absolute_Pos_Start_Immed);
		return 0;
	}else
	{
		return 1;
	}
}

int pos_set_rel_position(double angle, int motor_id){
	int success=0;

	success = _profile_position(angle, motor_id);

	//std::cout<<"target position status: "<<success<<std::endl;
	if (!success)
	{
		epos_Controlword(motor_id, Relative_Pos_Start_Immed);
		return 0;
	}else
	{
		return 1;
	}

}

