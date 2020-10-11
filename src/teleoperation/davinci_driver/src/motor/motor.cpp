#include <unistd.h>
#include <iostream>
#include "controlcan/motor/motor.h"
#include "controlcan/motor/epos.h"
#include "controlcan/canopen/NMT.h"
#include "controlcan/canopen/canopen.h"

using namespace std;

//------------------
int motor_setmode(enum Epos_mode mode, int motor_id){
	epos_Modes_of_Operation(motor_id, mode);
	return 0;
}

int motor_init(void){
	NMT_change_state(CANOPEN_BROADCAST_ID, NMT_Reset_Node);
	//leep(1);
	NMT_change_state(CANOPEN_BROADCAST_ID, NMT_Enter_PreOperational);
	//set motor mode
	//motor_setmode(Velocity_Mode);
	return 0;
}


int motor_enable(enum Epos_mode mode, int motor_id){
	NMT_change_state(CANOPEN_BROADCAST_ID, NMT_Start_Node);
	//sleep(1);
	motor_setmode(mode, motor_id);
	sleep(1);
	epos_Set_Parameter(motor_id, 0x6081,0x00,5000);
	//sleep(1);
	epos_Controlword(motor_id, Shutdown);
	sleep(1);
	epos_Controlword(motor_id, Switch_On_And_Enable_Operation);

	cout<<"motor "<<motor_id<<" enabled! "<<endl;
	return 0;
}


int motor_disable(int motor_id){
	epos_Controlword(motor_id, Disable_Voltage);
	
	return 0;
}



