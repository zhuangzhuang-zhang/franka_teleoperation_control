#include <iostream>
#include <unistd.h>
#include <stdint.h>

using namespace std;

extern "C" {
	#include "controlcan/controlcan.h"
}

#include "controlcan/can/can.h"

int CANIDX = 0;

int can_open(){
	if(VCI_OpenDevice(VCI_USBCAN2, 0, 0) != 1){
		cout<<"open device error"<<endl;
		return 1;
	}
	cout<<"CAN device open success"<<endl;
	

	VCI_INIT_CONFIG config;
	config.AccCode = 0;
	config.AccMask = 0xffffffff;
	config.Filter = 1;
	config.Mode = 0;
	//baud rate 500K:0x00 0x1C
	//			125K:0x03 0x1C
	//         1000K:0x00 0x14	
	config.Timing0 = 0x00;
	config.Timing1 = 0x14;
	if(VCI_InitCAN(VCI_USBCAN2, 0, CANIDX, &config) != 1){
		cout<<"init CAN error"<<endl;
		VCI_CloseDevice(VCI_USBCAN2, 0);
		return 1;
	}
	cout<<"CAN init success"<<endl;
	

	if(VCI_StartCAN(VCI_USBCAN2, 0, CANIDX) != 1){
		cout<<"Start CAN error"<<endl;
		VCI_CloseDevice(VCI_USBCAN2, 0);
		return 1;
	}
	cout<<"CAN start success"<<endl;
	return 0;
}

int can_write(uint16_t id, uint8_t length, uint8_t* data){
	VCI_CAN_OBJ sendMsg[1];
	
	sendMsg[0].ID = id;//sdo 发送的id用600+node_ID
	sendMsg[0].SendType = 0;
	sendMsg[0].RemoteFlag = 0;
	sendMsg[0].ExternFlag = 0;
	sendMsg[0].DataLen = length;
	
	sendMsg[0].Data[0] = data[0];//length of data, 2F: 一个字节; 22: 长度未指定;
	sendMsg[0].Data[1] = data[1];//index
	sendMsg[0].Data[2] = data[2];//index
	sendMsg[0].Data[3] = data[3];//sub_index
	sendMsg[0].Data[4] = data[4];//data
	sendMsg[0].Data[5] = data[5];
	sendMsg[0].Data[6] = data[6];
	sendMsg[0].Data[7] = data[7];
	
	int success_frame_number = 0;
	success_frame_number = VCI_Transmit(VCI_USBCAN2, 0, CANIDX, sendMsg, 1);
	if(success_frame_number == 1){
		//cout<< "send success, canopen index is: 0x"<< hex<< int(data[2])<< int(data[1])<< endl;
		//cout<< "data is: "<< hex<< int(data[5])<< int(data[4])<< endl;
		return 0;
		//sleep(1);
	}else{
		cout<<"error: send error"<<endl;
		return 1;
		sleep(1);
	}
}

void can_close(){
	VCI_CloseDevice(VCI_USBCAN2, 0);
}
