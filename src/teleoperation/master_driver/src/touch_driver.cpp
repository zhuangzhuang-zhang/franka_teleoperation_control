#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <stdio.h>
#include <assert.h>
#include <math.h> 

#define linux
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"


#include "robot_msgs/touch.h"
#include "robot_msgs/omega.h"

using namespace std;

double position_x;
double position_y;
double position_z;
double position_x1;
double position_y1;
double position_z1;

double gimbal_angle0;
double gimbal_angle1;
double gimbal_angle2;

int key_button = 0;

typedef struct
{
    HDboolean       m_buttonState;         
    hduVector3Dd    m_devicePosition;       
    hduVector3Dd    m_gimbal_angle;        
    HDErrorInfo     m_error;
} DeviceData;

static DeviceData gServoDeviceData;

HDCallbackCode HDCALLBACK updateDeviceCallback(void *pUserData){
    int nButtons = 0;
    hdBeginFrame(hdGetCurrentDevice());

    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    if(nButtons==1)
        key_button = 1;
//    else if(nButtons==2)
//        key_button = 2;
    else
        key_button = 0;
    //cout<<"button: "<<nButtons<<endl;

    gServoDeviceData.m_buttonState =
        (nButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;

    hdGetDoublev(HD_CURRENT_POSITION, gServoDeviceData.m_devicePosition);
    position_x = gServoDeviceData.m_devicePosition[0];
    position_y = gServoDeviceData.m_devicePosition[1];
    position_z = gServoDeviceData.m_devicePosition[2];
    //cout<<"position_x: "<<position_x<<"    position_y: "<<position_y<<"    position_z: "<<position_z<<endl;

    Eigen::Matrix<double,3,3> modify_matrix;
    modify_matrix(0,0) = 0;	    modify_matrix(0,1) = 0;			modify_matrix(0,2) = -1;
    modify_matrix(1,0) = -1;	modify_matrix(1,1) = 0;	        modify_matrix(1,2) = 0;
    modify_matrix(2,0) = 0;		modify_matrix(2,1) = 1;		    modify_matrix(2,2) = 0;
	Eigen::Matrix<double,3,1> position_orign,position_modify;
	position_orign(0,0) = position_x;	position_orign(1,0) = position_y;	position_orign(2,0) = position_z;
	position_modify = modify_matrix * position_orign;
	position_x1 = position_modify(0,0);	position_y1 = position_modify(1,0);	position_z1 = position_modify(2,0);
    //cout<<"position_x: "<<position_x1<<"    position_y: "<<position_y1<<"    position_z: "<<position_z1<<endl;


    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gServoDeviceData.m_gimbal_angle);
    gimbal_angle0 = gServoDeviceData.m_gimbal_angle[0];
    gimbal_angle1 = gServoDeviceData.m_gimbal_angle[1];
    gimbal_angle2 = gServoDeviceData.m_gimbal_angle[2];
    //cout<<"gimbal_angle0: "<<gimbal_angle0<<"    gimbal_angle1: "<<gimbal_angle1<<"    gimbal_angle2: "<<gimbal_angle2<<endl;

    gServoDeviceData.m_error = hdGetError();
    hdEndFrame(hdGetCurrentDevice());

    return HD_CALLBACK_CONTINUE;
}

int main(int argc, char* argv[]){

    HDSchedulerHandle hUpdateHandle = 0;
    HDErrorInfo error;

    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError())){
        hduPrintError(stderr, &error, "Failed to initialize the device");
        fprintf(stderr, "\nPress any key to quit.\n");
        return -1;
    }


    hUpdateHandle = hdScheduleAsynchronous(
        updateDeviceCallback, 0, HD_MAX_SCHEDULER_PRIORITY);

    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError())){
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        return -1;
    }

    ros::init(argc, argv, "touch_driver_node");
    ros::NodeHandle n;

    ros::Publisher touch_map_publish = n.advertise<robot_msgs::omega>("omega1/omega_map", 100);

    ros::Rate loop_rate(300);
    while(ros::ok()){
		robot_msgs::omega msg;
		msg.data.resize(6);
        msg.button.resize(1);
        msg.data[0] = position_x1/1000.0;	msg.data[1] = position_y1/1000.0;	msg.data[2] = position_z1/1000.0;
		msg.data[3] = gimbal_angle2*-1;	msg.data[4] = gimbal_angle1*-1;	msg.data[5] = gimbal_angle0;
		msg.button[0] = key_button*0.3;
		touch_map_publish.publish(msg);

        loop_rate.sleep();
    }

    hdStopScheduler();
    hdUnschedule(hUpdateHandle);
    hdDisableDevice(hHD);

    return 0;
}
