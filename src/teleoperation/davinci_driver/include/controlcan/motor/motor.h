#ifndef MOTOR_H
#define MOTOR_H
#include "vel.h"
#include "pos.h"
#include "controlcan/motor/epos.h"

#define MOTOR_ID	0x01

#define MOTOR_ERROR		(-1)

enum Motor_mode{
	Motor_mode_Profile_Position = 1,
	Motor_mode_Profile_Velocity = 3,
	Motor_mode_Position = -1,
	Motor_mode_Velocity = -2
};

/*!
 * Open can connection;
 * Configure motors;
 * \return 0 on success, MOTOR_ERROR (-1) on error
*/
int motor_init(void);


/*!
 * Turn on motors
 * \return 0 on success, MOTOR_ERROR (-1) on error
*/
int motor_enable(enum Epos_mode mode, int motor_id);


/*! Close can connection */
int motor_disable(int motor_id);

#endif // MOTOR_H
