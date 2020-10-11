#ifndef POS_H
#define POS_H

#include <stdint.h>

int pos_set_abs_position(double angle, int motor_id);

int pos_set_rel_position(double angle, int motor_id);

#endif // POS_H
