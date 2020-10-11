#ifndef CAN_H
#define CAN_H
#include <stdint.h>

int can_open();

int can_write(uint16_t id, uint8_t length, uint8_t* data);

void can_close();

#endif // CAN_H
