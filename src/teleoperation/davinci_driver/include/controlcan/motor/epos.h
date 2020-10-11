#ifndef EPOS_H
#define EPOS_H

#include <stdint.h>

enum Epos_mode{
	Profile_Position_Mode = 1,
	Profile_Velocity_Mode = 3,
	Position_Mode = -1,
	Velocity_Mode = -2
};

enum Epos_control{
	Shutdown = 0x06,
	Switch_On_And_Enable_Operation = 0x0F,
	Disable_Voltage = 0x00,
	Absolute_Pos_Start_Immed = 0x3F,
	Relative_Pos_Start_Immed = 0x7F
};

int epos_Modes_of_Operation(uint8_t nodeid, enum Epos_mode mode);

int epos_Controlword(uint8_t node_id, enum Epos_control control);

int epos_Set_Parameter(uint8_t node_id, uint8_t index, uint8_t subindex, int value);

#endif // EPOS_H
