#ifndef SDO_H
#define SDO_H

#include <stdint.h>

enum SDO_ID{
	SDO_TX = 0x580,
	SDO_RX = 0x600
};

typedef struct{
	uint8_t  nodeid;
	uint16_t index;
	uint8_t  subindex;
	uint8_t  data[4];
} SDO_data;

int SDO_write(SDO_data* d);

#endif // SDO_H
