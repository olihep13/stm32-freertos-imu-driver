#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>

#define SPI_BUFF_SIZE 		(7)

#define OUT_X_L_ADDR		(0x28)

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    uint32_t timestamp;
} Accel_Sample;

#endif
