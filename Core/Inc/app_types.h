#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>

#define SPI_BUFF_SIZE 		(7)

#define OUT_X_L_ADDR		(0x2A)

// Macros for PA8
#define PA8_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define PA8_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define PA8_TOGGLE() HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8)

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    uint32_t timestamp;
} Accel_Sample;

#endif
