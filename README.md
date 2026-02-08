# stm32-freertos-imu-driver
Realtime interrupt driven IMU processing pipeline on STM32 using FreeRTOS. This project implements a full embedded sensor processing stack, from SPI register reads to motion event detection using task separation and deterministic scheduling.  Goal is to model how production embedded systems handle sensors: ISRs collect data, tasks process meaning
