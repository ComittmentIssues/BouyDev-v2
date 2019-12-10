/*
 * IMU.h
 *
 *  Created on: Jul 13, 2019
 *      Author: Jamie
 */

#ifndef IMU_H_
#define IMU_H_

#include "../IMU/tm_stm32f4_i2c.h"
#include "../IMU/tm_stm32f4_mpu6050.h"
#include "../IMU/tm_stm32f4_lis302dl_lis3dsh.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "eeprom.h"

#define ADDRESS 0xD0
#define SAMPLE_RATE 2 //Hz
#define SAMPLE_TIME_MIN 1
#define Sample_Timer TIM7
#define Sample_PSC  65535
#define ACC_FSR 8
#define ACC_Resolution 8192 //LSB/g
#define __numSamples()  SAMPLE_RATE*SAMPLE_TIME_MIN*60
#define USE_EXT_IMU
#define USE_INT_IMU

#ifdef USE_EXT_IMU
#define NAxis 6 //incl magnetometer
#else
#define NAxis 3 //accelerator only
#endif

uint8_t sample_finished;
char buff[200];
float samples[NAxis];
uint8_t init_IMU(void);
void init_Timer(void);
void deinit_Timer(void);
#endif /* IMU_H_ */