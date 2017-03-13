/* hw_conf.h
 *
 *  Created on: Jul 6, 2012
 *      Author: mobintu
 */

// define USE_STDPERIPH_DRIVER here to enable eclipse indexing of the stdperiph_driver


#ifndef HW_CONF_H_
#define HW_CONF_H_

#include "stm32f4xx_conf.h"
#include "usbd_cdc_core.h"
#include "usbd_cdc_vcp.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usb_core.h"
#include "math_utils.h"
#include "lsm9ds0.h"

//PB10/I2C2_SCL, PB11/I2C2_SDA
// XM Read Address 8bit 0x3B, Gyro 8bit Read 0xD7
// Used in lsm9ds0.c
#define LSM9DS0_GY_addr 0xD7
#define LSM9DS0_AM_addr 0x3B

#define IMU_SCL_PORT 		GPIOB
#define IMU_SDA_PORT 		GPIOB
#define IMU_SCL_PIN			GPIO_Pin_10
#define IMU_SDA_PIN			GPIO_Pin_11

#define IMU_SCL_SET_AF		GPIO_PinAFConfig(GPIOB,GPIO_PinSource10, GPIO_AF_I2C2)
#define IMU_SDA_SET_AF		GPIO_PinAFConfig(GPIOB,GPIO_PinSource11, GPIO_AF_I2C2)


#define IMU_I2C				I2C2
#define IMU_I2C_CLK(x) 		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,(x))
#define IMU_PORT_CLK(x)		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,(x))




//#define M_PWM_TickPeriod 1000
#define tic		TIM2->CNT=0
#define toc		TIM2->CNT

#define TIMEDWHILE(cond,statOk,timeout) {\
		volatile u32 end=toc+(timeout);\
		while((cond) && (statOk) && (toc<end));\
		statOk=(toc<end) && (statOk);\
}

void LedInit(void);
#define Led1(x) (x==0)?(GPIOB->BSRRH=GPIO_Pin_12):(GPIOB->BSRRL=GPIO_Pin_12)
#define Led2(x) (x==0)?(GPIOB->BSRRH=GPIO_Pin_2):(GPIOB->BSRRL=GPIO_Pin_2)




typedef struct {
	real gyr[3];
	real acc[3];
	real mag[3];
}sensors_struct;



uint8_t I2C_read(uint8_t addr,uint8_t reg,uint8_t *buf,uint8_t len);
uint8_t I2C_write(uint8_t addr,uint8_t reg,uint8_t *buf,uint8_t len);

void initHw(void);
void GenTimInit(void);
void delay_us(uint32_t delay);

void imu_init(void);



#endif /* HW_CONF_H_ */
