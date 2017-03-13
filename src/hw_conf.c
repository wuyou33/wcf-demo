/*
 * hw_conf.c
 *
 *  Created on: Jul 6, 2012
 *      Author: mobintu
 */
#include "hw_conf.h"
#include "stdio.h"




void LedInit(void){
	GPIO_InitTypeDef      GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}




void GenTimInit(void){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = (84-1); //us CHECK prescaler
	TIM_TimeBaseStructure.TIM_ClockDivision = 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_Cmd(TIM2, ENABLE);
}




void delay_us(u32 delay){
	volatile u32 start=toc;
	while((toc-start)<delay);
}


void imu_init(void){
	GPIO_InitTypeDef      GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;
	uint8_t statOk=!0;
	int i;

	I2C_DeInit(IMU_I2C);
	I2C_Cmd(IMU_I2C, DISABLE);
	I2C_SoftwareResetCmd(IMU_I2C, ENABLE);
	I2C_SoftwareResetCmd(IMU_I2C, DISABLE);

	IMU_I2C_CLK(ENABLE);
	IMU_PORT_CLK(ENABLE);

	GPIO_InitStructure.GPIO_Pin = IMU_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(IMU_SCL_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = IMU_SDA_PIN;
	GPIO_Init(IMU_SDA_PORT, &GPIO_InitStructure);

	IMU_SDA_PORT->BSRRL = IMU_SDA_PIN;
	IMU_SCL_PORT->BSRRL = IMU_SCL_PIN;

	i=8;
	while( (i--) && ((IMU_SDA_PORT->IDR&IMU_SDA_PIN)==0)){
		delay_us(200);
		IMU_SCL_PORT->BSRRH = IMU_SCL_PIN;
		delay_us(200);
		IMU_SCL_PORT->BSRRL = IMU_SCL_PIN;
	}

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_OwnAddress1 = 0x01;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed=100e3;
	I2C_InitStructure.I2C_DutyCycle=I2C_DutyCycle_16_9;
	I2C_Init(IMU_I2C, &I2C_InitStructure);
	I2C_StretchClockCmd(IMU_I2C,ENABLE);
	I2C_Cmd(IMU_I2C, ENABLE);

	// Pass pin control to I2C
	IMU_SCL_SET_AF;
	IMU_SDA_SET_AF;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = IMU_SCL_PIN;
	GPIO_Init(IMU_SCL_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = IMU_SDA_PIN;
	GPIO_Init(IMU_SDA_PORT, &GPIO_InitStructure);

	delay_us(1e3);


	lsm9ds0.i2c_rd = &I2C_read;
	lsm9ds0.i2c_wr = &I2C_write;
	lsm9ds0.delay_us = &delay_us;
	lsm9ds0.gyr_stat = 123;
	lsm9ds0.acc_stat = 456;
	lsm9ds0.mag_stat = 789;
	lsm9ds0.temp = 7;
	lsm9ds0_Init();
}

uint8_t I2C_write(uint8_t addr,uint8_t reg,uint8_t *buf,uint8_t len){
	uint8_t statOk=!0;

	I2C_AcknowledgeConfig(IMU_I2C, ENABLE);
	I2C_GenerateSTART(IMU_I2C,ENABLE);
	TIMEDWHILE((!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_MODE_SELECT)),statOk,5e3);
	I2C_Send7bitAddress(IMU_I2C,addr,I2C_Direction_Transmitter);
	TIMEDWHILE((!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)),statOk,5e3);

	I2C_SendData(IMU_I2C,reg); //Set MSB for reg. increment
	TIMEDWHILE((!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTING)),statOk,50e3);

	while(len--){
		I2C_SendData(IMU_I2C,*buf++);
		TIMEDWHILE(!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTING),statOk,5e3);
	}
	TIMEDWHILE(!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED),statOk,5e3);
	I2C_GenerateSTOP(IMU_I2C,ENABLE);
	return statOk!=1;
}

uint8_t I2C_read(uint8_t addr,uint8_t reg,uint8_t *buf,uint8_t len){
	uint8_t statOk=!0;

	I2C_AcknowledgeConfig(IMU_I2C, ENABLE);
	I2C_GenerateSTART(IMU_I2C,ENABLE);
	TIMEDWHILE((!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_MODE_SELECT)),statOk,1e3);
	I2C_Send7bitAddress(IMU_I2C,addr,I2C_Direction_Transmitter);
	TIMEDWHILE((!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)),statOk,1e3);

	I2C_SendData(IMU_I2C,reg); //OUT_X_L_G + MSB
	TIMEDWHILE(!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED),statOk,1e3);

	I2C_GenerateSTART(IMU_I2C,ENABLE);
	TIMEDWHILE((!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_MODE_SELECT)),statOk,1e3);

	I2C_Send7bitAddress(IMU_I2C,addr,I2C_Direction_Receiver);
	TIMEDWHILE((!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)),statOk,50e3);
	len--;
	while(len--){
		TIMEDWHILE((!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_BYTE_RECEIVED)),statOk,1e3);
		*buf++=I2C_ReceiveData(IMU_I2C);
	}
	I2C_AcknowledgeConfig(IMU_I2C, DISABLE);
	I2C_GenerateSTOP(IMU_I2C, ENABLE);
	TIMEDWHILE((!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_BYTE_RECEIVED)),statOk,1e3);
	*buf++=I2C_ReceiveData(IMU_I2C);
	return statOk!=1;
}
