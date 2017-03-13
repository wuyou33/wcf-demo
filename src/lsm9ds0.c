/*
 * lsm9ds0.c
 *
 *  Created on: Feb 19, 2016
 *      Author: mobintu
 */

#include "lsm9ds0.h"

lsm9ds0_ui lsm9ds0;

typedef enum {GYR_245DPS,GYR_500DPS,GYR_2000DPS} GYR_SEL;
typedef enum {ACC_2G,ACC_4G,ACC_6G,ACC_8G,ACC_16G} ACC_SEL;
typedef enum {MAG_2G,MAG_4G,MAG_8G,MAG_12G} MAG_SEL;


static const uint32_t GYR_VAL[] = {LSM9DS0_GYRO_FS_245DPS_VAL,LSM9DS0_GYRO_FS_500DPS_VAL,LSM9DS0_GYRO_FS_2000DPS_VAL};
static const uint32_t ACC_VAL[] = {LSM9DS0_ACCEL_FS_2G_VAL,LSM9DS0_ACCEL_FS_4G_VAL,LSM9DS0_ACCEL_FS_6G_VAL,LSM9DS0_ACCEL_FS_8G_VAL,LSM9DS0_ACCEL_FS_16G_VAL};
static const uint32_t MAG_VAL[] = {LSM9DS0_MAGN_FS_2GAUSS_VAL,LSM9DS0_MAGN_FS_4GAUSS_VAL,LSM9DS0_MAGN_FS_8GAUSS_VAL,LSM9DS0_MAGN_FS_12GAUSS_VAL};


static const uint32_t GYR_GAIN[] = {LSM9DS0_GYRO_FS_245DPS_GAIN,LSM9DS0_GYRO_FS_500DPS_GAIN,LSM9DS0_GYRO_FS_2000DPS_GAIN};
static const uint32_t ACC_GAIN[] = {LSM9DS0_ACCEL_FS_2G_GAIN,LSM9DS0_ACCEL_FS_4G_GAIN,LSM9DS0_ACCEL_FS_6G_GAIN,LSM9DS0_ACCEL_FS_8G_GAIN,LSM9DS0_ACCEL_FS_16G_GAIN};
static const uint32_t MAG_GAIN[] = {LSM9DS0_MAGN_FS_2GAUSS_GAIN,LSM9DS0_MAGN_FS_4GAUSS_GAIN,LSM9DS0_MAGN_FS_8GAUSS_GAIN,LSM9DS0_MAGN_FS_12GAUSS_GAIN};



static GYR_SEL gyr_sel = GYR_500DPS;
static ACC_SEL acc_sel = ACC_4G;
static MAG_SEL mag_sel = MAG_2G;

static real gyr_sc;
static real acc_sc;
static real mag_sc;




void lsm9ds0_Init(void){

	uint8_t buff[9],ret;

	gyr_sc = (GYR_GAIN[gyr_sel]*M_PI/180.0/1000000.0);
	acc_sc =  ACC_GAIN[acc_sel]/1000000.0;
	mag_sc =  MAG_GAIN[mag_sel]/1000000.0;


	//CTRL_REG1_G (20h)
	buff[0] =  LSM9DS0_GYRO_ODR_380HZ_VAL | LSM9DS0_GYRO_BW(3) | LSM9DS0_GYRO_NORMAL_MODE| LSM9DS0_GYRO_X_EN | LSM9DS0_GYRO_Y_EN | LSM9DS0_GYRO_Z_EN;
	buff[1] = 0;
	buff[2] = 0;
	buff[3] = (1<<7) | GYR_VAL[gyr_sel];
	buff[4] = 0;

	ret = lsm9ds0.i2c_wr(LSM9DS0_GY_addr,0x20|0x80,buff,5);
	lsm9ds0.gyr_stat = ret;


	//CTRL_REG 0x1F
	buff[0] = 0;
	buff[1] = LSM9DS0_ACCEL_ODR_400HZ_VAL | (1<<3) | LSM9DS0_ACCEL_X_EN | LSM9DS0_ACCEL_Y_EN | LSM9DS0_ACCEL_Z_EN;
	buff[2] = LSM9DS0_ACC_BW(1) | ACC_VAL[acc_sel];
	buff[3] = 0;
	buff[4] = 0;
	buff[5] = LSM9DS0_TEMP_EN | LSM9DS0_MAGN_HIGH_RES_VAL | LSM9DS0_MAGN_ODR_100HZ_VAL;
	buff[6] = MAG_VAL[mag_sel];
	buff[7] = 0;

	if(ret == 0 )
		ret = lsm9ds0.i2c_wr(LSM9DS0_AM_addr,0x1F|0x80,buff,8);
	lsm9ds0.acc_stat = ret;
	lsm9ds0.mag_stat = ret;

	lsm9ds0.gyr_age = 65000;
	lsm9ds0.acc_age = 65000;
	lsm9ds0.mag_age = 65000;
}

void lsm9ds0_read(void){
	uint8_t *ptr;
	int16_t buff[3];
	ptr = (int8_t *)buff;
	uint8_t tmp;

	lsm9ds0.gyr_age++;
	lsm9ds0.acc_age++;
	lsm9ds0.mag_age++;


	lsm9ds0.i2c_rd(LSM9DS0_GY_addr,0x27,ptr,1);
	if ( (ptr[0]&0x0F) == 0x0F){
		lsm9ds0.gyr_stat = lsm9ds0.i2c_rd(LSM9DS0_GY_addr,0x28|0x80,ptr,6);
		if (lsm9ds0.gyr_stat == 0){
			lsm9ds0.gyr_age = 0;
			lsm9ds0.gyr[0] = gyr_sc*buff[0];
			lsm9ds0.gyr[1] = gyr_sc*buff[1];
			lsm9ds0.gyr[2] = gyr_sc*buff[2];
		}
	}
	else{
		lsm9ds0.gyr[0] = 0;
		lsm9ds0.gyr[1] = 0;
		lsm9ds0.gyr[2] = 0;
	}

	lsm9ds0.i2c_rd(LSM9DS0_AM_addr,0x27,ptr,1);
	if ( (ptr[0]&0x0F) == 0x0F){
		lsm9ds0.acc_stat = lsm9ds0.i2c_rd(LSM9DS0_AM_addr,0x28|0x80,ptr,6);
		if (lsm9ds0.acc_stat==0){
			lsm9ds0.acc_age = 0;
			lsm9ds0.acc[0] = acc_sc*buff[0];
			lsm9ds0.acc[1] = acc_sc*buff[1];
			lsm9ds0.acc[2] = acc_sc*buff[2];

		}
	}

	lsm9ds0.i2c_rd(LSM9DS0_AM_addr,0x07,ptr,1);
	if ( (ptr[0]&0x0F) == 0x0F){
		lsm9ds0.i2c_rd(LSM9DS0_AM_addr,0x05|0x80,ptr,2);
		lsm9ds0.temp = buff[0]*0.0625 + 25.0;

		lsm9ds0.mag_stat = lsm9ds0.i2c_rd(LSM9DS0_AM_addr,0x08|0x80,ptr,6);
		if (lsm9ds0.mag_stat == 0){
			lsm9ds0.mag_age = 0;
			lsm9ds0.mag[0] = mag_sc*buff[0];
			lsm9ds0.mag[1] = mag_sc*buff[1];
			lsm9ds0.mag[2] = -mag_sc*buff[2]; //Check axis
		}
	}
}


