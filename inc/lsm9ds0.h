/*
 * lsm9ds0.h
 *
 *  Created on: Feb 19, 2016
 *      Author: mobintu
 */

#ifndef SRC_LSM9DS0_H_
#define SRC_LSM9DS0_H_

#define LSM9DS0_GY_addr 0xD7
#define LSM9DS0_AM_addr 0x3B

#include "math_utils.h"



typedef struct {
	real gyr[3];
	real acc[3];
	real mag[3];
	real temp;
	volatile uint32_t gyr_stat;
	volatile uint32_t acc_stat;
	volatile uint32_t mag_stat;
	uint8_t (*i2c_wr)(uint8_t addr, uint8_t reg, uint8_t *buff, uint8_t len);
	uint8_t (*i2c_rd)(uint8_t addr, uint8_t reg, uint8_t *buff, uint8_t len);
	void (*delay_us)(uint32_t del);
	volatile uint16_t gyr_age;
	volatile uint16_t acc_age;
	volatile uint16_t mag_age;
}lsm9ds0_ui;

extern lsm9ds0_ui lsm9ds0;

void lsm9ds0_Init(void);
void lsm9ds0_read(void);




#define LSM9DS0_WHO_AM_I_REG            (0x0F)
#define LSM9DS0_CTRL_REG1_G_REG         (0x20)
#define LSM9DS0_CTRL_REG2_G_REG         (0x21)
#define LSM9DS0_CTRL_REG3_G_REG         (0x22)
#define LSM9DS0_CTRL_REG4_G_REG         (0x23)
#define LSM9DS0_CTRL_REG5_G_REG         (0x24)
#define LSM9DS0_REFERENCE_G_REG         (0x25)
#define LSM9DS0_STATUS_REG_G_REG        (0x27)
#define LSM9DS0_OUT_X_L_G_REG           (0x28)
#define LSM9DS0_OUT_X_H_G_REG           (0x29)
#define LSM9DS0_OUT_Y_L_G_REG           (0x2A)
#define LSM9DS0_OUT_Y_H_G_REG           (0x2B)
#define LSM9DS0_OUT_Z_L_G_REG           (0x2C)
#define LSM9DS0_OUT_Z_H_G_REG           (0x2D)
#define LSM9DS0_FIFO_CTRL_REG_G_REG     (0x2E)
#define LSM9DS0_FIFO_SRC_REG_G_REG      (0x2F)
#define LSM9DS0_INT1_CFG_G_REG          (0x30)
#define LSM9DS0_INT1_SRC_G_REG          (0x31)
#define LSM9DS0_INT1_TSH_XH_G_REG       (0x32)
#define LSM9DS0_INT1_TSH_XL_G_REG       (0x33)
#define LSM9DS0_INT1_TSH_YH_G_REG       (0x34)
#define LSM9DS0_INT1_TSH_YL_G_REG       (0x35)
#define LSM9DS0_INT1_TSH_ZH_G_REG       (0x36)
#define LSM9DS0_INT1_TSH_ZL_G_REG       (0x37)
#define LSM9DS0_INT1_DURATION_G_REG     (0x38)
#define LSM9DS0_OUT_TEMP_L_XM_REG       (0x05)
#define LSM9DS0_OUT_TEMP_H_XM_REG       (0x06)
#define LSM9DS0_STATUS_REG_M_REG        (0x07)
#define LSM9DS0_OUT_X_L_M_REG           (0x08)
#define LSM9DS0_OUT_X_H_M_REG           (0x09)
#define LSM9DS0_OUT_Y_L_M_REG           (0x0A)
#define LSM9DS0_OUT_Y_H_M_REG           (0x0B)
#define LSM9DS0_OUT_Z_L_M_REG           (0x0C)
#define LSM9DS0_OUT_Z_H_M_REG           (0x0D)
#define LSM9DS0_INT_CTRL_REG_M_REG      (0x12)
#define LSM9DS0_INT_SRC_REG_M_REG       (0x13)
#define LSM9DS0_INT_THS_L_M_REG         (0x14)
#define LSM9DS0_INT_THS_H_M_REG         (0x15)
#define LSM9DS0_OFFSET_X_L_M_REG        (0x16)
#define LSM9DS0_OFFSET_X_H_M_REG        (0x17)
#define LSM9DS0_OFFSET_Y_L_M_REG        (0x18)
#define LSM9DS0_OFFSET_Y_H_M_REG        (0x19)
#define LSM9DS0_OFFSET_Z_L_M_REG        (0x1A)
#define LSM9DS0_OFFSET_Z_H_M_REG        (0x1B)
#define LSM9DS0_REFERENCE_X_REG         (0x1C)
#define LSM9DS0_REFERENCE_Y_REG         (0x1D)
#define LSM9DS0_REFERENCE_Z_REG         (0x1E)
#define LSM9DS0_CTRL_REG0_XM_REG        (0x1F)
#define LSM9DS0_CTRL_REG1_XM_REG        (0x20)
#define LSM9DS0_CTRL_REG2_XM_REG        (0x21)
#define LSM9DS0_CTRL_REG3_XM_REG        (0x22)
#define LSM9DS0_CTRL_REG4_XM_REG        (0x23)
#define LSM9DS0_CTRL_REG5_XM_REG        (0x24)
#define LSM9DS0_CTRL_REG6_XM_REG        (0x25)
#define LSM9DS0_CTRL_REG7_XM_REG        (0x26)
#define LSM9DS0_STATUS_REG_A_REG        (0x27)
#define LSM9DS0_OUT_X_L_A_REG           (0x28)
#define LSM9DS0_OUT_X_H_A_REG           (0x29)
#define LSM9DS0_OUT_Y_L_A_REG           (0x2A)
#define LSM9DS0_OUT_Y_H_A_REG           (0x2B)
#define LSM9DS0_OUT_Z_L_A_REG           (0x2C)
#define LSM9DS0_OUT_Z_H_A_REG           (0x2D)
#define LSM9DS0_FIFO_CTRL_REG_REG       (0x2E)
#define LSM9DS0_FIFO_SRC_REG_REG        (0x2F)
#define LSM9DS0_INT_GEN_1_REG_REG       (0x30)
#define LSM9DS0_INT_GEN_1_SRC_REG       (0x31)
#define LSM9DS0_INT_GEN_1_THS_REG       (0x32)
#define LSM9DS0_INT_GEN_1_DURATION_REG  (0x33)
#define LSM9DS0_INT_GEN_2_REG_REG       (0x34)
#define LSM9DS0_INT_GEN_2_SRC_REG       (0x35)
#define LSM9DS0_INT_GEN_2_THS_REG       (0x36)
#define LSM9DS0_INT_GEN_2_DURATION_REG  (0x37)
#define LSM9DS0_CLICK_CFG_REG           (0x38)
#define LSM9DS0_CLICK_SRC_REG           (0x39)
#define LSM9DS0_CLICK_THS_REG           (0x3A)
#define LSM9DS0_TIME_LIMIT_REG          (0x3B)
#define LSM9DS0_TIME_LATENCY_REG        (0x3C)
#define LSM9DS0_TIME_WINDOW_REG         (0x3D)
#define LSM9DS0_ACT_THS_REG             (0x3E)
#define LSM9DS0_ACT_DUR_REG             (0x3F)

#define LSM9DS0_GYRO_ODR_95HZ_VAL       (0x00 << 6)
#define LSM9DS0_GYRO_ODR_190HZ_VAL      (0x01 << 6)
#define LSM9DS0_GYRO_ODR_380HZ_VAL      (0x02 << 6)
#define LSM9DS0_GYRO_ODR_760HZ_VAL      (0x03 << 6)

#define LSM9DS0_ACCEL_POWER_DOWN        (0x00 << 4)
#define LSM9DS0_ACCEL_ODR_3_125HZ_VAL   (0x01 << 4)
#define LSM9DS0_ACCEL_ODR_6_25HZ_VAL    (0x02 << 4)
#define LSM9DS0_ACCEL_ODR_12_5HZ_VAL    (0x03 << 4)
#define LSM9DS0_ACCEL_ODR_25HZ_VAL      (0x04 << 4)
#define LSM9DS0_ACCEL_ODR_50HZ_VAL      (0x05 << 4)
#define LSM9DS0_ACCEL_ODR_100HZ_VAL     (0x06 << 4)
#define LSM9DS0_ACCEL_ODR_200HZ_VAL     (0x07 << 4)
#define LSM9DS0_ACCEL_ODR_400HZ_VAL     (0x08 << 4)
#define LSM9DS0_ACCEL_ODR_800HZ_VAL     (0x09 << 4)
#define LSM9DS0_ACCEL_ODR_1600HZ_VAL    (0x0A << 4)

#define LSM9DS0_ACCEL_FS_MASK           (0x03 << 3)
#define LSM9DS0_ACCEL_FS_2G_VAL         (0x00 << 3)
#define LSM9DS0_ACCEL_FS_4G_VAL         (0x01 << 3)
#define LSM9DS0_ACCEL_FS_6G_VAL         (0x02 << 3)
#define LSM9DS0_ACCEL_FS_8G_VAL         (0x03 << 3)
#define LSM9DS0_ACCEL_FS_16G_VAL        (0x04 << 3)
#define LSM9DS0_ACCEL_FS_2G_GAIN        61     /* ug/LSB  */
#define LSM9DS0_ACCEL_FS_4G_GAIN        122    /* ug/LSB  */
#define LSM9DS0_ACCEL_FS_6G_GAIN        183    /* ug/LSB  */
#define LSM9DS0_ACCEL_FS_8G_GAIN        244    /* ug/LSB  */
#define LSM9DS0_ACCEL_FS_16G_GAIN       732    /* ug/LSB  */

#define LSM9DS0_MAGN_ODR_3_125HZ_VAL    (0x00 << 2)
#define LSM9DS0_MAGN_ODR_6_25HZ_VAL     (0x01 << 2)
#define LSM9DS0_MAGN_ODR_12_5HZ_VAL     (0x02 << 2)
#define LSM9DS0_MAGN_ODR_25HZ_VAL       (0x03 << 2)
#define LSM9DS0_MAGN_ODR_50HZ_VAL       (0x04 << 2)
#define LSM9DS0_MAGN_ODR_100HZ_VAL      (0x05 << 2)

#define LSM9DS0_MAGN_FS_MASK            (0x03 << 5)
#define LSM9DS0_MAGN_FS_2GAUSS_VAL      (0x00 << 5)
#define LSM9DS0_MAGN_FS_4GAUSS_VAL      (0x01 << 5)
#define LSM9DS0_MAGN_FS_8GAUSS_VAL      (0x02 << 5)
#define LSM9DS0_MAGN_FS_12GAUSS_VAL     (0x03 << 5)
#define LSM9DS0_MAGN_FS_2GAUSS_GAIN     80     /* ugauss/LSB  */
#define LSM9DS0_MAGN_FS_4GAUSS_GAIN     160    /* ugauss/LSB  */
#define LSM9DS0_MAGN_FS_8GAUSS_GAIN     320    /* ugauss/LSB  */
#define LSM9DS0_MAGN_FS_12GAUSS_GAIN    480    /* ugauss/LSB  */

#define LSM9DS0_GYRO_FS_MASK            (0x03 << 4)
#define LSM9DS0_GYRO_FS_245DPS_VAL      (0x00 << 4)
#define LSM9DS0_GYRO_FS_500DPS_VAL      (0x01 << 4)
#define LSM9DS0_GYRO_FS_2000DPS_VAL     (0x02 << 4)
#define LSM9DS0_GYRO_FS_245DPS_GAIN     8750   /* udps/LSB */
#define LSM9DS0_GYRO_FS_500DPS_GAIN     17500  /* udps/LSB */
#define LSM9DS0_GYRO_FS_2000DPS_GAIN    70000  /* udps/LSB */

#define LSM9DS0_GYRO_X_EN               BIT(1)
#define LSM9DS0_GYRO_Y_EN               BIT(0)
#define LSM9DS0_GYRO_Z_EN               BIT(2)
#define LSM9DS0_GYRO_POWER_DOWN         (0x00 << 3)
#define LSM9DS0_GYRO_NORMAL_MODE        BIT(3)
#define LSM9DS0_ACCEL_X_EN              BIT(0)
#define LSM9DS0_ACCEL_Y_EN              BIT(1)
#define LSM9DS0_ACCEL_Z_EN              BIT(2)
#define LSM9DS0_TEMP_EN                 BIT(7)
#define LSM9DS0_MAGN_LOW_RES_VAL        (0x00 << 5)
#define LSM9DS0_MAGN_HIGH_RES_VAL       (0x03 << 5)
#define LSM9DS0_MAGN_POWER_DOWN         (0x02)
#define LSM9DS0_MAGN_CONT_CONV_MODE     (0x00)
#define LSM9DS0_MAGN_SINGLE_CONV_MODE   (0x01)

#define LSM9DS0_GYRO_ID                  0xD4
#define LSM9DS0_ACCEL_MAGN_ID            0x49

#define LSM9DS0_GYRO_BW(x)				((x)<<4)
#define LSM9DS0_ACC_BW(x)				((x)<<6)


#endif /* SRC_LSM9DS0_H_ */
