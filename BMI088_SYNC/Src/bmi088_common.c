/**
 * Copyright (C) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/******************************************************************************/
#include <string.h>

#include "main.h"
#include "bmi08x.h"
#include "bmi08x_defs.h"
#include "bmi088_common.h"

/******************************************************************************/
/*!                       Macro definitions                                   */

#define BMI08X_READ_WRITE_LEN  UINT8_C(32)

/*! BMI085 shuttle id */
#define BMI085_SHUTTLE_ID      UINT16_C(0x46)

/*! BMI088 shuttle id */
#define BMI088_SHUTTLE_ID      UINT16_C(0x66)

#define BUS_TIMEOUT 1000

#define ACC_SPI_CS_ID 1
#define GYRO_SPI_CS_ID 2

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection for accel */
uint8_t acc_dev_add;

/*! Variable that holds the I2C device address or SPI chip selection for gyro */
uint8_t gyro_dev_add;

/* Data Sync configuration object */
struct bmi08x_data_sync_cfg sync_cfg;

/*! bmi08x int config */
struct bmi08x_int_cfg int_config;

/* Temporary write and read buffer */
uint8_t GTXBuffer[512];
uint8_t GRXBuffer[2048];

extern SPI_HandleTypeDef hspi2;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *  Also to initialize coines platform
 */
BMI08X_INTF_RET_TYPE bmi08x_interface_init(struct bmi08x_dev *bmi08x, uint8_t intf, uint8_t variant)
{
	int8_t rslt = BMI08X_OK;

	if (bmi08x != NULL)
	{
		/* Bus configuration : I2C */
		if (intf == BMI08X_I2C_INTF)
		{
			/* To initialize the user I2C function */
			acc_dev_add = BMI08X_ACCEL_I2C_ADDR_PRIMARY;
			gyro_dev_add = BMI08X_GYRO_I2C_ADDR_PRIMARY;
			bmi08x->intf = BMI08X_I2C_INTF;
			bmi08x->read = (bmi08x_read_fptr_t)bmi08x_i2c_read;
			bmi08x->write = (bmi08x_write_fptr_t)bmi08x_i2c_write;
		}
		/* Bus configuration : SPI */
		else if (intf == BMI08X_SPI_INTF)
		{
			/* To initialize the user SPI function */
			bmi08x->intf = BMI08X_SPI_INTF;
			bmi08x->read = bmi08x_spi_read;
			bmi08x->write = bmi08x_spi_write;

			/* SPI chip select pin for Accel (CSB1_A) */
			acc_dev_add = ACC_SPI_CS_ID;

			/* SPI chip select pin for Gyro (CSB2_G) */
			gyro_dev_add = GYRO_SPI_CS_ID;
		}

		/* Selection of bmi085 or bmi088 sensor variant */
		bmi08x->variant = variant;

		/* Assign accel device address to accel interface pointer */
		bmi08x->intf_ptr_accel = &acc_dev_add;

		/* Assign gyro device address to gyro interface pointer */
		bmi08x->intf_ptr_gyro = &gyro_dev_add;

		/* Configure delay in microseconds */
		bmi08x->delay_us = bmi08x_delay_us;

		/* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
		bmi08x->read_write_len = BMI08X_READ_WRITE_LEN;
	}
	else
	{
		rslt = BMI08X_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief    This internal API is used to initialize the bmi08x sensor
 */
BMI08X_INTF_RET_TYPE init_bmi08x(struct bmi08x_dev *bmi08x)
{
	int8_t rslt = BMI08X_OK;

	/* Initialize bmi08a */
	rslt = bmi08a_init(bmi08x);

	/* Initialize bmi08g */
	rslt = bmi08g_init(bmi08x);

	if (rslt == BMI08X_OK)
	{
		/* Reset the accelerometer */
		rslt = bmi08a_soft_reset(bmi08x);

		/* Max read/write length (maximum supported length is 32).
         To be set by the user */
		bmi08x->read_write_len = 32;

		/* Set accel power mode */
		bmi08x->accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
		rslt = bmi08a_set_power_mode(bmi08x);

		if (rslt == BMI08X_OK)
		{
			bmi08x->gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
			rslt = bmi08g_set_power_mode(bmi08x);
		}

		if ((bmi08x->accel_cfg.power == BMI08X_ACCEL_PM_ACTIVE) && (bmi08x->gyro_cfg.power == BMI08X_GYRO_PM_NORMAL))
		{
			/* API uploads the bmi08x config file onto the device */
			if (rslt == BMI08X_OK)
			{
				rslt = bmi08a_load_config_file(bmi08x);

				/* Wait for 150ms to enable the data synchronization --delay taken care inside the function */
				if (rslt == BMI08X_OK)
				{
					/* Assign accel range setting */
					if (bmi08x->variant == BMI085_VARIANT)
					{
						bmi08x->accel_cfg.range = BMI085_ACCEL_RANGE_16G;
					}
					else if (bmi08x->variant == BMI088_VARIANT)
					{
						bmi08x->accel_cfg.range = BMI088_ACCEL_RANGE_24G;
					}

					/* Assign gyro range setting */
					bmi08x->gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;

					/* Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
					sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_1000HZ;

					rslt = bmi08a_configure_data_synchronization(sync_cfg, bmi08x);
				}
			}
		}
	}

	return rslt;
}

/*!
 *  @brief This API is used to enable bmi08x synchronization interrupt
 */
BMI08X_INTF_RET_TYPE enable_bmi08x_data_synchronization_interrupt(struct bmi08x_dev *bmi08x)
{
	int8_t rslt = BMI08X_OK;

	/* Set accel interrupt pin configuration */
	int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
    int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
    int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

	/* Configure accel syncronization input interrupt pin */
	int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
    int_config.accel_int_config_2.int_type = BMI08X_ACCEL_INT_SYNC_DATA_RDY;
    int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

	/* Set gyro interrupt pin configuration */
	int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
    int_config.gyro_int_config_1.int_type = BMI08X_GYRO_INT_DATA_RDY;
    int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
    int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

	/* Configure gyro syncronization input interrupt pin */
	int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
    int_config.gyro_int_config_2.int_type = BMI08X_GYRO_INT_DATA_RDY;
    int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
    int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

	/* Enable synchronization interrupt pin */
	rslt = bmi08a_set_data_sync_int_config(&int_config, bmi08x);

	return rslt;
}

/*!
 *  @brief This API is used to disable bmi08x synchronization interrupt
 */
BMI08X_INTF_RET_TYPE disable_bmi08x_data_synchronization_interrupt(struct bmi08x_dev *bmi08x)
{
	int8_t rslt = BMI08X_OK;

	/*turn off the sync feature*/
	sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_OFF;

	rslt = bmi08a_configure_data_synchronization(sync_cfg, bmi08x);

	/* Wait for 150ms to enable the data synchronization --delay taken care inside the function */
	/* configure synchronization interrupt pins */
	if (rslt == BMI08X_OK)
	{
		/* Set accel interrupt pin configuration */
		/* Configure host data ready interrupt */
		int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
        int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
        int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

		/* Configure Accel synchronization input interrupt pin */
		int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
        int_config.accel_int_config_2.int_type = BMI08X_ACCEL_INT_SYNC_DATA_RDY;
        int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

		/* Set gyro interrupt pin configuration */
		int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
        int_config.gyro_int_config_1.int_type = BMI08X_GYRO_INT_DATA_RDY;
        int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

		/* Configure gyro synchronization input interrupt pin */
		int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
        int_config.gyro_int_config_2.int_type = BMI08X_GYRO_INT_DATA_RDY;
        int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
        int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

		/* Disable synchronization interrupt pin */
		rslt = bmi08a_set_data_sync_int_config(&int_config, bmi08x);
	}

	return rslt;
}

/*!
 * I2C read function map to COINES platform
 */
BMI08X_INTF_RET_TYPE bmi08x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{

	return 0;
}

/*!
 * I2C write function map to COINES platform
 */
BMI08X_INTF_RET_TYPE bmi08x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{

	return 0;
}

/*!
 * SPI read function
 */
BMI08X_INTF_RET_TYPE bmi08x_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	uint8_t cs_id = *(uint8_t*)intf_ptr;
	GTXBuffer[0] = reg_addr | 0x80;

	if(cs_id == ACC_SPI_CS_ID)
	{
		HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_RESET); // NSS low
		HAL_SPI_TransmitReceive(&hspi2, GTXBuffer, GRXBuffer, len+1, BUS_TIMEOUT); // timeout 1000msec;
		while(hspi2.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
		memcpy(reg_data, GRXBuffer+1, len);
		HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_SET); // NSS high
	}
	else	if(cs_id == GYRO_SPI_CS_ID)
	{
		HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET); // NSS low
		HAL_SPI_TransmitReceive(&hspi2, GTXBuffer, GRXBuffer, len+1, BUS_TIMEOUT); // timeout 1000msec;
		while(hspi2.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
		memcpy(reg_data, GRXBuffer+1, len);
		HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET); // NSS high
	}

	return 0;
}

/*!
 * SPI write function
 */
BMI08X_INTF_RET_TYPE bmi08x_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	uint8_t cs_id = *(uint8_t*)intf_ptr;

	GTXBuffer[0] = reg_addr & 0x7F;
	memcpy(&GTXBuffer[1], reg_data, len);

	if(cs_id == ACC_SPI_CS_ID)
	{
		HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_RESET); // NSS low
	}
	else	if(cs_id == GYRO_SPI_CS_ID)
	{
		HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET); // NSS low
	}

	HAL_SPI_Transmit(&hspi2, GTXBuffer, len+1, BUS_TIMEOUT); // send register address + write data
	while(hspi2.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

	if(cs_id == ACC_SPI_CS_ID)
	{
		HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_SET); // NSS high
	}
	else	if(cs_id == GYRO_SPI_CS_ID)
	{
		HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET); // NSS high
	}

	return 0;
}

/*!
 * Delay function
 */
void bmi08x_delay_us(uint32_t period, void *intf_ptr)
{
	HAL_Delay(period/1000);
}
