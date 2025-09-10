/*
 * Copyright (c) 2025 Natalia Pluta
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SEN0466_SEN0466_H_
#define ZEPHYR_DRIVERS_SENSOR_SEN0466_SEN0466_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

/* SEN0466 commands */
#define SEN0466_CMD_CHANGE_GET_METHOD     0X78
#define SEN0466_CMD_GET_GAS_CONCENTRATION 0X86
#define SEN0466_CMD_GET_TEMP              0X87
#define SEN0466_CMD_GET_ALL_DATA          0X88
#define SEN0466_CMD_SET_THRESHOLD_ALARMS  0X89
#define SEN0466_CMD_IIC_AVAILABLE         0X90
#define SEN0466_CMD_SENSOR_VOLTAGE        0X91
#define SEN0466_CMD_CHANGE_IIC_ADDR       0X92

/* SEN0466 mode values */
#define SEN0466_MODE_INITIATIVE 0x03 /* Sensor continuously reports data */
#define SEN0466_MODE_PASSIVE    0x04 /* Sensor reports data only on request */

/* SEN0466 frame constants */
#define SEN0466_FRAME_HEAD  0xFF
#define SEN0466_FRAME_ADDR  0x01
#define SEN0466_FRAME_LEN   9 /* total rx bytes */
#define SEN0466_PAYLOAD_LEN 6 /* data[0..5] inside protocol on tx */

/* Device data */
struct sen0466_data {
	uint16_t co_ppm; /* CO concentration in ppm */
#if defined(CONFIG_SEN0466_SAMPLE_ALL)
	double temp_celsius; /* Temperature in degrees Celsius */
#endif
#if defined(CONFIG_SEN0466_TEMP_COMP_EXTERNAL)
	double ext_temp_celsius; /* External temperature for compensation */
	bool ext_temp_valid;     /* Flag indicating if external temp is valid */
#endif
};

/* Device configuration */
struct sen0466_config {
	struct i2c_dt_spec i2c;
};

/**
 * @brief Initialize SEN0466 sensor
 *
 * @param dev Pointer to the device structure
 * @return 0 on success, negative error code on failure
 */
int sen0466_init(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_SENSOR_SEN0466_SEN0466_H_ */
