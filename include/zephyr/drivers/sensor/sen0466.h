/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Extended sensor channels for SEN0466 CO sensor
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_SEN0466_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_SEN0466_H_

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SEN0466 specific sensor channels.
 */
enum sensor_channel_sen0466 {
	/**
	 * @brief Carbon Monoxide sensor channel
	 *
	 * This channel represents CO concentration measurements.
	 * Unit: ppm (parts per million)
	 *
	 * val1: Integer part of CO concentration
	 * val2: Not used (0)
	 */
	SENSOR_CHAN_SEN0466_CO = SENSOR_CHAN_PRIV_START,

	/**
	 * @brief Temperature channel for SEN0466 sensor
	 *
	 * This represents the temperature reading from the SEN0466 sensor.
	 * Unit: degrees Celsius
	 *
	 * val1: Integer temperature value
	 * val2: Fractional part in microdegrees (1e-6)
	 */
	SENSOR_CHAN_SEN0466_TEMP,
};

// TODO: Implement SEN0466 specific sensor attributes.

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_SEN0466_H_ */
