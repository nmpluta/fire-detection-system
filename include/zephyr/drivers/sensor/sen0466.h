/*
 * Copyright (c) 2025 Natalia Pluta
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

#if defined(CONFIG_SEN0466_TEMP_COMP_EXTERNAL)
/**
 * @brief Update external temperature for CO measurement compensation
 *
 * This function provides external temperature data to improve CO concentration measurements
 * accuracy. The SEN0466 uses this for temperature compensation algorithms to adjust CO readings
 * based on the temperature.
 *
 * @note Call this function before performing CO gas measurements to ensure the sensor's
 *       temperature compensation is up to date.
 *
 * @note Internal temperature readings from the SEN0466 sensor can be enabled for compensation
 * instead of using external temperature readings via SEN0466_CO_COMP_INTEGRATED configuration.
 *
 * @param dev Pointer to the sensor device
 *
 * @param temperature Current temperature at the sensor
 *
 * @return 0 if successful, negative errno code if failure.
 */
int sen0466_temp_update(const struct device *dev, const struct sensor_value *temperature);
#endif /* CONFIG_SEN0466_TEMP_COMP_EXTERNAL */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_SEN0466_H_ */
