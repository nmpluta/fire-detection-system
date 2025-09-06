/*
 * Copyright (c) 2025 Natalia Pluta
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SENSOR_MODULE_H
#define SENSOR_MODULE_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/zbus/zbus.h>

/* Forward declaration of listener defined in main.c */
extern const struct zbus_observer sensor_response_listener;

/**
 * @brief Sensor message types
 */
enum sensor_msg_type {
	SENSOR_SAMPLE_REQUEST = 0x1,
	SENSOR_SAMPLE_RESPONSE,
};

/**
 * @brief Sensor types enumeration
 */
enum sensor_type {
	SENSOR_TYPE_BME280 = 0,
	SENSOR_TYPE_CCS811,
	SENSOR_TYPE_HM3301,
	SENSOR_TYPE_COUNT /* Keep this last - represents total number of sensors */
};

/**
 * @brief Sensor health monitoring structure
 */
struct sensor_health {
	uint32_t success_count;
	uint32_t failure_count;
	int64_t last_success_time;
	bool is_healthy;
};

/**
 * @brief Sensor information structure
 */
struct sensor_info {
	const struct device *device;
	struct sensor_health health;
	bool enabled;
};

/**
 * @brief Unified sensor message structure
 */
struct sensor_msg {
	enum sensor_msg_type type;

	/* Sensor readings - only valid for SENSOR_SAMPLE_RESPONSE */
	struct sensor_value temperature;
	struct sensor_value humidity;
	struct sensor_value pressure;
	struct sensor_value co2;
	struct sensor_value voc;
	struct sensor_value pm1_0;
	struct sensor_value pm2_5;
	struct sensor_value pm10;

	/* Timestamp - only valid for SENSOR_SAMPLE_RESPONSE */
	int64_t timestamp;
};

/* ZBUS channel declaration */
ZBUS_CHAN_DECLARE(sensor_chan);

/* Helper macro for casting ZBUS message to sensor_msg */
#define MSG_TO_SENSOR_MSG(_msg) (*(const struct sensor_msg *)_msg)

/**
 * @brief Initialize the sensor module
 *
 * This function initializes all sensors and starts the sensor thread.
 *
 * @return 0 on success, negative error code on failure
 */
int sensor_module_init(void);

/**
 * @brief Request sensor data
 *
 * This function sends a SENSOR_SAMPLE_REQUEST message via ZBUS channel.
 * The sensor module will respond with SENSOR_SAMPLE_RESPONSE containing sensor data.
 *
 * @return 0 on success, negative error code on failure
 */
int sensor_module_request_data(void);

/**
 * @brief Get sensor information for a specific sensor
 *
 * @param sensor_type The type of sensor to get info for
 * @param sensor_info Pointer to store sensor information
 * @return 0 on success, negative error code on failure
 */
int sensor_module_get_sensor_info(enum sensor_type sensor_type, struct sensor_info *sensor_info);

/**
 * @brief Get sensor health information for all sensors
 *
 * @param sensor_healths Array to store health info (must be SENSOR_TYPE_COUNT size)
 * @return 0 on success, negative error code on failure
 */
int sensor_module_get_all_health(struct sensor_health sensor_healths[SENSOR_TYPE_COUNT]);

/**
 * @brief Get sensor health information
 *
 * This function returns health statistics for all sensors.
 *
 * @param bme280_health Pointer to store BME280 health info
 * @param ccs811_health Pointer to store CCS811 health info
 * @param hm3301_health Pointer to store HM3301 health info
 * @return 0 on success, negative error code on failure
 */
int sensor_module_get_health(struct sensor_health *bme280_health,
			     struct sensor_health *ccs811_health,
			     struct sensor_health *hm3301_health);

#endif /* SENSOR_MODULE_H */
