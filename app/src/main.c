/*
 * Copyright (c) 2025 Natalia Pluta
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#include <app/drivers/blink.h>
#include <app_version.h>

#include <modules/sensor/sensor_module.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define BLINK_PERIOD_MS_STEP 100U
#define BLINK_PERIOD_MS_MAX  1000U

#define SENSORS_WARMUP_DELAY_MS 5000U
#define SENSOR_READ_INTERVAL_MS 1000U

/* ZBUS observer for sensor messages */
static void sensor_response_callback(const struct zbus_channel *chan);
ZBUS_LISTENER_DEFINE(sensor_response_listener, sensor_response_callback);

/* Function prototypes */
static int init_blink_led(const struct device **blink);
static void log_sensor_data(const struct sensor_msg *msg);

static int init_blink_led(const struct device **blink)
{
	*blink = DEVICE_DT_GET(DT_NODELABEL(blink_led));
	if (!device_is_ready(*blink)) {
		LOG_ERR("Blink LED not ready");
		return -ENODEV;
	}

	int ret = blink_off(*blink);
	if (ret < 0) {
		LOG_ERR("Could not turn off LED (%d)", ret);
		return ret;
	}

	LOG_INF("Blink LED initialized successfully");
	return 0;
}

static void log_sensor_data(const struct sensor_msg *msg)
{
	/* Log BME280 data */
	LOG_INF("BME280: Temp: %d.%06d C, Press: %d.%06d kPa, Hum: %d.%06d %%",
		msg->temperature.val1, msg->temperature.val2, msg->pressure.val1,
		msg->pressure.val2, msg->humidity.val1, msg->humidity.val2);

	/* Log CCS811 data */
	LOG_INF("CCS811: CO2: %d ppm, VOC: %d ppb", msg->co2.val1, msg->voc.val1);

	/* Log HM3301 data */
	LOG_INF("HM3301: PM1.0: %d ug/m3, PM2.5: %d ug/m3, PM10: %d ug/m3", msg->pm1_0.val1,
		msg->pm2_5.val1, msg->pm10.val1);

	LOG_INF("Sensor data timestamp: %lld ms", msg->timestamp);
}

static void sensor_response_callback(const struct zbus_channel *chan)
{
	const struct sensor_msg *msg;

	if (chan == &sensor_chan) {
		msg = &MSG_TO_SENSOR_MSG(zbus_chan_const_msg(chan));
		if (msg != NULL && msg->type == SENSOR_SAMPLE_RESPONSE) {
			LOG_DBG("Received sensor response");
			log_sensor_data(msg);
		}
	}
}

int main(void)
{
	int ret;
	const struct device *blink;

	LOG_INF("Zephyr Fire Detection System %s", APP_VERSION_STRING);

	/* Wait for sensors to stabilize */
	k_sleep(K_MSEC(SENSORS_WARMUP_DELAY_MS));

	/* Initialize blink LED */
	ret = init_blink_led(&blink);
	if (ret < 0) {
		LOG_ERR("Blink LED initialization failed");
		return 0;
	}

	/* Initialize sensor module */
	ret = sensor_module_init();
	if (ret < 0) {
		LOG_ERR("Sensor module initialization failed");
		return 0;
	}

	while (1) {
		/* Request sensor data */
		LOG_DBG("Requesting sensor data");
		ret = sensor_module_request_data();
		if (ret < 0) {
			LOG_ERR("Failed to request sensor data (%d)", ret);
		}

		k_sleep(K_MSEC(SENSOR_READ_INTERVAL_MS));
	}

	return 0;
}
