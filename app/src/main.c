/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <app/drivers/blink.h>

#include <app_version.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define BLINK_PERIOD_MS_STEP 100U
#define BLINK_PERIOD_MS_MAX  1000U

int main(void)
{
	int ret;
	const struct device *sensor, *blink;
	struct sensor_value temp, press, hum;

	LOG_INF("Zephyr Fire Detection System %s", APP_VERSION_STRING);

	sensor = DEVICE_DT_GET(DT_NODELABEL(bme280));
	if (!device_is_ready(sensor)) {
		LOG_ERR("BME280 sensor not ready");
		return 0;
	}

	blink = DEVICE_DT_GET(DT_NODELABEL(blink_led));
	if (!device_is_ready(blink)) {
		LOG_ERR("Blink LED not ready");
		return 0;
	}

	ret = blink_off(blink);
	if (ret < 0) {
		LOG_ERR("Could not turn off LED (%d)", ret);
		return 0;
	}

	while (1) {
		ret = sensor_sample_fetch(sensor);
		if (ret < 0) {
			LOG_ERR("Could not fetch BME280 sample (%d)", ret);
			k_sleep(K_MSEC(1000));
			continue;
		}

		ret = sensor_channel_get(sensor, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		if (ret < 0) {
			LOG_ERR("Could not get temperature (%d)", ret);
			continue;
		}

		ret = sensor_channel_get(sensor, SENSOR_CHAN_PRESS, &press);
		if (ret < 0) {
			LOG_ERR("Could not get pressure (%d)", ret);
			continue;
		}

		ret = sensor_channel_get(sensor, SENSOR_CHAN_HUMIDITY, &hum);
		if (ret < 0) {
			LOG_ERR("Could not get humidity (%d)", ret);
			continue;
		}

		LOG_INF("BME280: Temp: %d.%06d C, Press: %d.%06d kPa, Hum: %d.%06d %%", temp.val1,
			temp.val2, press.val1, press.val2, hum.val1, hum.val2);

		k_sleep(K_MSEC(1000));
	}

	return 0;
}
