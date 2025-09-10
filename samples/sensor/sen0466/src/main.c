/*
 * Copyright (c) 2025 Natalia Pluta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/sen0466.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

static void print_configuration(void)
{
	LOG_INF("Configuration: Sample %s, Temp compensation %s",
#if defined(CONFIG_SEN0466_SAMPLE_ALL)
		"ALL",
#else
		"CO ONLY",
#endif
#if defined(CONFIG_SEN0466_TEMP_COMPENSATION)
#if defined(CONFIG_SEN0466_TEMP_COMP_INTEGRATED)
		"INTEGRATED"
#elif defined(CONFIG_SEN0466_TEMP_COMP_EXTERNAL)
		"EXTERNAL"
#else
		"UNKNOWN"
#endif
#else
		"DISABLED"
#endif
	);
}

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(sen0466));
	struct sensor_value co;

#if defined(CONFIG_SEN0466_SAMPLE_ALL)
	struct sensor_value temp;
#endif

	int sample_count = 0;

	LOG_INF("Starting SEN0466 CO sensor sample application");

	if (!device_is_ready(dev)) {
		LOG_ERR("SEN0466 device %s is not ready", dev->name);
		return -ENODEV;
	}

	LOG_INF("SEN0466 device %s is ready", dev->name);
	print_configuration();

#if defined(CONFIG_SEN0466_TEMP_COMP_EXTERNAL)
	/* Provide external temperature for testing external compensation */
	struct sensor_value ext_temp = {
		.val1 = 25,    /* 25°C */
		.val2 = 500000 /* 0.5°C fractional part = 25.5°C */
	};

	int ret = sen0466_temp_update(dev, &ext_temp);
	if (ret == 0) {
		LOG_INF("External temperature provided: %d.%06d C", ext_temp.val1, ext_temp.val2);
	} else {
		LOG_ERR("Failed to set external temperature: %d", ret);
	}
#endif

	while (1) {
		sample_count++;

		int ret = sensor_sample_fetch(dev);
		if (ret) {
			LOG_ERR("sample_fetch error: %d", ret);
			k_sleep(K_SECONDS(1));
			return ret;
		}

		/* Always try to get CO concentration */
		ret = sensor_channel_get(dev, (enum sensor_channel)SENSOR_CHAN_SEN0466_CO, &co);
		if (ret == 0) {
			LOG_INF("CO: %d ppm", co.val1);
		} else {
			LOG_ERR("Failed to get CO channel: %d", ret);
		}

#if defined(CONFIG_SEN0466_SAMPLE_ALL)
		/* Try to get temperature only if we're sampling all data */
		ret = sensor_channel_get(dev, (enum sensor_channel)SENSOR_CHAN_SEN0466_TEMP, &temp);
		if (ret == 0) {
			LOG_INF("TEMP: %d.%06d C", temp.val1, temp.val2);
		} else {
			LOG_ERR("Failed to get temperature channel: %d", ret);
		}
#else
		/* In CO-only mode, temperature channel should not be available */
		if (sample_count == 1) {
			LOG_INF("Temperature channel not available (CO-only mode)");
		}
#endif

#if defined(CONFIG_SEN0466_TEMP_COMP_EXTERNAL)
		/* Update external temperature every few samples for testing */
		if (sample_count % 5 == 0) {
			ext_temp.val1 = 25 + (sample_count / 5); /* Vary temperature for testing */
			ret = sen0466_temp_update(dev, &ext_temp);
			if (ret == 0) {
				LOG_INF("Updated external temperature: %d.%06d C", ext_temp.val1,
					ext_temp.val2);
			}
		}
#endif

		k_sleep(K_SECONDS(2));
	}

	return 0;
}
