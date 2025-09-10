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

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(sen0466));
	struct sensor_value co, temp;

	LOG_INF("Starting SEN0466 CO sensor sample application");

	if (!device_is_ready(dev)) {
		LOG_ERR("SEN0466 device %s is not ready", dev->name);
		return -ENODEV;
	}

	LOG_INF("SEN0466 device %s is ready", dev->name);

	while (1) {
		int ret = sensor_sample_fetch(dev);
		if (ret) {
			LOG_ERR("sample_fetch error: %d", ret);
			k_sleep(K_SECONDS(1));
			return ret;
		}

		ret = sensor_channel_get(dev, (enum sensor_channel)SENSOR_CHAN_SEN0466_CO, &co);
		if (ret == 0) {
			LOG_INF("CO[ppm]: %d", co.val1);
		} else {
			LOG_ERR("Failed to get CO channel: %d", ret);
		}

		ret = sensor_channel_get(dev, (enum sensor_channel)SENSOR_CHAN_SEN0466_TEMP, &temp);
		if (ret == 0) {
			LOG_INF("TEMP[C]: %d.%06d", temp.val1, temp.val2);
		} else {
			LOG_ERR("Failed to get temperature channel: %d", ret);
		}

		k_sleep(K_SECONDS(2));
	}

	return 0;
}
