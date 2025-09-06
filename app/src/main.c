/*
 * Copyright (c) 2025 Natalia Pluta
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app/drivers/blink.h>
#include <app_version.h>

#include <modules/controller/controller_module.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

/* Sleep duration for main loop */
#define MAIN_LOOP_SLEEP_SECONDS 100U

/* Function prototypes */
static int init_blink_led(const struct device **blink);

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

int main(void)
{
	int ret;
	const struct device *blink;

	LOG_INF("Zephyr Fire Detection System %s", APP_VERSION_STRING);

	/* Initialize blink LED */
	ret = init_blink_led(&blink);
	if (ret < 0) {
		LOG_ERR("Blink LED initialization failed");
		return 0;
	}

	/* Initialize controller module */
	ret = controller_module_init();
	if (ret < 0) {
		LOG_ERR("Controller module initialization failed");
		return 0;
	}

	/* Start data sampling */
	ret = controller_module_start_sampling();
	if (ret < 0) {
		LOG_ERR("Failed to start sampling");
		return 0;
	}

	LOG_INF("System initialized successfully");

	while (1) {
		k_sleep(K_SECONDS(MAIN_LOOP_SLEEP_SECONDS));
	}

	return 0;
}
