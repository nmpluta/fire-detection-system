/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/sen0466.h>
#include <zephyr/task_wdt/task_wdt.h>
#include <zephyr/smf.h>

#if defined(CONFIG_APP_ENVIRONMENTAL_TIMESTAMP)
#include <date_time.h>
#endif

#include "app_common.h"
#include "environmental.h"

/* Register log module */
LOG_MODULE_REGISTER(environmental, CONFIG_APP_ENVIRONMENTAL_LOG_LEVEL);

/* Define channels provided by this module */
ZBUS_CHAN_DEFINE(ENVIRONMENTAL_CHAN, struct environmental_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));

/* Register subscriber */
ZBUS_MSG_SUBSCRIBER_DEFINE(environmental_subscriber);

/* Observe channels */
ZBUS_CHAN_ADD_OBS(ENVIRONMENTAL_CHAN, environmental_subscriber, 0);

#define MAX_MSG_SIZE sizeof(struct environmental_msg)

BUILD_ASSERT(CONFIG_APP_ENVIRONMENTAL_WATCHDOG_TIMEOUT_SECONDS >
		     CONFIG_APP_ENVIRONMENTAL_MSG_PROCESSING_TIMEOUT_SECONDS,
	     "Watchdog timeout must be greater than maximum message processing time");

/* State machine */

/* Environmental module states.
 */
enum environmental_module_state {
	/* The module is running and waiting for sensor value requests */
	STATE_RUNNING,
};

/* State object.
 * Used to transfer context data between state changes.
 */
struct environmental_state_object {
	/* This must be first */
	struct smf_ctx ctx;

	/* Last channel type that a message was received on */
	const struct zbus_channel *chan;

	/* Buffer for last zbus message */
	uint8_t msg_buf[MAX_MSG_SIZE];

	/* Pointer to the sensors devices */
	const struct device *const bme280;
	const struct device *const ccs811;
	const struct device *const hm330x;
	const struct device *const sen0466;

	/* Sensor values */
	double temperature;
	double pressure;
	double humidity;
	double co2;
	double voc;
	double pm1_0;
	double pm2_5;
	double pm10;
	double co;
	double temperature_sen0466;
};

/* Forward declarations of state handlers */
static void state_running_run(void *obj);

/* State machine definition */
static const struct smf_state states[] = {
	[STATE_RUNNING] = SMF_CREATE_STATE(NULL, state_running_run, NULL, NULL, NULL),
};

static void sample_sensors(const struct environmental_state_object *state)
{
	int err;
	struct sensor_value temp = {0}, press = {0}, humidity = {0};
	struct sensor_value co2 = {0}, voc = {0};
	struct sensor_value pm1_0 = {0}, pm2_5 = {0}, pm10 = {0};
	struct sensor_value co = {0}, temp_sen0466 = {0};

	// BME280
	if (state->bme280) {
		err = sensor_sample_fetch(state->bme280);
		if (err) {
			LOG_ERR("BME280: sensor_sample_fetch, error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}
		err = sensor_channel_get(state->bme280, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		if (err) {
			LOG_WRN("BME280: temp read failed: %d", err);
		}
		err = sensor_channel_get(state->bme280, SENSOR_CHAN_PRESS, &press);
		if (err) {
			LOG_WRN("BME280: press read failed: %d", err);
		}
		err = sensor_channel_get(state->bme280, SENSOR_CHAN_HUMIDITY, &humidity);
		if (err) {
			LOG_WRN("BME280: humidity read failed: %d", err);
		}
	}

	// CCS811
	if (state->ccs811) {
		err = sensor_sample_fetch(state->ccs811);
		if (err) {
			LOG_WRN("CCS811: sensor_sample_fetch, error: %d", err);
		} else {
			err = sensor_channel_get(state->ccs811, SENSOR_CHAN_CO2, &co2);
			if (err) {
				LOG_WRN("CCS811: CO2 read failed: %d", err);
			}
			err = sensor_channel_get(state->ccs811, SENSOR_CHAN_VOC, &voc);
			if (err) {
				LOG_WRN("CCS811: VOC read failed: %d", err);
			}
		}
	}

	// HM330X
	if (state->hm330x) {
		err = sensor_sample_fetch(state->hm330x);
		if (err) {
			LOG_WRN("HM330X: sensor_sample_fetch, error: %d", err);
		} else {
			err = sensor_channel_get(state->hm330x, SENSOR_CHAN_PM_1_0, &pm1_0);
			if (err) {
				LOG_WRN("HM330X: PM1.0 read failed: %d", err);
			}
			err = sensor_channel_get(state->hm330x, SENSOR_CHAN_PM_2_5, &pm2_5);
			if (err) {
				LOG_WRN("HM330X: PM2.5 read failed: %d", err);
			}
			err = sensor_channel_get(state->hm330x, SENSOR_CHAN_PM_10, &pm10);
			if (err) {
				LOG_WRN("HM330X: PM10 read failed: %d", err);
			}
		}
	}

	// SEN0466
	if (state->sen0466) {
		err = sensor_sample_fetch(state->sen0466);
		if (err) {
			LOG_WRN("SEN0466: sensor_sample_fetch, error: %d", err);
		} else {
			err = sensor_channel_get(state->sen0466, SENSOR_CHAN_SEN0466_CO, &co);
			if (err) {
				LOG_WRN("SEN0466: CO read failed: %d", err);
			}
			err = sensor_channel_get(state->sen0466, SENSOR_CHAN_SEN0466_TEMP,
						 &temp_sen0466);
			if (err) {
				LOG_WRN("SEN0466: temp read failed: %d", err);
			}
		}
	}

	// Transform and store in state object fields
	((struct environmental_state_object *)state)->temperature = sensor_value_to_double(&temp);
	((struct environmental_state_object *)state)->pressure = sensor_value_to_double(&press);
	((struct environmental_state_object *)state)->humidity = sensor_value_to_double(&humidity);
	((struct environmental_state_object *)state)->co2 = sensor_value_to_double(&co2);
	((struct environmental_state_object *)state)->voc = sensor_value_to_double(&voc);
	((struct environmental_state_object *)state)->pm1_0 = sensor_value_to_double(&pm1_0);
	((struct environmental_state_object *)state)->pm2_5 = sensor_value_to_double(&pm2_5);
	((struct environmental_state_object *)state)->pm10 = sensor_value_to_double(&pm10);
	((struct environmental_state_object *)state)->co = sensor_value_to_double(&co);
	((struct environmental_state_object *)state)->temperature_sen0466 =
		sensor_value_to_double(&temp_sen0466);

	struct environmental_msg msg = {
		.type = ENVIRONMENTAL_SENSOR_SAMPLE_RESPONSE,
		.temperature = ((struct environmental_state_object *)state)->temperature,
		.pressure = ((struct environmental_state_object *)state)->pressure,
		.humidity = ((struct environmental_state_object *)state)->humidity,
		.co2 = ((struct environmental_state_object *)state)->co2,
		.voc = ((struct environmental_state_object *)state)->voc,
		.pm1_0 = ((struct environmental_state_object *)state)->pm1_0,
		.pm2_5 = ((struct environmental_state_object *)state)->pm2_5,
		.pm10 = ((struct environmental_state_object *)state)->pm10,
		.co = ((struct environmental_state_object *)state)->co,
		.temperature_sen0466 =
			((struct environmental_state_object *)state)->temperature_sen0466,
	};

#if defined(CONFIG_APP_ENVIRONMENTAL_TIMESTAMP)
	err = date_time_now(&msg.timestamp);
	if (err) {
		LOG_ERR("date_time_now() failed, error: %d, using 0", err);
	}
#endif /* CONFIG_APP_ENVIRONMENTAL_TIMESTAMP */

	LOG_DBG("Sampled: T:%.2fC P:%.2fPa H:%.2f%% CO2:%.2f VOC:%.2f PM1.0:%.2f PM2.5:%.2f "
		"PM10:%.2f CO:%.2f T2:%.2fC",
		msg.temperature, msg.pressure, msg.humidity, msg.co2, msg.voc, msg.pm1_0, msg.pm2_5,
		msg.pm10, msg.co, msg.temperature_sen0466);

	LOG_INF("BME280: T:%.2fC P:%.2fkPa H:%.2f%%", msg.temperature, msg.pressure, msg.humidity);
	LOG_INF("CCS811: CO2:%.2f VOC:%.2f", msg.co2, msg.voc);
	LOG_INF("HM330X: PM1.0:%.2f PM2.5:%.2f PM10:%.2f", msg.pm1_0, msg.pm2_5, msg.pm10);
	LOG_INF("SEN0466: CO:%.2f T2:%.2fC", msg.co, msg.temperature_sen0466);

	err = zbus_chan_pub(&ENVIRONMENTAL_CHAN, &msg, K_NO_WAIT);
	if (err) {
		LOG_ERR("zbus_chan_pub, error: %d", err);
		SEND_FATAL_ERROR();
		return;
	}
}

static void env_wdt_callback(int channel_id, void *user_data)
{
	LOG_ERR("Watchdog expired, Channel: %d, Thread: %s", channel_id,
		k_thread_name_get((k_tid_t)user_data));

	SEND_FATAL_ERROR_WATCHDOG_TIMEOUT();
}

/* State handlers */

static void state_running_run(void *obj)
{
	struct environmental_state_object const *state_object = obj;

	if (&ENVIRONMENTAL_CHAN == state_object->chan) {
		struct environmental_msg msg = MSG_TO_ENVIRONMENTAL_MSG(state_object->msg_buf);

		if (msg.type == ENVIRONMENTAL_SENSOR_SAMPLE_REQUEST) {
			LOG_DBG("Environmental values sample request received, getting data");
			sample_sensors(state_object);
		}
	}
}

static void env_module_thread(void)
{
	int err;
	int task_wdt_id;
	const uint32_t wdt_timeout_ms =
		(CONFIG_APP_ENVIRONMENTAL_WATCHDOG_TIMEOUT_SECONDS * MSEC_PER_SEC);
	const uint32_t execution_time_ms =
		(CONFIG_APP_ENVIRONMENTAL_MSG_PROCESSING_TIMEOUT_SECONDS * MSEC_PER_SEC);
	const k_timeout_t zbus_wait_ms = K_MSEC(wdt_timeout_ms - execution_time_ms);
	struct environmental_state_object environmental_state = {
		.bme280 = DEVICE_DT_GET(DT_NODELABEL(bme280)),
		.ccs811 = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(ccs811)),
		.hm330x = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(hm330x)),
		.sen0466 = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(sen0466)),
	};

	LOG_DBG("Environmental module task started");

	task_wdt_id = task_wdt_add(wdt_timeout_ms, env_wdt_callback, (void *)k_current_get());
	if (task_wdt_id < 0) {
		LOG_ERR("Failed to add task to watchdog: %d", task_wdt_id);
		SEND_FATAL_ERROR();
		return;
	}

	smf_set_initial(SMF_CTX(&environmental_state), &states[STATE_RUNNING]);

	while (true) {
		err = task_wdt_feed(task_wdt_id);
		if (err) {
			LOG_ERR("task_wdt_feed, error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}

		err = zbus_sub_wait_msg(&environmental_subscriber, &environmental_state.chan,
					environmental_state.msg_buf, zbus_wait_ms);
		if (err == -ENOMSG) {
			continue;
		} else if (err) {
			LOG_ERR("zbus_sub_wait_msg, error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}

		err = smf_run_state(SMF_CTX(&environmental_state));
		if (err) {
			LOG_ERR("smf_run_state(), error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}
	}
}

K_THREAD_DEFINE(environmental_module_thread_id, CONFIG_APP_ENVIRONMENTAL_THREAD_STACK_SIZE,
		env_module_thread, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
