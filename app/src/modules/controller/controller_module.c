/*
 * Copyright (c) 2025 Natalia Pluta
 * SPDX-License-Identifier: Apache-2.0
 */

#include "controller_module.h"
#include <zephyr/logging/log.h>
#include <zephyr/smf.h>
#include <modules/sensor/sensor_module.h>

LOG_MODULE_REGISTER(controller_module, CONFIG_CONTROLLER_MODULE_LOG_LEVEL);

/* ZBUS subscriber for sensor responses */
ZBUS_SUBSCRIBER_DEFINE(controller_sensor_subscriber,
		       CONFIG_CONTROLLER_MODULE_ZBUS_SUBSCRIBER_QUEUE_SIZE);

/* Controller state machine object initialization macro */
#define CONTROLLER_STATE_OBJECT_INIT()                                                             \
	(struct controller_state_object)                                                           \
	{                                                                                          \
		.sample_interval_ms = CONFIG_CONTROLLER_MODULE_SAMPLE_INTERVAL_MS,                 \
		.max_retries = CONFIG_CONTROLLER_MODULE_MAX_RETRIES, .sampling_active = false,     \
		.sensor_module_ready = false, .ctx = {0}, .error_count = 0, .last_sample_time = 0, \
		.current_state = CONTROLLER_MODULE_STATE_INIT,                                     \
	}

/* Controller state machine object */
struct controller_state_object {
	struct smf_ctx ctx;

	/* Current state tracking */
	enum controller_module_state current_state;

	/* Sampling control */
	bool sampling_active;
	int64_t last_sample_time;
	int64_t sample_interval_ms;

	/* Error handling */
	int error_count;
	int max_retries;

	/* Module status tracking */
	bool sensor_module_ready;
};

/* Forward declarations for state functions */
static void controller_state_init_run(void *obj);
static void controller_state_idle_run(void *obj);
static void controller_state_active_run(void *obj);
static void controller_state_error_run(void *obj);
static void controller_state_recovery_run(void *obj);

/* State machine table */
static const struct smf_state controller_states[] = {
	[CONTROLLER_MODULE_STATE_INIT] =
		SMF_CREATE_STATE(NULL, controller_state_init_run, NULL, NULL, NULL),
	[CONTROLLER_MODULE_STATE_IDLE] =
		SMF_CREATE_STATE(NULL, controller_state_idle_run, NULL, NULL, NULL),
	[CONTROLLER_MODULE_STATE_ACTIVE] =
		SMF_CREATE_STATE(NULL, controller_state_active_run, NULL, NULL, NULL),
	[CONTROLLER_MODULE_STATE_ERROR] =
		SMF_CREATE_STATE(NULL, controller_state_error_run, NULL, NULL, NULL),
	[CONTROLLER_MODULE_STATE_RECOVERY] =
		SMF_CREATE_STATE(NULL, controller_state_recovery_run, NULL, NULL, NULL)};

/* Global controller state machine object */
static struct controller_state_object controller_state_obj;

/* Thread synchronization */
static K_MUTEX_DEFINE(controller_sm_mutex);

/* Thread stack and data */
static K_THREAD_STACK_DEFINE(controller_thread_stack, CONFIG_CONTROLLER_MODULE_STACK_SIZE);
static struct k_thread controller_thread_data;

/* Forward declarations */
static void controller_thread(void *p1, void *p2, void *p3);
static void sensor_response_callback(const struct zbus_channel *chan);
static void log_sensor_data(const struct sensor_msg *msg);
static void controller_set_state(struct controller_state_object *ctx,
				 enum controller_module_state new_state);

/* ZBUS listener for sensor responses */
ZBUS_LISTENER_DEFINE(controller_sensor_listener, sensor_response_callback);

int controller_module_init(void)
{
	int ret;

	LOG_INF("Initializing controller module");

	/* Initialize state machine object */
	controller_state_obj = CONTROLLER_STATE_OBJECT_INIT();

	/* Initialize state machine */
	smf_set_initial(SMF_CTX(&controller_state_obj),
			&controller_states[CONTROLLER_MODULE_STATE_INIT]);
	controller_state_obj.current_state = CONTROLLER_MODULE_STATE_INIT;

	LOG_INF("Controller state machine initialized");

	/* Run initial state machine setup */
	k_mutex_lock(&controller_sm_mutex, K_FOREVER);
	ret = smf_run_state(SMF_CTX(&controller_state_obj));
	k_mutex_unlock(&controller_sm_mutex);
	if (ret < 0) {
		LOG_ERR("Failed to run controller state machine (%d)", ret);
		return ret;
	}

	/* Create controller thread */
	k_thread_create(&controller_thread_data, controller_thread_stack,
			K_THREAD_STACK_SIZEOF(controller_thread_stack), controller_thread, NULL,
			NULL, NULL, CONFIG_CONTROLLER_MODULE_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(&controller_thread_data, "controller_module");

	LOG_INF("Controller module initialized successfully");
	return 0;
}

int controller_module_start_sampling(void)
{
	k_mutex_lock(&controller_sm_mutex, K_FOREVER);

	if (controller_state_obj.current_state == CONTROLLER_MODULE_STATE_IDLE) {
		controller_state_obj.sampling_active = true;
		controller_state_obj.last_sample_time = k_uptime_get();
		controller_set_state(&controller_state_obj, CONTROLLER_MODULE_STATE_ACTIVE);
		LOG_INF("Controller: Starting data sampling");
	} else {
		LOG_WRN("Controller: Cannot start sampling from current state");
	}

	k_mutex_unlock(&controller_sm_mutex);
	return 0;
}

int controller_module_stop_sampling(void)
{
	k_mutex_lock(&controller_sm_mutex, K_FOREVER);

	controller_state_obj.sampling_active = false;
	if (controller_state_obj.current_state == CONTROLLER_MODULE_STATE_ACTIVE) {
		controller_set_state(&controller_state_obj, CONTROLLER_MODULE_STATE_IDLE);
		LOG_INF("Controller: Stopping data sampling");
	}

	k_mutex_unlock(&controller_sm_mutex);
	return 0;
}

enum controller_module_state controller_module_get_state(void)
{
	k_mutex_lock(&controller_sm_mutex, K_FOREVER);
	enum controller_module_state current_state = controller_state_obj.current_state;
	k_mutex_unlock(&controller_sm_mutex);
	return current_state;
}

static void controller_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct zbus_channel *chan;

	LOG_INF("Controller thread started");

	while (1) {
		/* Check for ZBUS messages with timeout */
		int ret = zbus_sub_wait(&controller_sensor_subscriber, &chan,
					K_MSEC(CONFIG_CONTROLLER_MODULE_THREAD_SLEEP_MS));

		if (ret == 0) {
			/* Process received message if it's from sensor channel */
			if (chan == &sensor_chan) {
				LOG_DBG("Controller: Received sensor data via ZBUS");
				/* Message processing is handled by the listener callback */
			}
		}

		/* Run state machine periodically */
		k_mutex_lock(&controller_sm_mutex, K_FOREVER);
		smf_run_state(SMF_CTX(&controller_state_obj));
		k_mutex_unlock(&controller_sm_mutex);

		/* Small sleep to prevent tight loop */
		k_sleep(K_MSEC(CONFIG_CONTROLLER_MODULE_THREAD_SLEEP_MS));
	}
}

/* State machine implementation */
static void controller_state_init_run(void *obj)
{
	int ret;
	struct controller_state_object *ctx = (struct controller_state_object *)obj;

	LOG_INF("Controller SM: Initializing");

	/* Initialize sensor module */
	ret = sensor_module_init();
	if (ret < 0) {
		LOG_ERR("Controller SM: Sensor module initialization failed (%d)", ret);
		ctx->error_count++;
		controller_set_state(ctx, CONTROLLER_MODULE_STATE_ERROR);
		return;
	}

	ctx->sensor_module_ready = true;
	ctx->error_count = 0;

	LOG_INF("Controller SM: Initialization complete");
	controller_set_state(ctx, CONTROLLER_MODULE_STATE_IDLE);
}

static void controller_state_idle_run(void *obj)
{
	struct controller_state_object *ctx = (struct controller_state_object *)obj;

	LOG_DBG("Controller SM: Idle state");

	/* Check if sampling should be started automatically */
	if (CONFIG_CONTROLLER_MODULE_AUTO_START_SAMPLING && !ctx->sampling_active) {
		LOG_INF("Controller SM: Auto-starting sampling");
		ctx->sampling_active = true;
		ctx->last_sample_time = k_uptime_get();
		controller_set_state(ctx, CONTROLLER_MODULE_STATE_ACTIVE);
	}
}

static void controller_state_active_run(void *obj)
{
	int ret;
	struct controller_state_object *ctx = (struct controller_state_object *)obj;

	LOG_DBG("Controller SM: Active state");

	/* Check if sampling should be stopped */
	if (!ctx->sampling_active) {
		controller_set_state(ctx, CONTROLLER_MODULE_STATE_IDLE);
		return;
	}

	/* Check if it's time for next sample */
	int64_t now = k_uptime_get();
	if (now - ctx->last_sample_time >= ctx->sample_interval_ms) {
		LOG_DBG("Controller SM: Requesting sensor data");

		ret = sensor_module_request_data();
		if (ret < 0) {
			LOG_ERR("Controller SM: Failed to request sensor data (%d)", ret);
			ctx->error_count++;
			if (ctx->error_count > ctx->max_retries) {
				controller_set_state(ctx, CONTROLLER_MODULE_STATE_ERROR);
				return;
			}
		} else {
			ctx->last_sample_time = now;
			ctx->error_count = 0; /* Reset error count on successful request */
		}
	}
}

static void controller_state_error_run(void *obj)
{
	struct controller_state_object *ctx = (struct controller_state_object *)obj;

	LOG_ERR("Controller SM: Error state");

	/* Wait before attempting recovery */
	k_sleep(K_MSEC(CONFIG_CONTROLLER_MODULE_RECOVERY_DELAY_MS));

	controller_set_state(ctx, CONTROLLER_MODULE_STATE_RECOVERY);
}

static void controller_state_recovery_run(void *obj)
{
	struct controller_state_object *ctx = (struct controller_state_object *)obj;

	LOG_INF("Controller SM: Attempting recovery");

	/* Reset error count and try to recover */
	ctx->error_count = 0;
	ctx->sampling_active = false;
	ctx->sensor_module_ready = false;

	/* Re-initialize sensor module */
	int ret = sensor_module_init();
	if (ret < 0) {
		LOG_ERR("Controller SM: Recovery failed (%d)", ret);
		ctx->error_count++;

		if (ctx->error_count > CONFIG_CONTROLLER_MODULE_MAX_RECOVERY_ATTEMPTS) {
			LOG_ERR("Controller SM: Recovery attempts exhausted");
			controller_set_state(ctx, CONTROLLER_MODULE_STATE_ERROR);
		} else {
			/* Try recovery again */
			k_sleep(K_MSEC(CONFIG_CONTROLLER_MODULE_RECOVERY_RETRY_DELAY_MS));
		}
	} else {
		LOG_INF("Controller SM: Recovery successful");
		ctx->sensor_module_ready = true;
		controller_set_state(ctx, CONTROLLER_MODULE_STATE_IDLE);
	}
}

static void sensor_response_callback(const struct zbus_channel *chan)
{
	const struct sensor_msg *msg;
	const void *chan_msg;

	if (chan == &sensor_chan) {
		chan_msg = zbus_chan_const_msg(chan);
		if (chan_msg != NULL) {
			msg = &MSG_TO_SENSOR_MSG(chan_msg);
			if (msg->type == SENSOR_SAMPLE_RESPONSE) {
				LOG_DBG("Controller: Processing sensor response");
				log_sensor_data(msg);

				/* Reset error count on successful data reception */
				k_mutex_lock(&controller_sm_mutex, K_FOREVER);
				controller_state_obj.error_count = 0;
				k_mutex_unlock(&controller_sm_mutex);
			}
		}
	}
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

	/* Log SEN0466 data */
	LOG_INF("SEN0466: CO: %d ppm, Temp: %d.%06d C", msg->co.val1, msg->temperature_sen0466.val1,
		msg->temperature_sen0466.val2);

	LOG_INF("Sensor data timestamp: %lld ms", msg->timestamp);
}

/* Helper function to set state and keep current_state field in sync */
static void controller_set_state(struct controller_state_object *ctx,
				 enum controller_module_state new_state)
{
	ctx->current_state = new_state;
	smf_set_state(SMF_CTX(ctx), &controller_states[new_state]);
}
