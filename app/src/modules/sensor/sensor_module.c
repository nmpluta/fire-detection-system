/*
 * Copyright (c) 2025 Natalia Pluta
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sensor_module.h"
#include <zephyr/drivers/sensor/ccs811.h>
#include <zephyr/drivers/sensor/sen0466.h>
#include <zephyr/logging/log.h>
#include <zephyr/smf.h>

LOG_MODULE_REGISTER(sensor_module, CONFIG_SENSOR_MODULE_LOG_LEVEL);

/* Macro to validate sensor type enum values */
#define IS_VALID_SENSOR_TYPE(type) ((int)(type) >= 0 && (type) < SENSOR_TYPE_COUNT)

/* Macro for type-safe sensor state object initialization */
#define SENSOR_STATE_OBJECT_INIT()                                                                 \
	(struct sensor_state_object)                                                               \
	{                                                                                          \
		.ctx = {0}, .current_state = SENSOR_MODULE_STATE_INIT, .current_data = {0},        \
		.error_count = 0, .max_retries = 0, .last_read_time = 0, .read_timeout_ms = 0,     \
		IF_ENABLED(CONFIG_SENSOR_MODULE_WARMUP_ENABLE, \
			   (.sensor_init_time = 0,                \
				.sensor_warmup_complete = {false}))                                   \
	}

/* ZBUS subscriber for sensor requests */
ZBUS_SUBSCRIBER_DEFINE(sensor_request_subscriber, CONFIG_SENSOR_MODULE_ZBUS_SUBSCRIBER_QUEUE_SIZE);

/* Sensor state machine states */
enum sensor_module_state {
	SENSOR_MODULE_STATE_INIT,
	SENSOR_MODULE_STATE_IDLE,
	SENSOR_MODULE_STATE_READING,
	SENSOR_MODULE_STATE_ERROR,
	SENSOR_MODULE_STATE_RECOVERY
};

/* Sensor state machine events */
enum sensor_module_event {
	SENSOR_MODULE_EVENT_INIT_COMPLETE,
	SENSOR_MODULE_EVENT_READ_REQUEST,
	SENSOR_MODULE_EVENT_READ_SUCCESS,
	SENSOR_MODULE_EVENT_READ_ERROR,
	SENSOR_MODULE_EVENT_RECOVERY_ATTEMPT,
	SENSOR_MODULE_EVENT_TIMEOUT
};

/* Sensor state machine object */
struct sensor_state_object {
	struct smf_ctx ctx;

	/* Current state tracking */
	enum sensor_module_state current_state;

	/* Current sensor data */
	struct sensor_msg current_data;

	/* Error handling */
	int error_count;
	int max_retries;

	/* Timing */
	int64_t last_read_time;
	int64_t read_timeout_ms;

#if defined(CONFIG_SENSOR_MODULE_WARMUP_ENABLE)
	/* Sensor warmup tracking */
	int64_t sensor_init_time;
	bool sensor_warmup_complete[SENSOR_TYPE_COUNT];
#endif
};

/* Forward declarations for state functions */
static void sensor_state_init_run(void *obj);
static void sensor_state_idle_run(void *obj);
static void sensor_state_reading_run(void *obj);
static void sensor_state_error_run(void *obj);
static void sensor_state_recovery_run(void *obj);

/* State machine table */
static const struct smf_state sensor_states[] = {
	[SENSOR_MODULE_STATE_INIT] =
		SMF_CREATE_STATE(NULL, sensor_state_init_run, NULL, NULL, NULL),
	[SENSOR_MODULE_STATE_IDLE] =
		SMF_CREATE_STATE(NULL, sensor_state_idle_run, NULL, NULL, NULL),
	[SENSOR_MODULE_STATE_READING] =
		SMF_CREATE_STATE(NULL, sensor_state_reading_run, NULL, NULL, NULL),
	[SENSOR_MODULE_STATE_ERROR] =
		SMF_CREATE_STATE(NULL, sensor_state_error_run, NULL, NULL, NULL),
	[SENSOR_MODULE_STATE_RECOVERY] =
		SMF_CREATE_STATE(NULL, sensor_state_recovery_run, NULL, NULL, NULL)};

/* Global sensor state machine context */
static struct sensor_state_object sensor_state_obj;

/* Thread synchronization */
static K_MUTEX_DEFINE(sensor_sm_mutex);

/* CCS811 environmental compensation configuration */
#define SENSOR_VALUE_TO_MICRO(val) ((val)->val1 * 1000000UL + (val)->val2)

/* ZBUS channel definition */
ZBUS_CHAN_DEFINE(sensor_chan, struct sensor_msg, NULL, NULL,
		 ZBUS_OBSERVERS(controller_sensor_listener, sensor_request_subscriber),
		 ZBUS_MSG_INIT(0));

/* Sensor device pointers */
static struct sensor_info sensors[SENSOR_TYPE_COUNT] = {
	[SENSOR_TYPE_BME280] = {.device = DEVICE_DT_GET(DT_NODELABEL(bme280)),
				.health = {0},
				.enabled = true},
	[SENSOR_TYPE_CCS811] = {.device = DEVICE_DT_GET(DT_NODELABEL(ccs811)),
				.health = {0},
				.enabled = true},
	[SENSOR_TYPE_HM3301] = {.device = DEVICE_DT_GET(DT_NODELABEL(hm3301)),
				.health = {0},
				.enabled = true},
	[SENSOR_TYPE_SEN0466] = {
		.device = DEVICE_DT_GET(DT_NODELABEL(sen0466)), .health = {0}, .enabled = true}};

/* Thread stack and data */
static K_THREAD_STACK_DEFINE(sensor_thread_stack, CONFIG_SENSOR_MODULE_STACK_SIZE);
static struct k_thread sensor_thread_data;

/* Forward declarations */
static void sensor_thread(void *p1, void *p2, void *p3);
static int init_sensors(void);
static int read_sensor_data(enum sensor_type type, struct sensor_msg *data);
static void update_sensor_health(struct sensor_health *health, bool success);
static const char *get_sensor_name(enum sensor_type type);
static void sensor_set_state(struct sensor_state_object *ctx, enum sensor_module_state new_state);

#if defined(CONFIG_SENSOR_MODULE_WARMUP_ENABLE)
static bool is_sensor_warmup_complete(enum sensor_type type);
static int64_t get_sensor_warmup_time(enum sensor_type type);
#endif

#if defined(CONFIG_CCS811_ENV_COMPENSATION)
static int update_ccs811_env_data(const struct sensor_value *temp, const struct sensor_value *hum);
static void handle_ccs811_env_compensation(const struct sensor_value *temp,
					   const struct sensor_value *hum);
static void handle_ccs811_fallback_compensation(void);
#endif

int sensor_module_init(void)
{
	int ret;

	/* Initialize state machine context */
	sensor_state_obj = SENSOR_STATE_OBJECT_INIT();

	sensor_state_obj.max_retries = CONFIG_SENSOR_MODULE_MAX_RETRIES;
	sensor_state_obj.read_timeout_ms = CONFIG_SENSOR_MODULE_READ_TIMEOUT_MS;

	/* Initialize state machine */
	smf_set_initial(SMF_CTX(&sensor_state_obj), &sensor_states[SENSOR_MODULE_STATE_INIT]);
	sensor_state_obj.current_state = SENSOR_MODULE_STATE_INIT;

	LOG_INF("Sensor state machine initialized");

	/* Initialize sensors using state machine with mutex protection */
	k_mutex_lock(&sensor_sm_mutex, K_FOREVER);
	ret = smf_run_state(SMF_CTX(&sensor_state_obj));
	k_mutex_unlock(&sensor_sm_mutex);
	if (ret < 0) {
		LOG_ERR("Failed to run sensor state machine (%d)", ret);
		return ret;
	}

	/* Create single sensor thread for processing ZBUS messages */
	k_thread_create(&sensor_thread_data, sensor_thread_stack,
			K_THREAD_STACK_SIZEOF(sensor_thread_stack), sensor_thread, NULL, NULL, NULL,
			CONFIG_SENSOR_MODULE_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(&sensor_thread_data, "sensor_module");

	LOG_INF("Sensor module initialized successfully - sensor thread running");
	return 0;
}

int sensor_module_request_data(void)
{
	struct sensor_msg request = {.type = SENSOR_SAMPLE_REQUEST};

	int ret = zbus_chan_pub(&sensor_chan, &request,
				K_MSEC(CONFIG_SENSOR_MODULE_PUBLISH_TIMEOUT_MS));
	if (ret < 0) {
		LOG_ERR("Failed to publish sensor request (%d)", ret);
		return ret;
	}

	LOG_DBG("Sensor request published via ZBUS");
	return 0;
}

static void sensor_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct zbus_channel *chan;
	const struct sensor_msg *msg;

	LOG_INF("Sensor thread started - processing ZBUS requests directly");

	while (1) {
		/* Wait for messages directly from ZBUS subscriber */
		if (zbus_sub_wait(&sensor_request_subscriber, &chan, K_FOREVER) == 0) {
			if (chan == &sensor_chan) {
				const void *chan_msg = zbus_chan_const_msg(chan);

				if (chan_msg != NULL) {
					msg = &MSG_TO_SENSOR_MSG(chan_msg);

					if (msg->type == SENSOR_SAMPLE_REQUEST) {
						LOG_DBG("Processing sensor request directly");

						/* Trigger state machine to read sensors with mutex
						 * protection */
						k_mutex_lock(&sensor_sm_mutex, K_FOREVER);
						if (sensor_state_obj.current_state ==
						    SENSOR_MODULE_STATE_IDLE) {
							sensor_set_state(
								&sensor_state_obj,
								SENSOR_MODULE_STATE_READING);
						}

						/* Run state machine - this performs blocking sensor
						 * operations */
						int ret = smf_run_state(SMF_CTX(&sensor_state_obj));
						if (ret < 0) {
							LOG_ERR("State machine execution failed "
								"(%d)",
								ret);
							/* Error handling is done internally by the
							 * state machine */
						}
						k_mutex_unlock(&sensor_sm_mutex);
					}
				}
			}
		}

		/* Also run state machine periodically for maintenance and recovery */
		k_mutex_lock(&sensor_sm_mutex, K_FOREVER);
		smf_run_state(SMF_CTX(&sensor_state_obj));
		k_mutex_unlock(&sensor_sm_mutex);
		k_sleep(K_MSEC(CONFIG_SENSOR_MODULE_THREAD_SLEEP_MS));
	}
}

/* State machine implementation */

static void sensor_state_init_run(void *obj)
{
	struct sensor_state_object *ctx = (struct sensor_state_object *)obj;

	LOG_INF("Sensor SM: Initializing sensors");

	int ret = init_sensors();
	if (ret < 0) {
		LOG_ERR("Sensor SM: Initialization failed (%d)", ret);
		ctx->error_count++;
		sensor_set_state(ctx, SENSOR_MODULE_STATE_ERROR);
		return;
	}

#if defined(CONFIG_SENSOR_MODULE_WARMUP_ENABLE)
	/* Initialize warmup timing */
	ctx->sensor_init_time = k_uptime_get();
	for (int i = 0; i < SENSOR_TYPE_COUNT; i++) {
		ctx->sensor_warmup_complete[i] = false;
	}
#endif

	/* Initialize context with configured values */
	ctx->error_count = 0;
	ctx->max_retries = sensor_state_obj.max_retries;
	ctx->read_timeout_ms = sensor_state_obj.read_timeout_ms;
	ctx->last_read_time = k_uptime_get();

	LOG_INF("Sensor SM: Initialization complete");
	sensor_set_state(ctx, SENSOR_MODULE_STATE_IDLE);
}

static void sensor_state_idle_run(void *obj)
{
	struct sensor_state_object *ctx = (struct sensor_state_object *)obj;

	LOG_DBG("Sensor SM: Idle state - waiting for requests");

	/* In real implementation, this would wait for events */
	k_sleep(K_MSEC(CONFIG_SENSOR_MODULE_THREAD_SLEEP_MS));

	/* For demo purposes, automatically transition to reading after some time */
	if (k_uptime_get() - ctx->last_read_time > ctx->read_timeout_ms) {
		LOG_DBG("Sensor SM: Timeout - starting automatic read");
		sensor_set_state(ctx, SENSOR_MODULE_STATE_READING);
	}
}

static void sensor_state_reading_run(void *obj)
{
	struct sensor_state_object *ctx = (struct sensor_state_object *)obj;
	int ret;

	LOG_DBG("Sensor SM: Reading sensor data");

	/* Initialize response */
	ctx->current_data.type = SENSOR_SAMPLE_RESPONSE;
	ctx->current_data.timestamp = k_uptime_get();
	ctx->last_read_time = ctx->current_data.timestamp;

	int successful_reads = 0;

	/* Read data from all enabled sensors */
	for (int i = 0; i < SENSOR_TYPE_COUNT; i++) {
		if (!sensors[i].enabled) {
			continue;
		}

#if defined(CONFIG_SENSOR_MODULE_WARMUP_ENABLE)
		/* Check if sensor warmup is complete */
		if (!is_sensor_warmup_complete(i)) {
			LOG_DBG("Sensor SM: %s warmup not complete, skipping read",
				get_sensor_name(i));
			continue;
		}
#endif

		int ret = read_sensor_data(i, &ctx->current_data);
		if (ret == 0) {
			successful_reads++;
			update_sensor_health(&sensors[i].health, true);
			LOG_DBG("Sensor SM: %s read successful", get_sensor_name(i));

#if defined(CONFIG_CCS811_ENV_COMPENSATION)
			/* Update CCS811 environmental compensation if BME280 succeeded */
			if (i == SENSOR_TYPE_BME280) {
				handle_ccs811_env_compensation(&ctx->current_data.temperature,
							       &ctx->current_data.humidity);
			}
#endif
		} else {
			update_sensor_health(&sensors[i].health, false);
			LOG_WRN("Sensor SM: %s read failed (%d)", get_sensor_name(i), ret);
			ctx->error_count++;

#if defined(CONFIG_CCS811_ENV_COMPENSATION)
			/* Use fallback compensation if BME280 failed */
			if (i == SENSOR_TYPE_BME280) {
				handle_ccs811_fallback_compensation();
			}
#endif
		}
	}

	/* Count enabled sensors that have completed warmup */
	int enabled_sensor_count = 0;
	for (int i = 0; i < SENSOR_TYPE_COUNT; i++) {
		if (sensors[i].enabled) {
#if defined(CONFIG_SENSOR_MODULE_WARMUP_ENABLE)
			if (is_sensor_warmup_complete(i)) {
				enabled_sensor_count++;
			}
#else
			enabled_sensor_count++;
#endif
		}
	}

	/* Only publish data if at least one sensor read successfully */
	if (successful_reads == 0) {
#if defined(CONFIG_SENSOR_MODULE_WARMUP_ENABLE)
		/* Check if any sensors are still warming up */
		bool any_warming = false;
		for (int i = 0; i < SENSOR_TYPE_COUNT; i++) {
			if (sensors[i].enabled && !is_sensor_warmup_complete(i)) {
				any_warming = true;
				break;
			}
		}

		if (any_warming) {
			LOG_DBG("Sensor SM: No readings yet - sensors still warming up");
			sensor_set_state(ctx, SENSOR_MODULE_STATE_IDLE);
		} else {
			LOG_ERR("Sensor SM: All sensors failed, entering error state");
			sensor_set_state(ctx, SENSOR_MODULE_STATE_ERROR);
		}
#else
		LOG_ERR("Sensor SM: All sensors failed, entering error state");
		sensor_set_state(ctx, SENSOR_MODULE_STATE_ERROR);
#endif
	} else if (successful_reads < enabled_sensor_count) {
		LOG_WRN("Sensor SM: Partial sensor failure (%d/%d successful), but publishing "
			"available data",
			successful_reads, enabled_sensor_count);

		/* Publish partial data */
		ret = zbus_chan_pub(&sensor_chan, &ctx->current_data,
				    K_MSEC(CONFIG_SENSOR_MODULE_DATA_PUBLISH_TIMEOUT_MS));
		if (ret < 0) {
			LOG_ERR("Sensor SM: Failed to publish data (%d)", ret);
			ctx->error_count++;
		} else {
			LOG_DBG("Sensor SM: Partial data published successfully");
		}

		sensor_set_state(ctx, SENSOR_MODULE_STATE_IDLE);
	} else {
		/* All sensors successful */
		ret = zbus_chan_pub(&sensor_chan, &ctx->current_data,
				    K_MSEC(CONFIG_SENSOR_MODULE_DATA_PUBLISH_TIMEOUT_MS));
		if (ret < 0) {
			LOG_ERR("Sensor SM: Failed to publish data (%d)", ret);
			ctx->error_count++;
		} else {
			LOG_DBG("Sensor SM: Complete data published successfully");
			ctx->error_count = 0; /* Reset error count on complete success */
		}

		sensor_set_state(ctx, SENSOR_MODULE_STATE_IDLE);
	}
}

static void sensor_state_error_run(void *obj)
{
	struct sensor_state_object *ctx = (struct sensor_state_object *)obj;

	LOG_ERR("Sensor SM: Error state - attempting recovery");

	/* Wait before attempting recovery using configurable delay */
	k_sleep(K_MSEC(CONFIG_SENSOR_MODULE_RECOVERY_DELAY_MS));

	sensor_set_state(ctx, SENSOR_MODULE_STATE_RECOVERY);
}

static void sensor_state_recovery_run(void *obj)
{
	struct sensor_state_object *ctx = (struct sensor_state_object *)obj;

	LOG_INF("Sensor SM: Attempting recovery");

	/* Reset error count and try to re-initialize */
	ctx->error_count = 0;

	int ret = init_sensors();
	if (ret < 0) {
		LOG_ERR("Sensor SM: Recovery failed (%d)", ret);
		ctx->error_count++;

		/* If recovery keeps failing, stay in error state */
		if (ctx->error_count > CONFIG_SENSOR_MODULE_MAX_RECOVERY_ATTEMPTS) {
			LOG_ERR("Sensor SM: Recovery attempts exhausted");
			sensor_set_state(ctx, SENSOR_MODULE_STATE_ERROR);
		} else {
			/* Try recovery again */
			k_sleep(K_MSEC(CONFIG_SENSOR_MODULE_RECOVERY_RETRY_DELAY_MS));
		}
	} else {
		LOG_INF("Sensor SM: Recovery successful");
		sensor_set_state(ctx, SENSOR_MODULE_STATE_IDLE);
	}
}

static int init_sensors(void)
{
	int initialized_count = 0;

	for (int i = 0; i < SENSOR_TYPE_COUNT; i++) {
		if (!sensors[i].enabled) {
			LOG_INF("%s sensor disabled, skipping", get_sensor_name(i));
			continue;
		}

		/* Devices are now initialized at declaration, just check if ready */
		if (!device_is_ready(sensors[i].device)) {
			LOG_ERR("%s sensor not ready", get_sensor_name(i));
			sensors[i].enabled = false; /* Disable if not ready */
			continue;
		}

		LOG_INF("%s sensor initialized successfully", get_sensor_name(i));
		initialized_count++;
	}

	if (initialized_count == 0) {
		LOG_ERR("No sensors could be initialized");
		return -ENODEV;
	}

	LOG_INF("Sensor initialization complete: %d/%d sensors ready", initialized_count,
		SENSOR_TYPE_COUNT);
	return 0;
}

static int read_sensor_data(enum sensor_type type, struct sensor_msg *data)
{
	if (!data) {
		LOG_ERR("Invalid data pointer");
		return -EINVAL;
	}

	if (!IS_VALID_SENSOR_TYPE(type) || !sensors[type].enabled) {
		LOG_ERR("Invalid or disabled sensor type: %d", (int)type);
		return -EINVAL;
	}

	const struct device *device = sensors[type].device;
	if (!device || !device_is_ready(device)) {
		LOG_ERR("%s sensor not available", get_sensor_name(type));
		return -ENODEV;
	}

	int ret = sensor_sample_fetch(device);
	if (ret < 0) {
		LOG_ERR("Could not fetch %s sample (%d)", get_sensor_name(type), ret);
		return ret;
	}

	switch (type) {
	case SENSOR_TYPE_BME280:
		ret = sensor_channel_get(device, SENSOR_CHAN_AMBIENT_TEMP, &data->temperature);
		if (ret < 0) {
			LOG_ERR("Could not get temperature (%d)", ret);
			return ret;
		}

		ret = sensor_channel_get(device, SENSOR_CHAN_PRESS, &data->pressure);
		if (ret < 0) {
			LOG_ERR("Could not get pressure (%d)", ret);
			return ret;
		}

		ret = sensor_channel_get(device, SENSOR_CHAN_HUMIDITY, &data->humidity);
		if (ret < 0) {
			LOG_ERR("Could not get humidity (%d)", ret);
			return ret;
		}
		break;

	case SENSOR_TYPE_CCS811:
		ret = sensor_channel_get(device, SENSOR_CHAN_CO2, &data->co2);
		if (ret < 0) {
			LOG_ERR("Could not get CO2 (%d)", ret);
			return ret;
		}

		ret = sensor_channel_get(device, SENSOR_CHAN_VOC, &data->voc);
		if (ret < 0) {
			LOG_ERR("Could not get VOC (%d)", ret);
			return ret;
		}
		break;

	case SENSOR_TYPE_HM3301:
		ret = sensor_channel_get(device, SENSOR_CHAN_PM_1_0, &data->pm1_0);
		if (ret < 0) {
			LOG_ERR("Could not get PM1.0 (%d)", ret);
			return ret;
		}

		ret = sensor_channel_get(device, SENSOR_CHAN_PM_2_5, &data->pm2_5);
		if (ret < 0) {
			LOG_ERR("Could not get PM2.5 (%d)", ret);
			return ret;
		}

		ret = sensor_channel_get(device, SENSOR_CHAN_PM_10, &data->pm10);
		if (ret < 0) {
			LOG_ERR("Could not get PM10 (%d)", ret);
			return ret;
		}
		break;

	case SENSOR_TYPE_SEN0466:
		ret = sensor_channel_get(device, SENSOR_CHAN_SEN0466_CO, &data->co);
		if (ret < 0) {
			LOG_ERR("Could not get CO (%d)", ret);
			return ret;
		}

		/* Optionally read temperature if available */
		ret = sensor_channel_get(device, SENSOR_CHAN_SEN0466_TEMP,
					 &data->temperature_sen0466);
		if (ret < 0) {
			LOG_DBG("SEN0466 temperature not available (%d)", ret);
			/* Temperature reading is optional, don't fail */
		}
		break;

	default:
		LOG_ERR("Unknown sensor type: %d", type);
		return -EINVAL;
	}

	return 0;
}

#if defined(CONFIG_CCS811_ENV_COMPENSATION)
static int update_ccs811_env_data(const struct sensor_value *temp, const struct sensor_value *hum)
{
	/* Validate inputs */
	if (!temp || !hum) {
		LOG_ERR("Invalid parameters for CCS811 environmental update");
		return -EINVAL;
	}

	/* Validate CCS811 sensor */
	const struct device *ccs811_device = sensors[SENSOR_TYPE_CCS811].device;
	if (!ccs811_device || !device_is_ready(ccs811_device) ||
	    !sensors[SENSOR_TYPE_CCS811].enabled) {
		LOG_ERR("CCS811 sensor not available for environmental update");
		return -ENODEV;
	}

	/* Previous environmental values for smart updates */
	static struct sensor_value prev_temp = {0, 0};
	static struct sensor_value prev_hum = {0, 0};
	static bool env_data_initialized = false;

	bool should_update = false;

	/* Check if this is the first update */
	if (!env_data_initialized) {
		should_update = true;
		env_data_initialized = true;
		LOG_DBG("Initializing CCS811 environmental data");
	} else {
		/* Check for significant changes using configurable thresholds */
		int32_t temp_diff =
			abs(SENSOR_VALUE_TO_MICRO(temp) - SENSOR_VALUE_TO_MICRO(&prev_temp));
		int32_t hum_diff =
			abs(SENSOR_VALUE_TO_MICRO(hum) - SENSOR_VALUE_TO_MICRO(&prev_hum));

		/* Update if temperature or humidity changed beyond thresholds */
		if (temp_diff >= CONFIG_CCS811_TEMP_THRESHOLD_MICRO ||
		    hum_diff >= CONFIG_CCS811_HUM_THRESHOLD_MICRO) {
			should_update = true;
			LOG_DBG("Environmental change detected - updating CCS811");
		}
	}

	if (should_update) {
		int ret = ccs811_envdata_update(ccs811_device, temp, hum);
		if (ret < 0) {
			LOG_ERR("Could not update CCS811 environmental data (%d)", ret);
			return ret;
		}

		LOG_DBG("CCS811 env data updated: T=%d.%06d°C, H=%d.%06d%%", temp->val1, temp->val2,
			hum->val1, hum->val2);

		/* Store current values for next comparison */
		prev_temp = *temp;
		prev_hum = *hum;
	}

	return 0;
}

static void handle_ccs811_env_compensation(const struct sensor_value *temp,
					   const struct sensor_value *hum)
{
	int env_ret = update_ccs811_env_data(temp, hum);
	if (env_ret < 0) {
		LOG_ERR("Failed to update CCS811 environmental data");
	}
}

static void handle_ccs811_fallback_compensation(void)
{
	/* Use default environmental data when BME280 fails */
	struct sensor_value default_temp = {.val1 = CONFIG_CCS811_DEFAULT_TEMPERATURE, .val2 = 0};
	struct sensor_value default_hum = {.val1 = CONFIG_CCS811_DEFAULT_HUMIDITY, .val2 = 0};

	LOG_WRN("Using default environmental data: T=%d°C, H=%d%%RH",
		CONFIG_CCS811_DEFAULT_TEMPERATURE, CONFIG_CCS811_DEFAULT_HUMIDITY);

	int env_ret = update_ccs811_env_data(&default_temp, &default_hum);
	if (env_ret < 0) {
		LOG_ERR("Failed to update CCS811 environmental data with defaults");
	}
}
#endif /* CONFIG_CCS811_ENV_COMPENSATION */

int sensor_module_get_sensor_info(enum sensor_type sensor_type, struct sensor_info *sensor_info)
{
	if (!IS_VALID_SENSOR_TYPE(sensor_type) || !sensor_info) {
		LOG_ERR("Invalid parameters for sensor info request");
		return -EINVAL;
	}

	k_mutex_lock(&sensor_sm_mutex, K_FOREVER);
	*sensor_info = sensors[sensor_type];
	k_mutex_unlock(&sensor_sm_mutex);

	return 0;
}

int sensor_module_get_all_health(struct sensor_health sensor_healths[SENSOR_TYPE_COUNT])
{
	if (!sensor_healths) {
		LOG_ERR("Invalid sensor health array pointer");
		return -EINVAL;
	}

	k_mutex_lock(&sensor_sm_mutex, K_FOREVER);
	for (int i = 0; i < SENSOR_TYPE_COUNT; i++) {
		sensor_healths[i] = sensors[i].health;
	}
	k_mutex_unlock(&sensor_sm_mutex);

	return 0;
}

int sensor_module_get_health(struct sensor_health *bme280_hlth, struct sensor_health *ccs811_hlth,
			     struct sensor_health *hm3301_hlth)
{
	if (!bme280_hlth || !ccs811_hlth || !hm3301_hlth) {
		LOG_ERR("Invalid health structure pointers");
		return -EINVAL;
	}

	k_mutex_lock(&sensor_sm_mutex, K_FOREVER);
	*bme280_hlth = sensors[SENSOR_TYPE_BME280].health;
	*ccs811_hlth = sensors[SENSOR_TYPE_CCS811].health;
	*hm3301_hlth = sensors[SENSOR_TYPE_HM3301].health;
	k_mutex_unlock(&sensor_sm_mutex);

	return 0;
}

/* Helper function to update sensor health */
static void update_sensor_health(struct sensor_health *health, bool success)
{
	if (success) {
		health->success_count++;
		health->last_success_time = k_uptime_get();
		health->is_healthy = true;
	} else {
		health->failure_count++;
		/* Consider sensor unhealthy if more than threshold failure rate in recent attempts
		 */
		uint32_t total_attempts = health->success_count + health->failure_count;
		if (total_attempts == 0) {
			/* Avoid division by zero; not enough data to determine health */
			LOG_WRN("Not enough data to determine sensor health");
			return;
		}
		uint32_t failure_rate_percent =
			(uint32_t)(((uint64_t)health->failure_count * 100) / total_attempts);

		if (failure_rate_percent > CONFIG_SENSOR_MODULE_HEALTH_FAILURE_THRESHOLD_PERCENT &&
		    total_attempts >= CONFIG_SENSOR_MODULE_HEALTH_SAMPLE_SIZE) {
			health->is_healthy = false;
		}
	}
}

/* Helper function to get sensor name from type */
static const char *get_sensor_name(enum sensor_type type)
{
	switch (type) {
	case SENSOR_TYPE_BME280:
		return "BME280";
	case SENSOR_TYPE_CCS811:
		return "CCS811";
	case SENSOR_TYPE_HM3301:
		return "HM3301";
	case SENSOR_TYPE_SEN0466:
		return "SEN0466";
	default:
		return "Unknown";
	}
}

/* Helper function to set state and keep current_state field in sync */
static void sensor_set_state(struct sensor_state_object *ctx, enum sensor_module_state new_state)
{
	ctx->current_state = new_state;
	smf_set_state(SMF_CTX(ctx), &sensor_states[new_state]);
}

#if defined(CONFIG_SENSOR_MODULE_WARMUP_ENABLE)
/* Helper function to get warmup time for a sensor type */
static int64_t get_sensor_warmup_time(enum sensor_type type)
{
	switch (type) {
	case SENSOR_TYPE_BME280:
		return CONFIG_SENSOR_MODULE_BME280_WARMUP_MS;
	case SENSOR_TYPE_CCS811:
		return CONFIG_SENSOR_MODULE_CCS811_WARMUP_MS;
	case SENSOR_TYPE_HM3301:
		return CONFIG_SENSOR_MODULE_HM3301_WARMUP_MS;
	case SENSOR_TYPE_SEN0466:
		return CONFIG_SENSOR_MODULE_SEN0466_WARMUP_MS;
	default:
		return 0;
	}
}

/* Helper function to check if sensor warmup is complete */
static bool is_sensor_warmup_complete(enum sensor_type type)
{
	if (!IS_VALID_SENSOR_TYPE(type)) {
		LOG_ERR("Invalid sensor type: %d", (int)type);
		return false;
	}

	/* Check if already marked as complete */
	if (sensor_state_obj.sensor_warmup_complete[type]) {
		return true;
	}

	/* Check if warmup time has elapsed */
	int64_t elapsed_time = k_uptime_get() - sensor_state_obj.sensor_init_time;
	int64_t warmup_time = get_sensor_warmup_time(type);

	if (elapsed_time >= warmup_time) {
		/* Mark as complete and log */
		sensor_state_obj.sensor_warmup_complete[type] = true;
		LOG_INF("Sensor SM: %s warmup complete after %lld ms", get_sensor_name(type),
			elapsed_time);
		return true;
	}

	return false;
}
#endif /* CONFIG_SENSOR_MODULE_WARMUP_ENABLE */
