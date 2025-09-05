/*
 * Copyright (c) 2025 Natalia Pluta
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor/ccs811.h>

#include <app/drivers/blink.h>

#include <app_version.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define BLINK_PERIOD_MS_STEP 100U
#define BLINK_PERIOD_MS_MAX  1000U

#define SENSORS_WARMUP_DELAY_MS 5000U
#define SENSOR_READ_INTERVAL_MS 1000U

/* CCS811 environmental compensation configuration */
#define SENSOR_VALUE_TO_MICRO(val)  ((val)->val1 * 1000000UL + (val)->val2)
#define CCS811_TEMP_THRESHOLD_MICRO 500000 /* 0.5°C in microseconds */
#define CCS811_HUM_THRESHOLD_MICRO  500000 /* 0.5% RH in microseconds */

/* Function prototypes */
static int init_sensors(const struct device **bme280, const struct device **ccs811,
			const struct device **hm3301, const struct device **blink);
static int read_bme280_data(const struct device *sensor, struct sensor_value *temp,
			    struct sensor_value *press, struct sensor_value *hum);
static int read_ccs811_data(const struct device *sensor, struct sensor_value *co2,
			    struct sensor_value *voc);
static int read_hm3301_data(const struct device *sensor, struct sensor_value *pm1,
			    struct sensor_value *pm25, struct sensor_value *pm10);

#ifdef CONFIG_CCS811_ENV_COMPENSATION
static int update_ccs811_env_data(const struct device *ccs811_sensor,
				  const struct sensor_value *temp, const struct sensor_value *hum);
static void handle_ccs811_env_compensation(const struct device *ccs811_sensor,
					   const struct sensor_value *temp,
					   const struct sensor_value *hum);
static void handle_ccs811_fallback_compensation(const struct device *ccs811_sensor);
#endif

static int init_sensors(const struct device **bme280, const struct device **ccs811,
			const struct device **hm3301, const struct device **blink)
{
	/* Initialize BME280 sensor */
	*bme280 = DEVICE_DT_GET(DT_NODELABEL(bme280));
	if (!device_is_ready(*bme280)) {
		LOG_ERR("BME280 sensor not ready");
		return -ENODEV;
	}

	/* Initialize CCS811 sensor */
	*ccs811 = DEVICE_DT_GET(DT_NODELABEL(ccs811));
	if (!device_is_ready(*ccs811)) {
		LOG_ERR("CCS811 sensor not ready");
		return -ENODEV;
	}

	/* Initialize HM3301 sensor */
	*hm3301 = DEVICE_DT_GET(DT_NODELABEL(hm3301));
	if (!device_is_ready(*hm3301)) {
		LOG_ERR("HM3301 sensor not ready");
		return -ENODEV;
	}

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

	LOG_INF("All sensors initialized successfully");
	return 0;
}

static int read_bme280_data(const struct device *sensor, struct sensor_value *temp,
			    struct sensor_value *press, struct sensor_value *hum)
{
	int ret = sensor_sample_fetch(sensor);
	if (ret < 0) {
		LOG_ERR("Could not fetch BME280 sample (%d)", ret);
		return ret;
	}

	ret = sensor_channel_get(sensor, SENSOR_CHAN_AMBIENT_TEMP, temp);
	if (ret < 0) {
		LOG_ERR("Could not get temperature (%d)", ret);
		return ret;
	}

	ret = sensor_channel_get(sensor, SENSOR_CHAN_PRESS, press);
	if (ret < 0) {
		LOG_ERR("Could not get pressure (%d)", ret);
		return ret;
	}

	ret = sensor_channel_get(sensor, SENSOR_CHAN_HUMIDITY, hum);
	if (ret < 0) {
		LOG_ERR("Could not get humidity (%d)", ret);
		return ret;
	}

	return 0;
}

static int read_ccs811_data(const struct device *sensor, struct sensor_value *co2,
			    struct sensor_value *voc)
{
	int ret = sensor_sample_fetch(sensor);
	if (ret < 0) {
		LOG_ERR("Could not fetch CCS811 sample (%d)", ret);
		return ret;
	}

	ret = sensor_channel_get(sensor, SENSOR_CHAN_CO2, co2);
	if (ret < 0) {
		LOG_ERR("Could not get CO2 (%d)", ret);
		return ret;
	}

	ret = sensor_channel_get(sensor, SENSOR_CHAN_VOC, voc);
	if (ret < 0) {
		LOG_ERR("Could not get VOC (%d)", ret);
		return ret;
	}

	return 0;
}

static int read_hm3301_data(const struct device *sensor, struct sensor_value *pm1,
			    struct sensor_value *pm25, struct sensor_value *pm10)
{
	int ret = sensor_sample_fetch(sensor);
	if (ret < 0) {
		LOG_ERR("Could not fetch HM3301 sample (%d)", ret);
		return ret;
	}

	ret = sensor_channel_get(sensor, SENSOR_CHAN_PM_1_0, pm1);
	if (ret < 0) {
		LOG_ERR("Could not get PM1.0 (%d)", ret);
		return ret;
	}

	ret = sensor_channel_get(sensor, SENSOR_CHAN_PM_2_5, pm25);
	if (ret < 0) {
		LOG_ERR("Could not get PM2.5 (%d)", ret);
		return ret;
	}

	ret = sensor_channel_get(sensor, SENSOR_CHAN_PM_10, pm10);
	if (ret < 0) {
		LOG_ERR("Could not get PM10 (%d)", ret);
		return ret;
	}

	return 0;
}

#ifdef CONFIG_CCS811_ENV_COMPENSATION
static int update_ccs811_env_data(const struct device *ccs811_sensor,
				  const struct sensor_value *temp, const struct sensor_value *hum)
{
	/* Previous environmental values for smart updates */
	static struct sensor_value prev_temp = {0, 0};
	static struct sensor_value prev_hum = {0, 0};
	static bool env_data_initialized = false;

	bool should_update = false;

	/* Check if this is the first update */
	if (!env_data_initialized) {
		should_update = true;
		env_data_initialized = true;
		LOG_INF("Initializing CCS811 environmental data");
	} else {
		/* Check for significant changes (0.5°C or 0.5% RH threshold) */
		int32_t temp_diff =
			abs(SENSOR_VALUE_TO_MICRO(temp) - SENSOR_VALUE_TO_MICRO(&prev_temp));
		int32_t hum_diff =
			abs(SENSOR_VALUE_TO_MICRO(hum) - SENSOR_VALUE_TO_MICRO(&prev_hum));

		/* Update if temperature or humidity changed beyond thresholds */
		if (temp_diff >= CCS811_TEMP_THRESHOLD_MICRO ||
		    hum_diff >= CCS811_HUM_THRESHOLD_MICRO) {
			should_update = true;
			LOG_DBG("Environmental change detected - updating CCS811");
		}
	}

	if (should_update) {
		int ret = ccs811_envdata_update(ccs811_sensor, temp, hum);
		if (ret < 0) {
			LOG_ERR("Could not update CCS811 environmental data (%d)", ret);
			return ret;
		}

		LOG_INF("CCS811 env data updated: T=%d.%06d°C, H=%d.%06d%%", temp->val1, temp->val2,
			hum->val1, hum->val2);

		/* Store current values for next comparison */
		prev_temp = *temp;
		prev_hum = *hum;
	}

	return 0;
}

static void handle_ccs811_env_compensation(const struct device *ccs811_sensor,
					   const struct sensor_value *temp,
					   const struct sensor_value *hum)
{
	int env_ret = update_ccs811_env_data(ccs811_sensor, temp, hum);
	if (env_ret < 0) {
		LOG_ERR("Failed to update CCS811 environmental data");
	}
}

static void handle_ccs811_fallback_compensation(const struct device *ccs811_sensor)
{
	/* Use default environmental data when BME280 fails */
	struct sensor_value default_temp = {.val1 = CONFIG_CCS811_DEFAULT_TEMPERATURE, .val2 = 0};
	struct sensor_value default_hum = {.val1 = CONFIG_CCS811_DEFAULT_HUMIDITY, .val2 = 0};

	LOG_WRN("Using default environmental data: T=%d°C, H=%d%%RH",
		CONFIG_CCS811_DEFAULT_TEMPERATURE, CONFIG_CCS811_DEFAULT_HUMIDITY);

	int env_ret = update_ccs811_env_data(ccs811_sensor, &default_temp, &default_hum);
	if (env_ret < 0) {
		LOG_ERR("Failed to update CCS811 environmental data with defaults");
	}
}
#endif /* CONFIG_CCS811_ENV_COMPENSATION */

int main(void)
{
	int ret;
	const struct device *bme280_sensor, *ccs811_sensor, *hm3301_sensor, *blink;
	struct sensor_value temp, press, hum, co2, voc, pm1, pm25, pm10;

	LOG_INF("Zephyr Fire Detection System %s", APP_VERSION_STRING);

	/* Wait for sensors to stabilize */
	k_sleep(K_MSEC(SENSORS_WARMUP_DELAY_MS));

	/* Initialize all sensors */
	ret = init_sensors(&bme280_sensor, &ccs811_sensor, &hm3301_sensor, &blink);
	if (ret < 0) {
		LOG_ERR("Sensor initialization failed");
		return 0;
	}

	while (1) {
		/* Read BME280 data (temperature, pressure, humidity) */
		ret = read_bme280_data(bme280_sensor, &temp, &press, &hum);
		if (ret == 0) {
			LOG_INF("BME280: Temp: %d.%06d C, Press: %d.%06d kPa, Hum: %d.%06d %%",
				temp.val1, temp.val2, press.val1, press.val2, hum.val1, hum.val2);

#ifdef CONFIG_CCS811_ENV_COMPENSATION
			/* Use BME280 data if reading was successful */
			handle_ccs811_env_compensation(ccs811_sensor, &temp, &hum);
#endif /* CONFIG_CCS811_ENV_COMPENSATION */
		} else {
			LOG_ERR("Failed to read BME280 data");

#ifdef CONFIG_CCS811_ENV_COMPENSATION
			/* Use fallback environmental data when BME280 fails */
			handle_ccs811_fallback_compensation(ccs811_sensor);
#endif /* CONFIG_CCS811_ENV_COMPENSATION */
		}

		/* Read CCS811 data (CO2 and VOC) */
		ret = read_ccs811_data(ccs811_sensor, &co2, &voc);
		if (ret == 0) {
			LOG_INF("CCS811: CO2: %d ppm, VOC: %d ppb", co2.val1, voc.val1);
		} else {
			LOG_ERR("Failed to read CCS811 data");
		}

		/* Read HM3301 data (PM1.0, PM2.5, PM10) */
		ret = read_hm3301_data(hm3301_sensor, &pm1, &pm25, &pm10);
		if (ret == 0) {
			LOG_INF("HM3301: PM1.0: %d ug/m3, PM2.5: %d ug/m3, PM10: %d ug/m3",
				pm1.val1, pm25.val1, pm10.val1);
		} else {
			LOG_ERR("Failed to read HM3301 data");
		}

		k_sleep(K_MSEC(SENSOR_READ_INTERVAL_MS));
	}

	return 0;
}
