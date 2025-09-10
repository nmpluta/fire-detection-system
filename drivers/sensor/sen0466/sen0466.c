/*
 * Copyright (c) 2025 Natalia Pluta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor/sen0466.h>
#include <math.h>

#include "sen0466.h"

LOG_MODULE_REGISTER(sen0466, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT dfrobot_sen0466

/* Data conversion macros */
#define SEN0466_COMBINE_BYTES(hi, lo) (((uint16_t)(hi) << 8) | (lo))

/* Temperature conversion macro - converts ADC reading to Celsius */
#define SEN0466_ADC_TO_CELSIUS(adc)                                                                \
	({                                                                                         \
		double v = 3.0 * (double)(adc) / 1024.0;                                           \
		double rth = (v * 10000.0) / (3.0 - v);                                            \
		(1.0 / (1.0 / (273.15 + 25.0) + (1.0 / 3380.13) * log(rth / 10000.0))) - 273.15;   \
	})

/* ------------------- protocol helpers ------------------- */

/**
 * @brief Packed structure representing a transmission frame for the SEN0466 sensor.
 */
struct __packed sen0466_tx_frame {
	uint8_t head;                      /* Always 0xFF */
	uint8_t addr;                      /* Always 0x01 for gas sensor */
	uint8_t data[SEN0466_PAYLOAD_LEN]; /* Payload (command + parameters) */
	uint8_t checksum;                  /* Checksum */
};

static uint8_t sen0466_checksum(const uint8_t *buf /* 9 bytes */)
{
	uint8_t sum = 0;
	for (int i = 1; i < 7; i++) { /* bytes 1..6 */
		sum += buf[i];
	}
	return (uint8_t)(~sum + 1);
}

static void sen0466_pack(struct sen0466_tx_frame *out, uint8_t cmd, uint8_t p1, uint8_t p2,
			 uint8_t p3)
{
	memset(out, 0, sizeof(*out));
	out->head = SEN0466_FRAME_HEAD;
	out->addr = SEN0466_FRAME_ADDR;
	out->data[0] = cmd;
	out->data[1] = p1;
	out->data[2] = p2;
	out->data[3] = p3;
	out->checksum = sen0466_checksum((const uint8_t *)out);
}

static int sen0466_write_cmd(const struct sen0466_config *cfg, const struct sen0466_tx_frame *tx)
{
	/* The Arduino lib writes a dummy reg 0x00 then the 9-byte frame. */
	return i2c_burst_write_dt(&cfg->i2c, 0x00, (const uint8_t *)tx, SEN0466_FRAME_LEN);
}

static int sen0466_read_frame(const struct sen0466_config *cfg, uint8_t rx[9])
{
	return i2c_burst_read_dt(&cfg->i2c, 0x00, rx, SEN0466_FRAME_LEN);
}

/* ------------------- Zephyr sensor API ------------------- */

static int sen0466_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct sen0466_config *cfg = dev->config;
	struct sen0466_data *data = dev->data;
	int ret;

	ARG_UNUSED(chan);

	/* Request "all data" frame (includes conc, gas type, decimals, temp_hi/lo) */
	struct sen0466_tx_frame tx;
	sen0466_pack(&tx, SEN0466_CMD_GET_ALL_DATA, 0, 0, 0);

	ret = sen0466_write_cmd(cfg, &tx);
	if (ret) {
		LOG_DBG("write GET_ALL_DATA failed: %d", ret);
		return ret;
	}

	uint8_t rx[SEN0466_FRAME_LEN] = {0};
	ret = sen0466_read_frame(cfg, rx);
	if (ret) {
		LOG_DBG("read frame failed: %d", ret);
		return ret;
	}

	/* Validate checksum */
	uint8_t exp = sen0466_checksum(rx);
	if (rx[8] != exp) {
		LOG_DBG("checksum mismatch: got 0x%02x exp 0x%02x", rx[8], exp);
		return -EIO;
	}

	/* rx layout:
	 * [0]=head(0xFF) [1]=cmd echo(0x88) [2]=conc_hi [3]=conc_lo
	 * [4]=gas_type (CO=0x04) [5]=decimal_digits
	 * [6]=temp_hi (ADC) [7]=temp_lo (ADC) [8]=checksum
	 */

	/* Only supported gas type is CO (0x04) */
	if (rx[5] != 0 || rx[4] != 0x04) {
		LOG_WRN("Unsupported gas type 0x%02x or decimal digits %u", rx[4], rx[5]);
	}

	/* Only supported decimal digits for CO concentration is 0 - no decimals */
	if (rx[5] != 0) {
		LOG_ERR("Unsupported decimal digits %u", rx[5]);
		return -EIO;
	}

	/* CO concentration (ppm) conversion */
	data->co_ppm = SEN0466_COMBINE_BYTES(rx[2], rx[3]);

	/* Temperature conversion */
	uint16_t temp_adc = SEN0466_COMBINE_BYTES(rx[6], rx[7]);
	data->temp_celsius = SEN0466_ADC_TO_CELSIUS(temp_adc);

	return 0;
}

static int sen0466_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	const struct sen0466_data *data = dev->data;

	/* CO concentration (ppm) -> integer part only */
	if (chan == (enum sensor_channel)SENSOR_CHAN_SEN0466_CO) {
		val->val1 = data->co_ppm;
		val->val2 = 0;
		return 0;
	}

	/* Temperature (°C) -> full precision */
	if (chan == (enum sensor_channel)SENSOR_CHAN_SEN0466_TEMP) {
		sensor_value_from_double(val, data->temp_celsius);
		return 0;
	}

	return -ENOTSUP;
}

int sen0466_init(const struct device *dev)
{
	const struct sen0466_config *cfg = dev->config;

	if (!device_is_ready(cfg->i2c.bus)) {
		return -ENODEV;
	}

	/* Force Initiative mode (0x03). Non-fatal if it fails. */
	struct sen0466_tx_frame tx;
	sen0466_pack(&tx, SEN0466_CMD_CHANGE_GET_METHOD, SEN0466_MODE_INITIATIVE, 0, 0);
	int ret = sen0466_write_cmd(cfg, &tx);
	if (ret) {
		LOG_WRN("failed to set initiative mode: %d", ret);
	}

	/* Optional probe read */
	uint8_t rx[SEN0466_FRAME_LEN];
	ret = sen0466_read_frame(cfg, rx);
	if (ret) {
		LOG_WRN("probe read failed: %d", ret);
	}

	return 0;
}

/* ------------------- instantiation ------------------- */

static const struct sensor_driver_api sen0466_api = {
	.sample_fetch = sen0466_sample_fetch,
	.channel_get = sen0466_channel_get,
};

#define SEN0466_DEFINE(inst)                                                                       \
	static struct sen0466_data sen0466_data_##inst;                                            \
	static const struct sen0466_config sen0466_config_##inst = {                               \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, sen0466_init, NULL, &sen0466_data_##inst,                      \
			      &sen0466_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,    \
			      &sen0466_api);

DT_INST_FOREACH_STATUS_OKAY(SEN0466_DEFINE)
