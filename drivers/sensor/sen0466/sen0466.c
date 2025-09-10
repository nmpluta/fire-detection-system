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

/* Temperature conversion constants */
#define SEN0466_TEMP_REFERENCE_VOLTAGE 3.0     /* Reference voltage in volts */
#define SEN0466_TEMP_ADC_RESOLUTION    1024.0  /* ADC resolution (10-bit) */
#define SEN0466_TEMP_PULLUP_RESISTOR   10000.0 /* Pull-up resistor in ohms */
#define SEN0466_TEMP_REFERENCE_TEMP_K  298.15  /* 25°C in Kelvin (273.15 + 25.0) */
#define SEN0466_TEMP_BETA_COEFFICIENT  3380.13 /* Thermistor beta coefficient */
#define SEN0466_TEMP_KELVIN_OFFSET     273.15  /* Celsius to Kelvin conversion */

/* Temperature compensation constants */
#define SEN0466_TEMP_COMP_MIN         -20.0 /* Minimum compensation temperature (°C) */
#define SEN0466_TEMP_COMP_THRESHOLD   20.0  /* Temperature compensation threshold (°C) */
#define SEN0466_TEMP_COMP_MAX         50.0  /* Maximum compensation temperature (°C) */
#define SEN0466_TEMP_COMP_COEFFICIENT 0.005 /* Temperature compensation coefficient */
#define SEN0466_TEMP_COMP_OFFSET      0.9   /* Temperature compensation offset */
#define SEN0466_TEMP_COMP_HIGH_COEFF  0.3   /* High temperature coefficient */
#define SEN0466_TEMP_COMP_HIGH_OFFSET 6.0   /* High temperature offset */

/* Sensor limits */
#define SEN0466_CO_PPM_MIN 0    /* Minimum CO concentration (ppm) */
#define SEN0466_CO_PPM_MAX 1000 /* Maximum CO concentration (ppm) */

/* Protocol constants */
#define SEN0466_EXPECTED_GAS_TYPE  0x04 /* Expected gas type for CO */
#define SEN0466_EXPECTED_DECIMALS  0    /* Expected decimal places */
#define SEN0466_SENSOR_DELAY_MS    10   /* Sensor processing delay (ms) */
#define SEN0466_CHECKSUM_START_IDX 1    /* Checksum calculation start index */
#define SEN0466_CHECKSUM_END_IDX   7    /* Checksum calculation end index */
#define SEN0466_RX_CHECKSUM_IDX    8    /* Checksum position in RX frame */
#define SEN0466_RX_HEADER_IDX      0    /* Header position in RX frame */
#define SEN0466_RX_ECHO_CMD_IDX    1    /* Echo command position in RX frame */
#define SEN0466_RX_CONC_HI_IDX     2    /* CO concentration high byte index */
#define SEN0466_RX_CONC_LO_IDX     3    /* CO concentration low byte index */
#define SEN0466_RX_GAS_TYPE_IDX    4    /* Gas type index in RX frame */
#define SEN0466_RX_DECIMALS_IDX    5    /* Decimal places index in RX frame */
#define SEN0466_RX_TEMP_HI_IDX     6    /* Temperature high byte index */
#define SEN0466_RX_TEMP_LO_IDX     7    /* Temperature low byte index */

#if defined(CONFIG_SEN0466_SAMPLE_ALL)
/* Temperature conversion macro - converts ADC reading to Celsius */
#define SEN0466_ADC_TO_CELSIUS(adc)                                                                \
	({                                                                                         \
		double v = SEN0466_TEMP_REFERENCE_VOLTAGE * (double)(adc) /                        \
			   SEN0466_TEMP_ADC_RESOLUTION;                                            \
		double rth =                                                                       \
			(v * SEN0466_TEMP_PULLUP_RESISTOR) / (SEN0466_TEMP_REFERENCE_VOLTAGE - v); \
		(1.0 / (1.0 / SEN0466_TEMP_REFERENCE_TEMP_K +                                      \
			(1.0 / SEN0466_TEMP_BETA_COEFFICIENT) *                                    \
				log(rth / SEN0466_TEMP_PULLUP_RESISTOR))) -                        \
			SEN0466_TEMP_KELVIN_OFFSET;                                                \
	})
#endif

/* function sen0466 commands to string */
static const char *sen0466_cmd_to_str(uint8_t cmd)
{
	switch (cmd) {
	case SEN0466_CMD_CHANGE_GET_METHOD:
		return "CHANGE_GET_METHOD";
	case SEN0466_CMD_GET_GAS_CONCENTRATION:
		return "GET_GAS_CONCENTRATION";
	case SEN0466_CMD_GET_TEMP:
		return "GET_TEMP";
	case SEN0466_CMD_GET_ALL_DATA:
		return "GET_ALL_DATA";
	case SEN0466_CMD_SET_THRESHOLD_ALARMS:
		return "SET_THRESHOLD_ALARMS";
	case SEN0466_CMD_IIC_AVAILABLE:
		return "IIC_AVAILABLE";
	case SEN0466_CMD_SENSOR_VOLTAGE:
		return "SENSOR_VOLTAGE";
	case SEN0466_CMD_CHANGE_IIC_ADDR:
		return "CHANGE_IIC_ADDR";
	default:
		return "UNKNOWN_COMMAND";
	}
}

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
	for (int i = SEN0466_CHECKSUM_START_IDX; i < SEN0466_CHECKSUM_END_IDX; i++) {
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
	return i2c_burst_write_dt(&cfg->i2c, 0x00, (const uint8_t *)tx, SEN0466_FRAME_LEN);
}

static int sen0466_read_frame(const struct sen0466_config *cfg, uint8_t rx[9])
{
	return i2c_burst_read_dt(&cfg->i2c, 0x00, rx, SEN0466_FRAME_LEN);
}

/* -------------- CO temperature compensation -------------- */

#if defined(CONFIG_SEN0466_TEMP_COMPENSATION)
static int sen0466_temp_compensate(struct sen0466_data *data)
{
	if (!data) {
		return -EINVAL;
	}

	double compensation_temp;
	double temp_co_ppm = data->co_ppm;

#if defined(CONFIG_SEN0466_TEMP_COMP_INTEGRATED)
	compensation_temp = data->temp_celsius;
#elif defined(CONFIG_SEN0466_TEMP_COMP_EXTERNAL)
	/* Skip compensation until application provides external temperature */
	if (!data->ext_temp_valid) {
		LOG_WRN("No valid external temperature for compensation was provided");
		LOG_WRN("Skipping temperature compensation");
		return 0;
	}
	compensation_temp = data->ext_temp_celsius;
#else
#error "No temperature compensation method defined"
#endif

	if ((compensation_temp > SEN0466_TEMP_COMP_MIN) &&
	    (compensation_temp <= SEN0466_TEMP_COMP_THRESHOLD)) {
		temp_co_ppm = temp_co_ppm / (SEN0466_TEMP_COMP_COEFFICIENT * compensation_temp +
					     SEN0466_TEMP_COMP_OFFSET);
	} else if ((compensation_temp > SEN0466_TEMP_COMP_THRESHOLD) &&
		   (compensation_temp <= SEN0466_TEMP_COMP_MAX)) {
		temp_co_ppm = temp_co_ppm / (SEN0466_TEMP_COMP_COEFFICIENT * compensation_temp +
					     SEN0466_TEMP_COMP_OFFSET) -
			      (SEN0466_TEMP_COMP_HIGH_COEFF * compensation_temp -
			       SEN0466_TEMP_COMP_HIGH_OFFSET);
	} else {
		LOG_WRN("Temperature %.2f°C is outside the sensor's operating range (%.0f°C to "
			"%.0f°C)",
			compensation_temp, SEN0466_TEMP_COMP_MIN, SEN0466_TEMP_COMP_MAX);
		LOG_WRN("Skipping temperature compensation");
		return 0;
	}

	/* Adjusting range to sensor limits */
	if (temp_co_ppm < SEN0466_CO_PPM_MIN) {
		data->co_ppm = SEN0466_CO_PPM_MIN;
	} else if (temp_co_ppm > SEN0466_CO_PPM_MAX) {
		data->co_ppm = SEN0466_CO_PPM_MAX;
	} else {
		data->co_ppm = (uint16_t)round(temp_co_ppm);
	}

	return 0;
}
#endif

/* ------------------- Zephyr sensor API ------------------- */

static int sen0466_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct sen0466_config *cfg = dev->config;
	struct sen0466_data *data = dev->data;
	struct sen0466_tx_frame tx;
	int ret;

	ARG_UNUSED(chan);

#if defined(CONFIG_SEN0466_SAMPLE_ALL)
	/* Request "all data" frame including CO concentration and temperature */
	sen0466_pack(&tx, SEN0466_CMD_GET_ALL_DATA, 0, 0, 0);
#elif defined(CONFIG_SEN0466_SAMPLE_CO)
	/* Request only CO concentration */
	sen0466_pack(&tx, SEN0466_CMD_GET_GAS_CONCENTRATION, 0, 0, 0);
#else
#error "No sampling method defined"
#endif

	ret = sen0466_write_cmd(cfg, &tx);
	if (ret) {
		LOG_ERR("Write cmd %s failed: %d", sen0466_cmd_to_str(tx.data[0]), ret);
		return ret;
	}

	/* Sleep for sensor processing time */
	k_sleep(K_MSEC(SEN0466_SENSOR_DELAY_MS));

	uint8_t rx[SEN0466_FRAME_LEN] = {0};
	ret = sen0466_read_frame(cfg, rx);
	if (ret) {
		LOG_ERR("Read frame failed: %d", ret);
		return ret;
	}

	/* Validate checksum */
	uint8_t exp = sen0466_checksum(rx);
	if (rx[SEN0466_RX_CHECKSUM_IDX] != exp) {
		LOG_ERR("Checksum mismatch: got 0x%02x exp 0x%02x", rx[SEN0466_RX_CHECKSUM_IDX],
			exp);
		return -EIO;
	}

	/* Validate header */
	if (rx[SEN0466_RX_HEADER_IDX] != SEN0466_FRAME_HEAD) {
		LOG_ERR("Invalid frame header: got 0x%02x", rx[SEN0466_RX_HEADER_IDX]);
		return -EIO;
	}

	/* Validate echo command */
	if (rx[SEN0466_RX_ECHO_CMD_IDX] != tx.data[0]) {
		LOG_ERR("Invalid echo command: got 0x%02x exp 0x%02x", rx[SEN0466_RX_ECHO_CMD_IDX],
			tx.data[0]);
		return -EIO;
	}

	/* RX Layout: [0]=0xFF [1]=echo(0x88) [2]=conc_hi [3]=conc_lo
	 *            [4]=gas_type (CO=0x04) [5]=decimal_digits
	 *            [6]=temp_hi (ADC) [7]=temp_lo (ADC) [8]=checksum
	 */

	/* Only supported gas type is CO (0x04) */
	if (rx[SEN0466_RX_GAS_TYPE_IDX] != SEN0466_EXPECTED_GAS_TYPE) {
		LOG_ERR("Unsupported gas type 0x%02x", rx[SEN0466_RX_GAS_TYPE_IDX]);
		return -EIO;
	}

	/* Only supported decimal digits for CO concentration is 0 - no decimals */
	if (rx[SEN0466_RX_DECIMALS_IDX] != SEN0466_EXPECTED_DECIMALS) {
		LOG_ERR("Unsupported decimal digits %u", rx[SEN0466_RX_DECIMALS_IDX]);
		return -EIO;
	}

	/* CO concentration (ppm) conversion */
	data->co_ppm =
		SEN0466_COMBINE_BYTES(rx[SEN0466_RX_CONC_HI_IDX], rx[SEN0466_RX_CONC_LO_IDX]);

#if defined(CONFIG_SEN0466_SAMPLE_ALL)
	/* Temperature conversion */
	uint16_t temp_adc =
		SEN0466_COMBINE_BYTES(rx[SEN0466_RX_TEMP_HI_IDX], rx[SEN0466_RX_TEMP_LO_IDX]);
	data->temp_celsius = SEN0466_ADC_TO_CELSIUS(temp_adc);
#endif

	/* Optional temperature compensation */
#if defined(CONFIG_SEN0466_TEMP_COMPENSATION)
	ret = sen0466_temp_compensate(data);
	if (ret) {
		LOG_WRN("Temperature compensation failed: %d", ret);
		return ret;
	}
#endif

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

#if defined(CONFIG_SEN0466_SAMPLE_ALL)
	/* Temperature (°C) -> full precision */
	if (chan == (enum sensor_channel)SENSOR_CHAN_SEN0466_TEMP) {
		sensor_value_from_double(val, data->temp_celsius);
		return 0;
	}
#endif

	return -ENOTSUP;
}

int sen0466_init(const struct device *dev)
{
	const struct sen0466_config *cfg = dev->config;

	if (!device_is_ready(cfg->i2c.bus)) {
		return -ENODEV;
	}

	/* Force Initiative mode */
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

#if defined(CONFIG_SEN0466_TEMP_COMP_EXTERNAL)
	/* No external temperature at startup, set valid flag to false */
	struct sen0466_data *data = dev->data;
	data->ext_temp_valid = false;
#endif

	return 0;
}

/* ---------------- SEN0466 specific API ---------------- */

#if defined(CONFIG_SEN0466_TEMP_COMP_EXTERNAL)
int sen0466_temp_update(const struct device *dev, const struct sensor_value *temperature)
{
	if ((dev == NULL) || (temperature == NULL)) {
		return -EINVAL;
	}
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	/* Update external temperature */
	struct sen0466_data *data = dev->data;
	data->ext_temp_celsius = sensor_value_to_double(temperature);
	data->ext_temp_valid = true;

	return 0;
}
#endif

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
