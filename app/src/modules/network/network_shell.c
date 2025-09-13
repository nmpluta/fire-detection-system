/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <modem/lte_lc.h>
#include <modem/modem_info.h>

#include "network_module.h"

LOG_MODULE_DECLARE(network_module, CONFIG_NETWORK_MODULE_LOG_LEVEL);

/**
 * @brief Display current connectivity status
 */
static int cmd_connectivity_status(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "=== Connectivity Status ===");

	/* Get network registration status */
	enum lte_lc_nw_reg_status reg_status;
	int ret = lte_lc_nw_reg_status_get(&reg_status);
	if (ret == 0) {
		shell_print(shell, "Registration: %s",
			    reg_status == LTE_LC_NW_REG_REGISTERED_HOME       ? "Home network"
			    : reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING  ? "Roaming"
			    : reg_status == LTE_LC_NW_REG_SEARCHING           ? "Searching"
			    : reg_status == LTE_LC_NW_REG_NOT_REGISTERED      ? "Not registered"
			    : reg_status == LTE_LC_NW_REG_REGISTRATION_DENIED ? "Denied"
			    : reg_status == LTE_LC_NW_REG_UNKNOWN             ? "Unknown"
									      : "Other");
	} else {
		shell_print(shell, "Registration: Failed to get status (%d)", ret);
	}

	/* Get functional mode */
	enum lte_lc_func_mode func_mode;
	ret = lte_lc_func_mode_get(&func_mode);
	if (ret == 0) {
		shell_print(shell, "Functional mode: %s",
			    func_mode == LTE_LC_FUNC_MODE_NORMAL            ? "Normal"
			    : func_mode == LTE_LC_FUNC_MODE_OFFLINE         ? "Offline"
			    : func_mode == LTE_LC_FUNC_MODE_POWER_OFF       ? "Power off"
			    : func_mode == LTE_LC_FUNC_MODE_ACTIVATE_LTE    ? "Activating LTE"
			    : func_mode == LTE_LC_FUNC_MODE_ACTIVATE_GNSS   ? "Activating GNSS"
			    : func_mode == LTE_LC_FUNC_MODE_DEACTIVATE_LTE  ? "Deactivating LTE"
			    : func_mode == LTE_LC_FUNC_MODE_DEACTIVATE_GNSS ? "Deactivating GNSS"
									    : "Unknown");
	} else {
		shell_print(shell, "Functional mode: Failed to get (%d)", ret);
	}

	/* Try to get basic modem info directly */
	char operator_name[32];
	ret = modem_info_string_get(MODEM_INFO_OPERATOR, operator_name, sizeof(operator_name));
	if (ret > 0) {
		shell_print(shell, "Operator: %s", operator_name);
	} else {
		shell_print(shell, "Operator: Not available (%d)", ret);
	}

	return 0;
}

/**
 * @brief Display signal quality metrics
 */
static int cmd_signal_status(const struct shell *shell, size_t argc, char **argv)
{
	uint16_t rsrp_raw = 0, band = 0;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "=== Signal Quality ===");

	/* Get RSRP directly */
	ret = modem_info_short_get(MODEM_INFO_RSRP, &rsrp_raw);
	if (ret >= 0) {
		if (rsrp_raw == 255 || rsrp_raw == 0) {
			shell_print(shell, "RSRP: Not available (raw: %d)", rsrp_raw);
		} else {
			int rsrp_dbm = RSRP_IDX_TO_DBM((int)rsrp_raw);
			shell_print(shell, "RSRP: %d dBm (index: %d)", rsrp_dbm, rsrp_raw);
			shell_print(shell, "Signal strength: %s, %d dBm",
				    rsrp_dbm > -90    ? "Excellent"
				    : rsrp_dbm > -105 ? "Good"
				    : rsrp_dbm > -115 ? "Fair"
						      : "Poor",
				    rsrp_dbm);
		}
	} else {
		shell_print(shell, "RSRP: Failed to get (%d)", ret);
	}

	/* Get current band */
	ret = modem_info_short_get(MODEM_INFO_CUR_BAND, &band);
	if (ret >= 0) {
		if (band == 0) {
			shell_print(shell, "Current Band: Not connected");
		} else {
			shell_print(shell, "Current Band: %d", band);
		}
	} else {
		shell_print(shell, "Current Band: Failed to get (%d)", ret);
	}

	return 0;
}

/**
 * @brief Display modem information
 */
static int cmd_modem_info(const struct shell *shell, size_t argc, char **argv)
{
	char fw_version[MODEM_INFO_FWVER_SIZE];
	char imei[32];
	char imsi[32];
	char iccid[32];
	uint16_t battery = 0;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "=== Modem Information ===");

	/* Get firmware version */
	ret = modem_info_string_get(MODEM_INFO_FW_VERSION, fw_version, sizeof(fw_version));
	if (ret > 0) {
		shell_print(shell, "Firmware: %s", fw_version);
	} else {
		shell_print(shell, "Firmware: Not available");
	}

	/* Get IMEI */
	ret = modem_info_string_get(MODEM_INFO_IMEI, imei, sizeof(imei));
	if (ret > 0) {
		shell_print(shell, "IMEI: %s", imei);
	} else {
		shell_print(shell, "IMEI: Not available");
	}

	/* Get IMSI */
	ret = modem_info_string_get(MODEM_INFO_IMSI, imsi, sizeof(imsi));
	if (ret > 0) {
		shell_print(shell, "IMSI: %s", imsi);
	} else {
		shell_print(shell, "IMSI: Not available");
	}

	/* Get ICCID */
	ret = modem_info_string_get(MODEM_INFO_ICCID, iccid, sizeof(iccid));
	if (ret > 0) {
		shell_print(shell, "ICCID: %s", iccid);
	} else {
		shell_print(shell, "ICCID: Not available");
	}

	/* Get battery voltage */
	ret = modem_info_short_get(MODEM_INFO_BATTERY, &battery);
	if (ret >= 0) {
		shell_print(shell, "Battery: %d mV", battery);
	} else {
		shell_print(shell, "Battery: Not available");
	}

	return 0;
}

/**
 * @brief Display network information
 */
static int cmd_network_info(const struct shell *shell, size_t argc, char **argv)
{
	char operator_name[32];
	char apn[64];
	uint16_t mcc = 0, mnc = 0, tac = 0;
	char ip_addr[32];
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "=== Network Information ===");

	/* Get operator name */
	ret = modem_info_string_get(MODEM_INFO_OPERATOR, operator_name, sizeof(operator_name));
	if (ret > 0) {
		shell_print(shell, "Operator: %s", operator_name);
	} else {
		shell_print(shell, "Operator: Not available");
	}

	/* Get MCC */
	ret = modem_info_short_get(MODEM_INFO_MCC, &mcc);
	if (ret >= 0) {
		shell_print(shell, "MCC: %d", mcc);
	}

	/* Get MNC */
	ret = modem_info_short_get(MODEM_INFO_MNC, &mnc);
	if (ret >= 0) {
		shell_print(shell, "MNC: %d", mnc);
	}

	/* Get Tracking Area Code */
	ret = modem_info_short_get(MODEM_INFO_AREA_CODE, &tac);
	if (ret >= 0) {
		shell_print(shell, "TAC: %d", tac);
	}

	/* Get IP address */
	ret = modem_info_string_get(MODEM_INFO_IP_ADDRESS, ip_addr, sizeof(ip_addr));
	if (ret > 0) {
		shell_print(shell, "IP Address: %s", ip_addr);
	} else {
		shell_print(shell, "IP Address: Not available");
	}

	/* Get APN */
	ret = modem_info_string_get(MODEM_INFO_APN, apn, sizeof(apn));
	if (ret > 0) {
		shell_print(shell, "APN: %s", apn);
	} else {
		shell_print(shell, "APN: Not available");
	}

	return 0;
}

static int cmd_connect(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int err;
	const struct network_msg msg = {
		.type = NETWORK_CONNECT_REQUEST,
	};

	LOG_INF("Sending network connect request");
	err = zbus_chan_pub(&network_chan, &msg, K_SECONDS(1));
	if (err) {
		(void)shell_print(sh, "zbus_chan_pub, error: %d", err);
		return 1;
	}

	return 0;
}

static int cmd_disconnect(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int err;
	const struct network_msg msg = {
		.type = NETWORK_DISCONNECT_REQUEST,
	};

	LOG_INF("Sending network disconnect request");
	err = zbus_chan_pub(&network_chan, &msg, K_SECONDS(1));
	if (err) {
		(void)shell_print(sh, "zbus_chan_pub, error: %d", err);
		return 1;
	}

	return 0;
}

static int cmd_search_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int err;
	const struct network_msg msg = {
		.type = NETWORK_SEARCH_STOP_REQUEST,
	};

	LOG_INF("Sending network search stop request");
	err = zbus_chan_pub(&network_chan, &msg, K_SECONDS(1));
	if (err) {
		(void)shell_print(sh, "zbus_chan_pub, error: %d", err);
		return 1;
	}

	return 0;
}

static int cmd_quality_sample(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int err;
	const struct network_msg msg = {
		.type = NETWORK_QUALITY_SAMPLE_REQUEST,
	};

	LOG_INF("Sending network quality sample request");
	err = zbus_chan_pub(&network_chan, &msg, K_SECONDS(1));
	if (err) {
		(void)shell_print(sh, "zbus_chan_pub, error: %d", err);
		return 1;
	}

	return 0;
}

static int cmd_system_mode_get(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int err;
	const struct network_msg msg = {
		.type = NETWORK_SYSTEM_MODE_REQUEST,
	};

	LOG_INF("Sending system mode request");
	err = zbus_chan_pub(&network_chan, &msg, K_SECONDS(1));
	if (err) {
		(void)shell_print(sh, "zbus_chan_pub, error: %d", err);
		return 1;
	}

	return 0;
}

static int cmd_system_mode_set_ltem(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int err;
	const struct network_msg msg = {
		.type = NETWORK_SYSTEM_MODE_SET_LTEM_REQUEST,
	};

	LOG_INF("Sending system mode set LTE-M request");
	err = zbus_chan_pub(&network_chan, &msg, K_SECONDS(1));
	if (err) {
		(void)shell_print(sh, "zbus_chan_pub, error: %d", err);
		return 1;
	}

	return 0;
}

static int cmd_system_mode_set_nbiot(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int err;
	const struct network_msg msg = {
		.type = NETWORK_SYSTEM_MODE_SET_NBIOT_REQUEST,
	};

	LOG_INF("Sending system mode set NB-IoT request");
	err = zbus_chan_pub(&network_chan, &msg, K_SECONDS(1));
	if (err) {
		(void)shell_print(sh, "zbus_chan_pub, error: %d", err);
		return 1;
	}

	return 0;
}

static int cmd_system_mode_set_ltem_nbiot(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int err;
	const struct network_msg msg = {
		.type = NETWORK_SYSTEM_MODE_SET_LTEM_NBIOT_REQUEST,
	};

	LOG_INF("Sending system mode set LTE-M+NB-IoT request");
	err = zbus_chan_pub(&network_chan, &msg, K_SECONDS(1));
	if (err) {
		(void)shell_print(sh, "zbus_chan_pub, error: %d", err);
		return 1;
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_cmds, SHELL_CMD(connect, NULL, "Connect to LTE", cmd_connect),
	SHELL_CMD(disconnect, NULL, "Disconnect from LTE", cmd_disconnect),
	SHELL_CMD(search_stop, NULL, "Stop network search", cmd_search_stop),
	SHELL_CMD(quality, NULL, "Request network quality sample", cmd_quality_sample),
	SHELL_CMD(sysmode_get, NULL, "Get current system mode", cmd_system_mode_get),
	SHELL_CMD(sysmode_ltem, NULL, "Set system mode to LTE-M only", cmd_system_mode_set_ltem),
	SHELL_CMD(sysmode_nbiot, NULL, "Set system mode to NB-IoT only", cmd_system_mode_set_nbiot),
	SHELL_CMD(sysmode_dual, NULL, "Set system mode to LTE-M+NB-IoT",
		  cmd_system_mode_set_ltem_nbiot),
	SHELL_CMD(status, NULL, "Show connectivity status", cmd_connectivity_status),
	SHELL_CMD(signal, NULL, "Show signal quality", cmd_signal_status),
	SHELL_CMD(modem, NULL, "Show modem information", cmd_modem_info),
	SHELL_CMD(network, NULL, "Show network information", cmd_network_info),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(network_module, &sub_cmds, "Network module commands", NULL);
