/*
 * Copyright (c) 2025 Natalia Pluta
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include "controller_module.h"

LOG_MODULE_REGISTER(controller_shell, CONFIG_CONTROLLER_MODULE_LOG_LEVEL);

/* Global variable to control sensor data logging */
static bool sensor_logging_enabled = true;

/* Function to get the current logging state */
bool controller_shell_is_sensor_logging_enabled(void)
{
	return sensor_logging_enabled;
}

/* Shell command: enable sensor data logging */
static int cmd_sensor_log_enable(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	sensor_logging_enabled = true;
	shell_print(sh, "Sensor data logging enabled");

	return 0;
}

/* Shell command: disable sensor data logging */
static int cmd_sensor_log_disable(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	sensor_logging_enabled = false;
	shell_print(sh, "Sensor data logging disabled");

	return 0;
}

/* Shell command: show sensor logging status */
static int cmd_sensor_log_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Sensor data logging: %s", sensor_logging_enabled ? "ENABLED" : "DISABLED");

	return 0;
}

/* Shell command: start sampling */
static int cmd_start_sampling(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int ret = controller_module_start_sampling();
	if (ret == 0) {
		shell_print(sh, "Sampling started");
	} else {
		shell_error(sh, "Failed to start sampling: %d", ret);
	}

	return ret;
}

/* Shell command: stop sampling */
static int cmd_stop_sampling(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int ret = controller_module_stop_sampling();
	if (ret == 0) {
		shell_print(sh, "Sampling stopped");
	} else {
		shell_error(sh, "Failed to stop sampling: %d", ret);
	}

	return ret;
}

/* Shell command: show controller status */
static int cmd_controller_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	enum controller_module_state state = controller_module_get_state();
	const char *state_name;

	switch (state) {
	case CONTROLLER_MODULE_STATE_INIT:
		state_name = "INIT";
		break;
	case CONTROLLER_MODULE_STATE_IDLE:
		state_name = "IDLE";
		break;
	case CONTROLLER_MODULE_STATE_ACTIVE:
		state_name = "ACTIVE";
		break;
	case CONTROLLER_MODULE_STATE_ERROR:
		state_name = "ERROR";
		break;
	case CONTROLLER_MODULE_STATE_RECOVERY:
		state_name = "RECOVERY";
		break;
	default:
		state_name = "UNKNOWN";
		break;
	}

	shell_print(sh, "Controller state: %s", state_name);
	shell_print(sh, "Sensor logging: %s", sensor_logging_enabled ? "ENABLED" : "DISABLED");

	return 0;
}

/* Define sensor logging subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_sensor_log,
	SHELL_CMD(enable, NULL, "Enable sensor data logging", cmd_sensor_log_enable),
	SHELL_CMD(disable, NULL, "Disable sensor data logging", cmd_sensor_log_disable),
	SHELL_CMD(status, NULL, "Show sensor logging status", cmd_sensor_log_status),
	SHELL_SUBCMD_SET_END);

/* Define controller subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_controller, SHELL_CMD(start, NULL, "Start sensor data sampling", cmd_start_sampling),
	SHELL_CMD(stop, NULL, "Stop sensor data sampling", cmd_stop_sampling),
	SHELL_CMD(status, NULL, "Show controller status", cmd_controller_status),
	SHELL_CMD(log, &sub_sensor_log, "Sensor logging commands", NULL), SHELL_SUBCMD_SET_END);

/* Register main controller shell command */
SHELL_CMD_REGISTER(controller, &sub_controller, "Controller module commands", NULL);
