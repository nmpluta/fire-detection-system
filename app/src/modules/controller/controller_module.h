/*
 * Copyright (c) 2025 Natalia Pluta
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONTROLLER_MODULE_H
#define CONTROLLER_MODULE_H

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

/**
 * @brief Controller module states
 *
 * This enumeration defines the main states of the controller state machine.
 * The controller coordinates all other modules and handles application logic.
 */
enum controller_module_state {
	CONTROLLER_MODULE_STATE_INIT,
	CONTROLLER_MODULE_STATE_IDLE,
	CONTROLLER_MODULE_STATE_ACTIVE,
	CONTROLLER_MODULE_STATE_ERROR,
	CONTROLLER_MODULE_STATE_RECOVERY
};

/**
 * @brief Controller module events
 *
 * Events that can trigger state transitions in the controller state machine.
 */
enum controller_module_event {
	CONTROLLER_MODULE_EVENT_INIT_COMPLETE,
	CONTROLLER_MODULE_EVENT_START_SAMPLING,
	CONTROLLER_MODULE_EVENT_STOP_SAMPLING,
	CONTROLLER_MODULE_EVENT_SENSOR_DATA_RECEIVED,
	CONTROLLER_MODULE_EVENT_ERROR_OCCURRED,
	CONTROLLER_MODULE_EVENT_RECOVERY_ATTEMPT
};

/**
 * @brief Start data sampling
 *
 * Instructs the controller to begin periodic sensor data collection.
 *
 * @return 0 on success, negative error code on failure
 */
int controller_module_start_sampling(void);

/**
 * @brief Stop data sampling
 *
 * Instructs the controller to stop periodic sensor data collection.
 *
 * @return 0 on success, negative error code on failure
 */
int controller_module_stop_sampling(void);

/**
 * @brief Get controller status
 *
 * Returns the current state of the controller.
 *
 * @return Current controller state
 */
enum controller_module_state controller_module_get_state(void);

#endif /* CONTROLLER_MODULE_H */
