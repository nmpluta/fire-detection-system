/*
 * Copyright (c) 2025 Natalia Pluta
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_FATAL_ERROR_H
#define APP_FATAL_ERROR_H

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Send fatal error signal
 *
 * This macro logs a fatal error and triggers a system halt.
 * It's used throughout the application modules when unrecoverable
 * errors occur.
 */
#define SEND_FATAL_ERROR()                                                                         \
	do {                                                                                       \
		LOG_ERR("FATAL ERROR at %s:%d", __FILE__, __LINE__);                               \
		k_panic();                                                                         \
	} while (0)

/**
 * @brief Send fatal error signal with watchdog timeout
 *
 * This macro logs a fatal error due to watchdog timeout and triggers
 * a system halt.
 */
#define SEND_FATAL_ERROR_WATCHDOG_TIMEOUT()                                                        \
	do {                                                                                       \
		LOG_ERR("FATAL ERROR: Watchdog timeout at %s:%d", __FILE__, __LINE__);             \
		k_panic();                                                                         \
	} while (0)

#ifdef __cplusplus
}
#endif

#endif /* APP_FATAL_ERROR_H */
