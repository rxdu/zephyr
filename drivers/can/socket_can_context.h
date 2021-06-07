/*
 * Copyright (c) 2019 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

/* CANBUS related functions that are generic in all the drivers. */

#ifndef ZEPHYR_DRIVERS_CAN_SOCKET_CAN_CONTEXT_H_
#define ZEPHYR_DRIVERS_CAN_SOCKET_CAN_CONTEXT_H_

#include <net/socket_can.h>

struct socket_can_context {
	const struct device *can_dev;
	struct net_if *iface;
	struct k_msgq *msgq;

    /* for can device on native posix */
    const char *if_name;
	int dev_fd;

	/* TODO: remove the thread and push data to net directly from rx isr */
	k_tid_t rx_tid;
	struct k_thread rx_thread_data;
};

#endif /* ZEPHYR_DRIVERS_CAN_SOCKET_CAN_CONTEXT_H_ */
