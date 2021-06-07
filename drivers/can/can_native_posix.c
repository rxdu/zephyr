/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * CANBUS driver for native_posix board. This is meant to test CANBUS
 * connectivity between host and Zephyr.
 */

#define DT_DRV_COMPAT zephyr_native_posix_can

#define LOG_MODULE_NAME canbus_posix
#define LOG_LEVEL CONFIG_CAN_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <stdio.h>

#include <kernel.h>
#include <stdbool.h>
#include <errno.h>
#include <stddef.h>

#include <net/net_pkt.h>
#include <net/net_l2.h>
#include <net/net_core.h>
#include <net/net_if.h>

#include <net/can.h>
#include <net/socket_can.h>

#include "can_native_posix_priv.h"
#include "socket_can_context.h"

#define NET_BUF_TIMEOUT K_MSEC(100)

static int read_data(struct socket_can_context *ctx, int fd)
{
	// struct net_pkt *pkt;
	int count;
    struct can_frame frame;

	count = canbus_np_read_data(fd, (void *)(&frame),
				    sizeof(frame));
	if (count <= 0) {
		return 0;
	}

	struct zcan_frame zframe;
	can_copy_frame_to_zframe(&frame, &zframe);
	// pkt = net_pkt_rx_alloc_with_buffer(ctx->iface, sizeof(zframe), AF_CAN,
	// 				   0, NET_BUF_TIMEOUT);
	// if (!pkt) {
	// 	return -ENOMEM;
	// }

	// if (net_pkt_write(pkt, (void *)(&zframe), sizeof(zframe))) {
	// 	net_pkt_unref(pkt);
	// 	return -ENOBUFS;
	// }

	// if (net_recv_data(ctx->iface, pkt) < 0) {
	// 	net_pkt_unref(pkt);
	// }

    /* send data to consumers */
    while (k_msgq_put(ctx->msgq, &zframe, K_NO_WAIT) != 0) {
        /* message queue is full: purge old data & try again */
        k_msgq_purge(ctx->msgq);
    }

	return 0;
}

static void canbus_np_rx(struct socket_can_context *ctx)
{
	LOG_DBG("Starting ZCAN RX thread");

	while (1) {
		if (ctx->iface && net_if_is_up(ctx->iface)) {
			while (!canbus_np_wait_data(ctx->dev_fd)) {
				read_data(ctx, ctx->dev_fd);
			}
		}

		k_sleep(K_MSEC(10));
	}
}

static int canbus_np_send(const struct device *dev,
			  const struct zcan_frame *msg, k_timeout_t timeout,
			  can_tx_callback_t callback_isr, void *callback_arg)
{
	struct socket_can_context *ctx = dev->data;
	int ret = -ENODEV;

	ARG_UNUSED(timeout);
	ARG_UNUSED(callback_isr);
	ARG_UNUSED(callback_arg);

	if (ctx->dev_fd > 0) {
		struct can_frame frame;

		can_copy_zframe_to_frame(msg, &frame);

		ret = canbus_np_write_data(ctx->dev_fd, &frame, sizeof(frame));
		if (ret < 0) {
			LOG_ERR("Cannot send CAN data len %d (%d)",
				frame.can_dlc, -errno);
		}
	}

	return ret < 0 ? ret : 0;
}

static int canbus_np_attach_isr(const struct device *dev, can_rx_callback_t isr,
				void *callback_arg,
				const struct zcan_filter *filter)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(isr);
	ARG_UNUSED(callback_arg);
	ARG_UNUSED(filter);

	return 0;
}

static void canbus_np_detach(const struct device *dev, int filter_nr)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(filter_nr);
}

enum can_state canbus_np_get_state(const struct device *dev,
				   struct can_bus_err_cnt *err_cnt)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(err_cnt);
	return CAN_ERROR_ACTIVE;
}

void canbus_np_register_state_change_isr(const struct device *dev,
					 can_state_change_isr_t isr)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(isr);
}

static const struct can_driver_api can_api_funcs = {
	.send = canbus_np_send,
	.attach_isr = canbus_np_attach_isr,
	.detach = canbus_np_detach,
	.get_state = canbus_np_get_state,
	.register_state_change_isr = canbus_np_register_state_change_isr
};

#ifdef CONFIG_CAN_NATIVE_POSIX_INTERFACE_1_ENABLE
K_KERNEL_STACK_DEFINE(canbus_rx_stack1,
		      CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE);
static struct k_thread rx_thread_data1;
static struct socket_can_context canbus_context_data1;

static int canbus_np1_init(const struct device *dev)
{
	struct socket_can_context *ctx = dev->data;

	ctx->if_name = CONFIG_CAN_NATIVE_POSIX_INTERFACE_1_NAME;

	ctx->dev_fd = canbus_np_iface_open(ctx->if_name);
	if (ctx->dev_fd < 0) {
		LOG_ERR("Cannot open %s (%d)", ctx->if_name, ctx->dev_fd);
	} else {
		/* Create a thread that will handle incoming data from host */
		k_thread_create(&rx_thread_data1, canbus_rx_stack1,
				K_THREAD_STACK_SIZEOF(canbus_rx_stack1),
				(k_thread_entry_t)canbus_np_rx, ctx, NULL, NULL,
				K_PRIO_COOP(14), 0, K_NO_WAIT);
	}

	return 0;
}

DEVICE_DT_INST_DEFINE(0, &canbus_np1_init, NULL,
	    (void *)&canbus_context_data1, NULL,
	    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
	    &can_api_funcs);
#endif /* CONFIG_CAN_NATIVE_POSIX_INTERFACE_1_ENABLE */

#ifdef CONFIG_CAN_NATIVE_POSIX_INTERFACE_2_ENABLE
K_KERNEL_STACK_DEFINE(canbus_rx_stack2,
		      CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE);
static struct k_thread rx_thread_data2;
static struct socket_can_context canbus_context_data2;

static int canbus_np2_init(const struct device *dev)
{
	struct socket_can_context *ctx = dev->data;

	ctx->if_name = CONFIG_CAN_NATIVE_POSIX_INTERFACE_2_NAME;

	ctx->dev_fd = canbus_np_iface_open(ctx->if_name);
	if (ctx->dev_fd < 0) {
		LOG_ERR("Cannot open %s (%d)", ctx->if_name, ctx->dev_fd);
	} else {
		/* Create a thread that will handle incoming data from host */
		k_thread_create(&rx_thread_data2, canbus_rx_stack2,
				K_THREAD_STACK_SIZEOF(canbus_rx_stack2),
				(k_thread_entry_t)canbus_np_rx, ctx, NULL, NULL,
				K_PRIO_COOP(14), 0, K_NO_WAIT);
	}

	return 0;
}

DEVICE_DT_INST_DEFINE(1, &canbus_np2_init, NULL,
	    (void *)&canbus_context_data2, NULL,
	    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
	    &can_api_funcs);
#endif /* CONFIG_CAN_NATIVE_POSIX_INTERFACE_2_ENABLE */

#if defined(CONFIG_CAN_NATIVE_POSIX_INTERFACE_1_ENABLE) ||                            \
	defined(CONFIG_CAN_NATIVE_POSIX_INTERFACE_2_ENABLE)

#if defined(CONFIG_NET_SOCKETS_CAN)

#include "socket_can_generic.h"

// #define SEND_TIMEOUT K_MSEC(100)
// #define BUF_ALLOC_TIMEOUT K_MSEC(50)

// static void socket_can_iface_init(struct net_if *iface)
// {
// 	const struct device *dev = net_if_get_device(iface);
// 	struct socket_can_context *socket_context = dev->data;

// 	socket_context->iface = iface;

// 	LOG_DBG("Init CAN interface %p dev %p", iface, dev);
// }

// static void tx_irq_callback(uint32_t error_flags, void *arg)
// {
// 	if (error_flags) {
// 		LOG_DBG("Callback! error-code: %d", error_flags);
// 	}
// }

// /* This is called by net_if.c when packet is about to be sent */
// static int socket_can_send(const struct device *dev, struct net_pkt *pkt)
// {
// 	struct socket_can_context *socket_context = dev->data;
// 	int ret;

// 	if (net_pkt_family(pkt) != AF_CAN) {
// 		return -EPFNOSUPPORT;
// 	}

// 	ret = can_send(socket_context->can_dev,
// 		       (struct zcan_frame *)pkt->frags->data, SEND_TIMEOUT,
// 		       tx_irq_callback, NULL);
// 	if (ret) {
// 		LOG_DBG("Cannot send socket CAN msg (%d)", ret);
// 	}

// 	/* If something went wrong, then we need to return negative value to
// 	 * net_if.c:net_if_tx() so that the net_pkt will get released.
// 	 */
// 	return -ret;
// }

// static int socket_can_setsockopt(const struct device *dev, void *obj, int level,
// 				 int optname, const void *optval,
// 				 socklen_t optlen)
// {
// 	struct socket_can_context *socket_context = dev->data;
// 	struct can_filter filter;

// 	if (level != SOL_CAN_RAW && optname != CAN_RAW_FILTER) {
// 		errno = EINVAL;
// 		return -1;
// 	}

// 	/* Our userspace can send either zcan_filter or can_filter struct.
// 	 * They are different sizes so we need to convert them if needed.
// 	 */
// 	if (optlen != sizeof(struct can_filter) &&
// 	    optlen != sizeof(struct zcan_filter)) {
// 		errno = EINVAL;
// 		return -1;
// 	}

// 	if (optlen == sizeof(struct zcan_filter)) {
// 		can_copy_zfilter_to_filter((struct zcan_filter *)optval,
// 					   &filter);
// 	} else {
// 		memcpy(&filter, optval, sizeof(filter));
// 	}

// 	return canbus_np_setsockopt(socket_context->dev_fd, level, optname,
// 				    &filter, sizeof(filter));
// }

// static void socket_can_close(const struct device *dev, int filter_id)
// {
// 	struct socket_can_context *socket_context = dev->data;

// 	can_detach(socket_context->can_dev, filter_id);
// }

// static struct canbus_api socket_can_api = {
// 	.iface_api.init = socket_can_iface_init,
// 	.send = socket_can_send,
// 	.close = socket_can_close,
// 	.setsockopt = socket_can_setsockopt,
// };

#ifdef CONFIG_CAN_NATIVE_POSIX_INTERFACE_1_ENABLE
CAN_DEFINE_MSGQ(socket_can_msgq1, 5);
K_KERNEL_STACK_DEFINE(rx_thread_stack1, RX_THREAD_STACK_SIZE);

static int socket_can_init_1(const struct device *dev)
{
	// const struct device *can_dev = DEVICE_DT_INST_GET(0);
	// struct socket_can_context *socket_context = dev->data;

	// LOG_DBG("Init socket CAN device %p (%s) for dev %p (%s)", dev,
	// 	dev->name, can_dev, can_dev->name);

	// socket_context->can_dev = can_dev;
    const struct device *can_dev = DEVICE_DT_GET(DT_NODELABEL(can1));
	struct socket_can_context *socket_context = dev->data;

	LOG_DBG("Init socket CAN device %p (%s) for dev %p (%s)",
		dev, dev->name, can_dev, can_dev->name);

	socket_context->can_dev = can_dev;
	socket_context->msgq = &socket_can_msgq1;

	socket_context->rx_tid =
		k_thread_create(&socket_context->rx_thread_data,
				rx_thread_stack1,
				K_KERNEL_STACK_SIZEOF(rx_thread_stack1),
				rx_thread, socket_context, NULL, NULL,
				RX_THREAD_PRIORITY, 0, K_NO_WAIT);

	return 0;
}

NET_DEVICE_INIT_INSTANCE(socket_can_native_posix_1,
			CONFIG_CAN_NATIVE_POSIX_INTERFACE_1_SOCKETCAN_NAME, 0,
			socket_can_init_1, NULL,
			&canbus_context_data1, NULL,
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &socket_can_api,
			CANBUS_RAW_L2, NET_L2_GET_CTX_TYPE(CANBUS_RAW_L2), CAN_MTU);
#endif /* CONFIG_CAN_NATIVE_POSIX_INTERFACE_1_ENABLE */

#ifdef CONFIG_CAN_NATIVE_POSIX_INTERFACE_2_ENABLE
CAN_DEFINE_MSGQ(socket_can_msgq2, 5);
K_KERNEL_STACK_DEFINE(rx_thread_stack2, RX_THREAD_STACK_SIZE);

static int socket_can_init_2(const struct device *dev)
{
	// const struct device *can_dev = DEVICE_DT_INST_GET(1);
	// struct socket_can_context *socket_context = dev->data;

	// LOG_DBG("Init socket CAN device %p (%s) for dev %p (%s)", dev,
	// 	dev->name, can_dev, can_dev->name);

	// socket_context->can_dev = can_dev;

    const struct device *can_dev = DEVICE_DT_GET(DT_NODELABEL(can1));
	struct socket_can_context *socket_context = dev->data;

	LOG_DBG("Init socket CAN device %p (%s) for dev %p (%s)",
		dev, dev->name, can_dev, can_dev->name);

	socket_context->can_dev = can_dev;
	socket_context->msgq = &socket_can_msgq2;

	socket_context->rx_tid =
		k_thread_create(&socket_context->rx_thread_data,
				rx_thread_stack2,
				K_KERNEL_STACK_SIZEOF(rx_thread_stack2),
				rx_thread, socket_context, NULL, NULL,
				RX_THREAD_PRIORITY, 0, K_NO_WAIT);

	return 0;
}

NET_DEVICE_INIT_INSTANCE(socket_can_native_posix_2,
			CONFIG_CAN_NATIVE_POSIX_INTERFACE_2_SOCKETCAN_NAME, 1,
			socket_can_init_2, NULL,
			&canbus_context_data2, NULL,
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &socket_can_api,
			CANBUS_RAW_L2, NET_L2_GET_CTX_TYPE(CANBUS_RAW_L2), CAN_MTU);
#endif /* CONFIG_CAN_NATIVE_POSIX_INTERFACE_2_ENABLE */

#endif /* CONFIG_NET_SOCKETS_CAN */

#endif /* CONFIG_CAN_NATIVE_POSIX_INTERFACE_1 */
