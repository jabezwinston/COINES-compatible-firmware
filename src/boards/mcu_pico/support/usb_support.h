/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef DEV_LOWLEVEL_H_
#define DEV_LOWLEVEL_H_

#include "usb_common.h"

// Struct in which we keep the endpoint configuration
typedef void (*usb_ep_handler)(uint8_t *buf, uint16_t len);
struct usb_endpoint_configuration {
    const struct usb_endpoint_descriptor *descriptor;
    usb_ep_handler handler;

    // Pointers to endpoint + buffer control registers
    // in the USB controller DPSRAM
    volatile uint32_t *endpoint_control;
    volatile uint32_t *buffer_control;
    volatile uint8_t *data_buffer;

    // Toggle after each packet (unless replying to a SETUP)
    uint8_t next_pid;
};

// Struct in which we keep the device configuration
struct usb_device_configuration {
    const struct usb_device_descriptor *device_descriptor;
    const struct usb_interface_descriptor *interface_descriptor;
    const struct usb_configuration_descriptor *config_descriptor;
    const unsigned char *lang_descriptor;
    const unsigned char **descriptor_strings;
    // USB num endpoints is 16
    struct usb_endpoint_configuration endpoints[USB_NUM_ENDPOINTS];
};

#define EP0_IN_ADDR  (USB_DIR_IN  | 0)
#define EP0_OUT_ADDR (USB_DIR_OUT | 0)
#define EP1_IN_ADDR  (USB_DIR_IN  | 1)
#define EP2_OUT_ADDR (USB_DIR_OUT | 2)

void usb_device_init();

void usb_start_transfer(struct usb_endpoint_configuration *ep, uint8_t *buf, uint16_t len);

void usb_send_data(uint8_t *data, uint8_t len);

int usb_receive_data(uint8_t *data, uint8_t len);

struct usb_endpoint_configuration *usb_get_endpoint_configuration(uint8_t addr);

#endif
