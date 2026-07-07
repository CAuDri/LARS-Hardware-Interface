/**
 * @file usb_device_helper.h
 *
 * @brief CAuDri - Shared STM32 USB device helpers
 */

#pragma once

#include <stdint.h>

#include "usbd_def.h"

#ifdef __cplusplus
extern "C" {
#endif

void usb_serial_descriptor_install(
    USBD_DescriptorsTypeDef *full_speed_descriptors,
    USBD_DescriptorsTypeDef *high_speed_descriptors);

#define USB_CDC_LINE_CODING_SIZE 7U

typedef struct {
    uint8_t data[USB_CDC_LINE_CODING_SIZE];
} usb_cdc_line_coding_t;

void usb_cdc_line_coding_init(usb_cdc_line_coding_t *line_coding, uint32_t bitrate);
void usb_cdc_line_coding_store(usb_cdc_line_coding_t *line_coding, const uint8_t *buffer, uint16_t length);
void usb_cdc_line_coding_load(uint8_t *buffer, const usb_cdc_line_coding_t *line_coding, uint16_t length);

#ifdef __cplusplus
}
#endif
