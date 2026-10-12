/**
 * @file usb_device_helper.h
 *
 * @brief CAuDri - Shared STM32 USB device helpers
 */

#pragma once

#include <stdint.h>

#include "stm32f4xx_hal.h"
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

// GNU ld --wrap routes the generated CDC allocator calls to these functions.
void *__wrap_USBD_static_malloc(uint32_t size);
void __wrap_USBD_static_free(void *memory);

// Called from CubeMX's preserved Suspend/Resume USER CODE blocks.
void usb_device_suspend_complete(PCD_HandleTypeDef *pcd);
void usb_device_resume_prepare(PCD_HandleTypeDef *pcd);

#ifdef __cplusplus
}
#endif
