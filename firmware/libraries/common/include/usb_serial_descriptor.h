/**
 * @file usb_serial_descriptor.h
 *
 * @brief CAuDri - Stable STM32 USB serial-number descriptor callback
 */

#pragma once

#include "usbd_def.h"

#ifdef __cplusplus
extern "C" {
#endif

void usb_serial_descriptor_install(
    USBD_DescriptorsTypeDef *full_speed_descriptors,
    USBD_DescriptorsTypeDef *high_speed_descriptors);

#ifdef __cplusplus
}
#endif
