/**
 * @file usb_cdc_transport.h
 *
 * @brief CAuDri - Configurable USB CDC custom transport for micro-ROS
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <stm32f4xx_hal.h>
#include <usbd_cdc.h>
#include <uxr/client/transport.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_CDC_RX_BUFFER_SIZE 4096U
#define USB_CDC_RX_BUFFER_PADDING CDC_DATA_HS_MAX_PACKET_SIZE
#define USB_CDC_WRITE_RETRIES 100U
#define USB_CDC_WRITE_TIMEOUT_MS 5U
#define USB_CDC_RX_COMPLETE_FLAG 0x04U
#define USB_CDC_DMA_COMPLETE_FLAG 0x08U
#define USB_CDC_TX_COMPLETE_FLAG 0x10U

/**
 * @brief Board-specific resources for the USB CDC transport
 *
 * Both handles must be initialized before the transport is opened. The DMA
 * stream must use memory-to-memory mode with byte alignment and incrementing
 * source and destination addresses.
 */
typedef struct {
    USBD_HandleTypeDef* usb_device;
    DMA_HandleTypeDef* rx_dma;
} usb_cdc_transport_config_t;

bool usb_cdc_transport_open(struct uxrCustomTransport* transport);
bool usb_cdc_transport_close(struct uxrCustomTransport* transport);
size_t usb_cdc_transport_write(
    struct uxrCustomTransport* transport, const uint8_t* buffer, size_t length, uint8_t* error);
size_t usb_cdc_transport_read(
    struct uxrCustomTransport* transport, uint8_t* buffer, size_t length, int timeout_ms, uint8_t* error);

#ifdef __cplusplus
}
#endif
