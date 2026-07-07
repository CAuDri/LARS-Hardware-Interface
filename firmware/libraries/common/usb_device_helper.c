/**
 * @file usb_device_helper.c
 *
 * @brief CAuDri - Shared STM32 USB device helpers
 *
 * STM32CubeMX occasionally generates Get_SerialNum() without initializing its
 * local UID words. This implementation lives outside the generated board files
 * and replaces only the serial callback in the generated descriptor tables.
 */

#include "usb_device_helper.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "stm32f4xx_hal.h"

#define USB_SERIAL_DESCRIPTOR_SIZE 26U
#define USB_SERIAL_FIRST_WORD_DIGITS 8U
#define USB_SERIAL_SECOND_WORD_DIGITS 4U

static uint8_t serial_descriptor[USB_SERIAL_DESCRIPTOR_SIZE] __attribute__((aligned(4))) = {
    USB_SERIAL_DESCRIPTOR_SIZE,
    USB_DESC_TYPE_STRING,
};

static uint8_t *get_serial_descriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static void write_unicode_hex(uint32_t value, uint8_t *destination, uint8_t digits);

/**
 * @brief Install the stable serial-number callback in both USB descriptor tables.
 *
 * The serial value follows STM32CubeMX's normal format: UID words zero and two
 * are added for the first eight hexadecimal digits, followed by four digits
 * from UID word one. The descriptor is prepared once before USB starts, so the
 * enumeration callback only returns an immutable buffer.
 *
 * @param full_speed_descriptors Generated Full-Speed descriptor table.
 * @param high_speed_descriptors Generated High-Speed descriptor table.
 */
void usb_serial_descriptor_install(
    USBD_DescriptorsTypeDef *full_speed_descriptors,
    USBD_DescriptorsTypeDef *high_speed_descriptors) {
    const uint32_t first_serial_word = HAL_GetUIDw0() + HAL_GetUIDw2();
    const uint32_t second_serial_word = HAL_GetUIDw1();

    write_unicode_hex(first_serial_word, &serial_descriptor[2], USB_SERIAL_FIRST_WORD_DIGITS);
    write_unicode_hex(second_serial_word, &serial_descriptor[18], USB_SERIAL_SECOND_WORD_DIGITS);

    if (full_speed_descriptors != NULL) {
        full_speed_descriptors->GetSerialStrDescriptor = get_serial_descriptor;
    }
    if (high_speed_descriptors != NULL) {
        high_speed_descriptors->GetSerialStrDescriptor = get_serial_descriptor;
    }
}

void usb_cdc_line_coding_init(usb_cdc_line_coding_t *line_coding, uint32_t bitrate) {
    if (line_coding == NULL) {
        return;
    }

    line_coding->data[0] = (uint8_t)bitrate;
    line_coding->data[1] = (uint8_t)(bitrate >> 8U);
    line_coding->data[2] = (uint8_t)(bitrate >> 16U);
    line_coding->data[3] = (uint8_t)(bitrate >> 24U);
    line_coding->data[4] = 0x00U;  // One stop bit.
    line_coding->data[5] = 0x00U;  // No parity.
    line_coding->data[6] = 0x08U;  // Eight data bits.
}

void usb_cdc_line_coding_store(usb_cdc_line_coding_t *line_coding, const uint8_t *buffer, uint16_t length) {
    if (line_coding == NULL || buffer == NULL || length < USB_CDC_LINE_CODING_SIZE) {
        return;
    }

    memcpy(line_coding->data, buffer, USB_CDC_LINE_CODING_SIZE);
}

void usb_cdc_line_coding_load(uint8_t *buffer, const usb_cdc_line_coding_t *line_coding, uint16_t length) {
    if (buffer == NULL || line_coding == NULL || length < USB_CDC_LINE_CODING_SIZE) {
        return;
    }

    memcpy(buffer, line_coding->data, USB_CDC_LINE_CODING_SIZE);
}

static uint8_t *get_serial_descriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    (void)speed;
    *length = USB_SERIAL_DESCRIPTOR_SIZE;
    return serial_descriptor;
}

static void write_unicode_hex(uint32_t value, uint8_t *destination, uint8_t digits) {
    for (uint8_t index = 0U; index < digits; index++) {
        const uint8_t nibble = (uint8_t)(value >> 28U);
        destination[index * 2U] = nibble < 10U ? (uint8_t)(nibble + '0') : (uint8_t)(nibble - 10U + 'A');
        destination[index * 2U + 1U] = 0U;
        value <<= 4U;
    }
}
