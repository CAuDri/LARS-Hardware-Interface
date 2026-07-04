/**
 * @file usb_cdc_transport.c
 *
 * @brief CAuDri - DMA-backed USB CDC custom transport for micro-ROS
 *
 * USB reception writes directly into a fixed ring buffer. Reads use DMA to
 * copy contiguous data into the micro-ROS buffer.
 */

#include "usb_cdc_transport.h"

#include <string.h>

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "logger.h"
#include "task.h"

enum {
    TRANSPORT_ERROR_NONE = 0,
    TRANSPORT_ERROR_INVALID_CONFIG = 1,
    TRANSPORT_ERROR_TIMEOUT = 2,
    TRANSPORT_ERROR_IO = 3,
    TRANSPORT_ERROR_BUSY = 4,
};

static usb_cdc_transport_config_t* active_config = NULL;
static USBD_CDC_ItfTypeDef* active_interface = NULL;
static USBD_CDC_ItfTypeDef previous_interface = {0};
static uint8_t* previous_rx_buffer = NULL;
static size_t active_packet_size = 0U;

/* One packet of padding lets USB write a complete packet at the end of the
 * logical ring without crossing the physical array boundary. */
static uint8_t rx_buffer[USB_CDC_RX_BUFFER_SIZE + USB_CDC_RX_BUFFER_PADDING] __attribute__((aligned(4)));

static volatile size_t rx_head = 0;
static volatile size_t rx_tail = 0;
static volatile size_t rx_wrap = 0;
static volatile bool rx_full = false;
static volatile bool rx_paused = false;
static volatile bool dma_failed = false;

static osThreadId_t receive_thread = NULL;
static osThreadId_t transmit_thread = NULL;

static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};

static bool validate_config(const usb_cdc_transport_config_t* config);
static USBD_CDC_HandleTypeDef* get_cdc_handle(const USBD_HandleTypeDef* usb_device);
static bool dma_interrupt_enabled(const DMA_HandleTypeDef* dma);
static uint32_t get_empty_read_timeout(int timeout_ms);
static uint32_t get_dma_read_timeout(int timeout_ms);
static bool reception_has_packet_space(void);
static void resume_reception_if_paused(void);
static void reset_active_transport(usb_cdc_transport_config_t* config);
static int8_t cdc_control(uint8_t command, uint8_t* buffer, uint16_t length);
static int8_t cdc_receive_complete(uint8_t* buffer, uint32_t* length);
static int8_t cdc_transmit_complete(uint8_t* buffer, uint32_t* length, uint8_t endpoint);
static void dma_receive_complete(DMA_HandleTypeDef* dma);
static void dma_receive_error(DMA_HandleTypeDef* dma);

/**
 * @brief Take ownership of one configured USB CDC interface.
 *
 * The function installs the transport's CDC callbacks, registers the DMA
 * completion callbacks and arms USB reception into the private ring buffer.
 * STM32's CDC callbacks do not carry a user context, therefore only one USB
 * CDC transport may be active at a time.
 *
 * @param transport micro-XRCE-DDS transport containing a
 * usb_cdc_transport_config_t in its args member.
 * @return true when the interface is ready, otherwise false.
 */
bool usb_cdc_transport_open(struct uxrCustomTransport* transport) {
    if (transport == NULL || !validate_config((usb_cdc_transport_config_t*)transport->args)) {
        LogWarning("micro-ROS USB CDC Transport: Invalid configuration");
        return false;
    }

    usb_cdc_transport_config_t* config = (usb_cdc_transport_config_t*)transport->args;
    if (active_config != NULL) {
        return active_config == config;
    }

    if (config->usb_device->dev_state != USBD_STATE_CONFIGURED) {
        return false;
    }

    USBD_CDC_HandleTypeDef* cdc = get_cdc_handle(config->usb_device);
    if (cdc == NULL) {
        LogWarning("micro-ROS USB CDC Transport: USB CDC class data is not available");
        return false;
    }

    USBD_CDC_ItfTypeDef* interface = (USBD_CDC_ItfTypeDef*)config->usb_device->pUserData[config->usb_device->classId];
    if (interface == NULL) {
        LogWarning("micro-ROS USB CDC Transport: CDC interface callbacks are not registered");
        return false;
    }

    const uint8_t endpoint = CDC_OUT_EP & 0x0FU;
    size_t packet_size = config->usb_device->ep_out[endpoint].maxpacket;
    if (packet_size == 0U) {
        packet_size = config->usb_device->dev_speed == USBD_SPEED_HIGH ? CDC_DATA_HS_MAX_PACKET_SIZE : CDC_DATA_FS_MAX_PACKET_SIZE;
    }
    if (packet_size > USB_CDC_RX_BUFFER_PADDING) {
        LogWarning("micro-ROS USB CDC Transport: USB packet size does not fit RX padding");
        return false;
    }

    previous_interface = *interface;
    previous_rx_buffer = cdc->RxBuffer;

    if (HAL_DMA_RegisterCallback(config->rx_dma, HAL_DMA_XFER_CPLT_CB_ID, dma_receive_complete) != HAL_OK) {
        LogWarning("micro-ROS USB CDC Transport: Failed to register DMA complete callback");
        return false;
    }
    if (HAL_DMA_RegisterCallback(config->rx_dma, HAL_DMA_XFER_ERROR_CB_ID, dma_receive_error) != HAL_OK) {
        (void)HAL_DMA_UnRegisterCallback(config->rx_dma, HAL_DMA_XFER_CPLT_CB_ID);
        LogWarning("micro-ROS USB CDC Transport: Failed to register DMA error callback");
        return false;
    }

    rx_head = 0;
    rx_tail = 0;
    rx_wrap = USB_CDC_RX_BUFFER_SIZE;
    rx_full = false;
    rx_paused = false;
    dma_failed = false;
    receive_thread = NULL;
    transmit_thread = NULL;
    active_config = config;
    active_interface = interface;
    active_packet_size = packet_size;

    active_interface->Control = cdc_control;
    active_interface->Receive = cdc_receive_complete;
    active_interface->TransmitCplt = cdc_transmit_complete;
    USBD_CDC_SetRxBuffer(config->usb_device, rx_buffer);
    if (USBD_CDC_ReceivePacket(config->usb_device) != USBD_OK) {
        reset_active_transport(config);
        LogWarning("micro-ROS USB CDC Transport: Failed to arm USB reception");
        return false;
    }
    return true;
}

/**
 * @brief Release the active USB CDC interface and restore its old callbacks.
 * @param transport Transport previously passed to usb_cdc_transport_open().
 * @return true on success, false when another transport owns the interface.
 */
bool usb_cdc_transport_close(struct uxrCustomTransport* transport) {
    if (transport == NULL || transport->args == NULL) {
        return false;
    }

    usb_cdc_transport_config_t* config = (usb_cdc_transport_config_t*)transport->args;
    if (active_config == NULL) {
        return true;
    }
    if (active_config != config) {
        return false;
    }

    reset_active_transport(config);
    transmit_thread = NULL;
    return true;
}

static void reset_active_transport(usb_cdc_transport_config_t* config) {
    if (active_config == NULL || active_config != config || active_interface == NULL) {
        return;
    }

    active_interface->Control = previous_interface.Control;
    active_interface->Receive = previous_interface.Receive;
    active_interface->TransmitCplt = previous_interface.TransmitCplt;
    if (previous_rx_buffer != NULL) {
        USBD_CDC_SetRxBuffer(config->usb_device, previous_rx_buffer);
        (void)USBD_CDC_ReceivePacket(config->usb_device);
    }

    if (config->rx_dma->State == HAL_DMA_STATE_READY) {
        (void)HAL_DMA_UnRegisterCallback(config->rx_dma, HAL_DMA_XFER_CPLT_CB_ID);
        (void)HAL_DMA_UnRegisterCallback(config->rx_dma, HAL_DMA_XFER_ERROR_CB_ID);
    }

    active_config = NULL;
    active_interface = NULL;
    active_packet_size = 0U;
    receive_thread = NULL;
    transmit_thread = NULL;
}

/**
 * @brief Send one micro-XRCE-DDS byte sequence over USB CDC.
 *
 * USB may temporarily report BUSY. The operation retries a bounded number of
 * times and then waits for the CDC transmit-complete callback.
 *
 * @param transport Open custom transport.
 * @param buffer Bytes to transmit.
 * @param length Number of bytes to transmit.
 * @param error Receives a transport-specific error code.
 * @return Number of transmitted bytes, or zero on failure.
 */
size_t usb_cdc_transport_write(struct uxrCustomTransport* transport, const uint8_t* buffer, size_t length, uint8_t* error) {
    if (error != NULL) {
        *error = TRANSPORT_ERROR_NONE;
    }
    if (transport == NULL || transport->args != active_config || buffer == NULL || error == NULL) {
        if (error != NULL)
            *error = TRANSPORT_ERROR_INVALID_CONFIG;
        return 0;
    }

    usb_cdc_transport_config_t* config = active_config;
    transmit_thread = osThreadGetId();
    (void)osThreadFlagsClear(USB_CDC_TX_COMPLETE_FLAG);
    USBD_CDC_SetTxBuffer(config->usb_device, (uint8_t*)buffer, (uint32_t)length);

    for (uint32_t attempt = 0; attempt < USB_CDC_WRITE_RETRIES; ++attempt) {
        const uint8_t result = USBD_CDC_TransmitPacket(config->usb_device);
        if (result == USBD_FAIL) {
            *error = TRANSPORT_ERROR_IO;
            return 0;
        }
        if (result == USBD_OK) {
            const uint32_t flags = osThreadFlagsWait(USB_CDC_TX_COMPLETE_FLAG, osFlagsWaitAny, USB_CDC_WRITE_TIMEOUT_MS);
            if ((flags & osFlagsError) == 0U && (flags & USB_CDC_TX_COMPLETE_FLAG) != 0U) {
                return length;
            }
            *error = TRANSPORT_ERROR_TIMEOUT;
            return 0;
        }
    }

    *error = TRANSPORT_ERROR_BUSY;
    return 0;
}

/**
 * @brief Read available bytes from the USB CDC ring buffer.
 *
 * The read waits only when the ring is empty. Data is copied with the
 * configured memory-to-memory DMA stream; a wrapped transfer overlaps DMA and
 * a short CPU copy to avoid starting DMA twice.
 *
 * @param transport Open custom transport.
 * @param buffer Destination supplied by micro-XRCE-DDS.
 * @param length Maximum number of bytes to read.
 * @param timeout_ms Maximum wait in milliseconds; zero is non-blocking.
 * @param error Receives a transport-specific error code.
 * @return Number of bytes copied into buffer, or zero when no data was read.
 */
size_t usb_cdc_transport_read(struct uxrCustomTransport* transport, uint8_t* buffer, size_t length, int timeout_ms, uint8_t* error) {
    if (error != NULL) {
        *error = TRANSPORT_ERROR_NONE;
    }
    if (transport == NULL || transport->args != active_config || buffer == NULL || error == NULL) {
        if (error != NULL)
            *error = TRANSPORT_ERROR_INVALID_CONFIG;
        return 0;
    }
    if (length == 0U) {
        return 0;
    }

    receive_thread = osThreadGetId();
    if (rx_head == rx_tail && !rx_full) {
        /* Discard an old notification, then recheck the ring before blocking.
         * A packet arriving after the clear either changes rx_head or sets a
         * fresh flag, so this sequence cannot lose the wakeup. */
        (void)osThreadFlagsClear(USB_CDC_RX_COMPLETE_FLAG);
        if (rx_head == rx_tail && !rx_full) {
            const uint32_t timeout = get_empty_read_timeout(timeout_ms);
            const uint32_t flags = osThreadFlagsWait(USB_CDC_RX_COMPLETE_FLAG, osFlagsWaitAny, timeout);
            if ((flags & osFlagsError) != 0U || (flags & USB_CDC_RX_COMPLETE_FLAG) == 0U) {
                *error = TRANSPORT_ERROR_TIMEOUT;
                return 0;
            }
        }
    }

    taskENTER_CRITICAL();
    const size_t head = rx_head;
    size_t tail = rx_tail;
    const size_t wrap = rx_wrap;
    const bool full = rx_full;
    taskEXIT_CRITICAL();

    const bool wrapped = full || head < tail;
    const size_t available = wrapped ? wrap - tail + head : head - tail;
    const size_t to_read = length < available ? length : available;
    if (to_read == 0U) {
        *error = TRANSPORT_ERROR_IO;
        return 0;
    }

    /* A wrapped read has two physical sections. Start DMA on the section at
     * the beginning of the ring, then copy the tail section on the CPU while
     * DMA runs. This overlaps both transfers without needing a second DMA IRQ. */
    size_t cpu_length = 0U;
    size_t dma_tail = tail;
    size_t dma_length = to_read;
    if (wrapped && wrap - tail < to_read) {
        cpu_length = wrap - tail;
        dma_tail = 0U;
        dma_length = to_read - cpu_length;
    }

    dma_failed = false;
    (void)osThreadFlagsClear(USB_CDC_DMA_COMPLETE_FLAG);
    if (HAL_DMA_Start_IT(active_config->rx_dma,
                         (uint32_t)(uintptr_t)&rx_buffer[dma_tail],
                         (uint32_t)(uintptr_t)(buffer + cpu_length),
                         (uint32_t)dma_length) != HAL_OK) {
        *error = TRANSPORT_ERROR_IO;
        return 0;
    }

    if (cpu_length > 0U) {
        memcpy(buffer, &rx_buffer[tail], cpu_length);
    }

    const uint32_t timeout = get_dma_read_timeout(timeout_ms);
    const uint32_t flags = osThreadFlagsWait(USB_CDC_DMA_COMPLETE_FLAG, osFlagsWaitAny, timeout);
    if ((flags & osFlagsError) != 0U || (flags & USB_CDC_DMA_COMPLETE_FLAG) == 0U || dma_failed) {
        *error = dma_failed ? TRANSPORT_ERROR_IO : TRANSPORT_ERROR_TIMEOUT;
        if (!dma_failed) {
            (void)HAL_DMA_Abort(active_config->rx_dma);
        }
        taskENTER_CRITICAL();
        rx_tail = cpu_length > 0U ? 0U : tail;
        rx_full = false;
        taskEXIT_CRITICAL();
        resume_reception_if_paused();
        return cpu_length;
    }

    taskENTER_CRITICAL();
    rx_tail = dma_tail + dma_length;
    rx_full = false;
    taskEXIT_CRITICAL();
    resume_reception_if_paused();
    return to_read;
}

static bool validate_config(const usb_cdc_transport_config_t* config) {
    if (config == NULL) {
        LogWarning("micro-ROS USB CDC Transport: Missing transport configuration");
        return false;
    }
    if (config->usb_device == NULL) {
        LogWarning("micro-ROS USB CDC Transport: Missing USB device handle");
        return false;
    }
    if (config->rx_dma == NULL || config->rx_dma->Instance == NULL) {
        LogWarning("micro-ROS USB CDC Transport: Missing RX DMA handle");
        return false;
    }

    if (config->rx_dma->Init.Direction != DMA_MEMORY_TO_MEMORY) {
        LogWarning("micro-ROS USB CDC Transport: RX DMA direction is not memory-to-memory");
        return false;
    }
    if (config->rx_dma->Init.PeriphInc != DMA_PINC_ENABLE || config->rx_dma->Init.MemInc != DMA_MINC_ENABLE) {
        LogWarning("micro-ROS USB CDC Transport: RX DMA address increment mode is invalid");
        return false;
    }
    if (config->rx_dma->Init.PeriphDataAlignment != DMA_PDATAALIGN_BYTE ||
        config->rx_dma->Init.MemDataAlignment != DMA_MDATAALIGN_BYTE) {
        LogWarning("micro-ROS USB CDC Transport: RX DMA data alignment is invalid");
        return false;
    }
    if (config->rx_dma->Init.Mode != DMA_NORMAL) {
        LogWarning("micro-ROS USB CDC Transport: RX DMA mode is not normal");
        return false;
    }
    if (config->rx_dma->State != HAL_DMA_STATE_READY) {
        LogWarning("micro-ROS USB CDC Transport: RX DMA is not ready: %lu", (uint32_t)config->rx_dma->State);
        return false;
    }
    if (!dma_interrupt_enabled(config->rx_dma)) {
        LogWarning("micro-ROS USB CDC Transport: RX DMA interrupt is not enabled");
        return false;
    }

    return true;
}

static USBD_CDC_HandleTypeDef* get_cdc_handle(const USBD_HandleTypeDef* usb_device) {
    if (usb_device == NULL) {
        return NULL;
    }
    return (USBD_CDC_HandleTypeDef*)usb_device->pClassDataCmsit[usb_device->classId];
}

static bool dma_interrupt_enabled(const DMA_HandleTypeDef* dma) {
    IRQn_Type interrupt;

    if (dma->Instance == DMA1_Stream0)
        interrupt = DMA1_Stream0_IRQn;
    else if (dma->Instance == DMA1_Stream1)
        interrupt = DMA1_Stream1_IRQn;
    else if (dma->Instance == DMA1_Stream2)
        interrupt = DMA1_Stream2_IRQn;
    else if (dma->Instance == DMA1_Stream3)
        interrupt = DMA1_Stream3_IRQn;
    else if (dma->Instance == DMA1_Stream4)
        interrupt = DMA1_Stream4_IRQn;
    else if (dma->Instance == DMA1_Stream5)
        interrupt = DMA1_Stream5_IRQn;
    else if (dma->Instance == DMA1_Stream6)
        interrupt = DMA1_Stream6_IRQn;
    else if (dma->Instance == DMA1_Stream7)
        interrupt = DMA1_Stream7_IRQn;
    else if (dma->Instance == DMA2_Stream0)
        interrupt = DMA2_Stream0_IRQn;
    else if (dma->Instance == DMA2_Stream1)
        interrupt = DMA2_Stream1_IRQn;
    else if (dma->Instance == DMA2_Stream2)
        interrupt = DMA2_Stream2_IRQn;
    else if (dma->Instance == DMA2_Stream3)
        interrupt = DMA2_Stream3_IRQn;
    else if (dma->Instance == DMA2_Stream4)
        interrupt = DMA2_Stream4_IRQn;
    else if (dma->Instance == DMA2_Stream5)
        interrupt = DMA2_Stream5_IRQn;
    else if (dma->Instance == DMA2_Stream6)
        interrupt = DMA2_Stream6_IRQn;
    else if (dma->Instance == DMA2_Stream7)
        interrupt = DMA2_Stream7_IRQn;
    else
        return false;

    return NVIC_GetEnableIRQ(interrupt) != 0U;
}

static uint32_t get_empty_read_timeout(int timeout_ms) {
    if (timeout_ms <= 0) {
        return 0U;
    }

    /* The upper layer may request a very large timeout while waiting for framed
     * session data. Waiting for that value directly would block the connection
     * thread for too long, but a zero-timeout busy loop can starve session
     * setup. Use a tiny bounded wait instead. */
    if ((uint32_t)timeout_ms > USB_CDC_MAX_BLOCKING_READ_TIMEOUT_MS) {
        return USB_CDC_FALLBACK_READ_TIMEOUT_MS;
    }

    return (uint32_t)timeout_ms;
}

static uint32_t get_dma_read_timeout(int timeout_ms) {
    if (timeout_ms <= 0) {
        return 0U;
    }

    /* Once data is already available, the memory-to-memory DMA copy should
     * complete within a few milliseconds. Keep a bounded wait even if the
     * upper layer supplied an effectively infinite timeout. */
    if ((uint32_t)timeout_ms > USB_CDC_MAX_BLOCKING_READ_TIMEOUT_MS) {
        return USB_CDC_DMA_TIMEOUT_MS;
    }

    return (uint32_t)timeout_ms;
}

static bool reception_has_packet_space(void) {
    if (rx_full) {
        return false;
    }
    if (rx_head < rx_tail) {
        return rx_tail - rx_head >= active_packet_size;
    }
    return USB_CDC_RX_BUFFER_SIZE - rx_head >= active_packet_size;
}

static void resume_reception_if_paused(void) {
    taskENTER_CRITICAL();
    const bool should_resume = rx_paused && reception_has_packet_space();
    if (should_resume) {
        rx_paused = false;
        USBD_CDC_SetRxBuffer(active_config->usb_device, &rx_buffer[rx_head]);
    }
    taskEXIT_CRITICAL();
    if (should_resume && USBD_CDC_ReceivePacket(active_config->usb_device) != USBD_OK) {
        rx_paused = true;
    }
}

static int8_t cdc_control(uint8_t command, uint8_t* buffer, uint16_t length) {
    if (command == CDC_SET_LINE_CODING) {
        memcpy(line_coding, buffer, sizeof(line_coding));
    } else if (command == CDC_GET_LINE_CODING) {
        memcpy(buffer, line_coding, sizeof(line_coding));
    } else if (previous_interface.Control != NULL) {
        return previous_interface.Control(command, buffer, length);
    }
    return USBD_OK;
}

static int8_t cdc_receive_complete(uint8_t* buffer, uint32_t* length) {
    (void)buffer;
    UBaseType_t interrupt_state = taskENTER_CRITICAL_FROM_ISR();
    const size_t size = USB_CDC_RX_BUFFER_SIZE;
    size_t next_head = rx_head + *length;

    /* The USB stack has already written this packet into rx_buffer. Commit it
     * before deciding whether another packet fits; the callback return value
     * is ignored by ST's USBD_CDC_DataOut(), so returning BUSY cannot ask the
     * host to retry an uncommitted packet. */
    if (next_head >= size || next_head + active_packet_size > size) {
        rx_wrap = next_head <= size ? next_head : size;
        next_head = 0U;
    }
    rx_head = next_head;
    /* A zero-length USB packet carries no data and must not turn an empty ring
     * into a falsely full one. */
    rx_full = *length > 0U && rx_head == rx_tail;

    /* NAK further OUT traffic when a complete maximum-sized packet no longer
     * fits. A later read re-arms reception only after freeing enough space. */
    if (!reception_has_packet_space()) {
        rx_paused = true;
        taskEXIT_CRITICAL_FROM_ISR(interrupt_state);
        if (receive_thread != NULL) {
            (void)osThreadFlagsSet(receive_thread, USB_CDC_RX_COMPLETE_FLAG);
        }
        return USBD_OK;
    }

    USBD_CDC_SetRxBuffer(active_config->usb_device, &rx_buffer[rx_head]);
    taskEXIT_CRITICAL_FROM_ISR(interrupt_state);

    if (USBD_CDC_ReceivePacket(active_config->usb_device) != USBD_OK) {
        rx_paused = true;
        return USBD_BUSY;
    }
    if (receive_thread != NULL) {
        (void)osThreadFlagsSet(receive_thread, USB_CDC_RX_COMPLETE_FLAG);
    }
    return USBD_OK;
}

static int8_t cdc_transmit_complete(uint8_t* buffer, uint32_t* length, uint8_t endpoint) {
    (void)buffer;
    (void)length;
    (void)endpoint;
    if (transmit_thread != NULL) {
        (void)osThreadFlagsSet(transmit_thread, USB_CDC_TX_COMPLETE_FLAG);
    }
    return USBD_OK;
}

static void dma_receive_complete(DMA_HandleTypeDef* dma) {
    if (active_config != NULL && dma == active_config->rx_dma && receive_thread != NULL) {
        (void)osThreadFlagsSet(receive_thread, USB_CDC_DMA_COMPLETE_FLAG);
    }
}

static void dma_receive_error(DMA_HandleTypeDef* dma) {
    if (active_config != NULL && dma == active_config->rx_dma && receive_thread != NULL) {
        dma_failed = true;
        (void)osThreadFlagsSet(receive_thread, USB_CDC_DMA_COMPLETE_FLAG);
    }
}
