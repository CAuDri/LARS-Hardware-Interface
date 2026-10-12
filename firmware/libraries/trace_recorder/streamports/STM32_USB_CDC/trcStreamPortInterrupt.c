/*
 * CAuDri - Interrupt-driven STM32 USB CDC stream port for TraceRecorder.
 *
 * The recorder owns the TX memory until this synchronous write returns.
 * USB runs asynchronously while the calling task sleeps on a completion flag.
 * No TX staging buffer or fixed delay between transfers is needed.
 */

#include <trcRecorder.h>
#include <usb_device.h>
#include <usbd_cdc_if.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <limits.h>
#include <stdbool.h>
#include <string.h>

#if (TRC_USE_TRACEALYZER_RECORDER == 1)

#if (TRC_USE_INTERNAL_BUFFER != 1)
#error "The interrupt-driven USB stream port requires the recorder's internal buffer"
#endif

/* Reserved for the caller of this stream port (normally TzCtrl). */
#define TRACE_USB_TX_COMPLETE_FLAG 0x01U

static USBD_HandleTypeDef* const trace_usb = &DEBUG_TRACE_RECORDER_USB_HANDLE;
static TraceStreamPortBuffer_t* rx_buffer;
static osThreadId_t volatile transmit_thread;
static int8_t (*previous_receive)(uint8_t*, uint32_t*);
static int8_t (*previous_transmit_complete)(uint8_t*, uint32_t*, uint8_t);
static int8_t (*previous_deinit)(void);
static bool callbacks_installed;
static volatile bool transport_failed;
static volatile bool transfer_cancelled;

static USBD_CDC_HandleTypeDef* cdc_handle(void)
{
    return (USBD_CDC_HandleTypeDef*)trace_usb->pClassDataCmsit[trace_usb->classId];
}

static int8_t receive_complete(uint8_t* data, uint32_t* length)
{
    /* USB IRQ priority must permit FreeRTOS ISR APIs (board uses priority 5). */
    UBaseType_t interrupt_state = taskENTER_CRITICAL_FROM_ISR();
    uint32_t available = sizeof(rx_buffer->bufferUSB) - rx_buffer->idx;
    uint32_t count = *length < available ? *length : available;
    memcpy(&rx_buffer->bufferUSB[rx_buffer->idx], data, count);
    rx_buffer->idx += count;
    /* Never silently truncate a Tracealyzer command. */
    if (count != *length)
    {
        transport_failed = true;
    }
    taskEXIT_CRITICAL_FROM_ISR(interrupt_state);

    /* The original callback re-arms the OUT endpoint. */
    return previous_receive != NULL ? previous_receive(data, length) : USBD_OK;
}

static int8_t transmit_complete(uint8_t* data, uint32_t* length, uint8_t endpoint)
{
    if (previous_transmit_complete != NULL)
    {
        (void)previous_transmit_complete(data, length, endpoint);
    }
    osThreadId_t waiter = transmit_thread;
    if (waiter != NULL)
    {
        (void)osThreadFlagsSet(waiter, TRACE_USB_TX_COMPLETE_FLAG);
    }
    return USBD_OK;
}

static int8_t interface_deinit(void)
{
    /* ST closes the CDC endpoints before invoking this callback. The old TX
     * memory is now safe to release, even if no completion IRQ was delivered.
     * Cancellation survives a reconnect before the waiting task runs. */
    int8_t result = previous_deinit != NULL ? previous_deinit() : USBD_OK;
    transfer_cancelled = true;
    osThreadId_t waiter = transmit_thread;
    if (waiter != NULL)
    {
        (void)osThreadFlagsSet(waiter, TRACE_USB_TX_COMPLETE_FLAG);
    }
    return result;
}

traceResult xTraceStreamPortInitialize(TraceStreamPortBuffer_t* buffer)
{
    if (buffer == NULL)
    {
        return TRC_FAIL;
    }

    /* Called before USB class data exists, so hook the static interface table. */
    USBD_CDC_ItfTypeDef* interface = trace_usb == &hUsbDeviceHS
        ? &USBD_Interface_fops_HS : &USBD_Interface_fops_FS;
    taskENTER_CRITICAL();
    rx_buffer = buffer;
    rx_buffer->idx = 0U;
    if (!callbacks_installed)
    {
        previous_receive = interface->Receive;
        previous_transmit_complete = interface->TransmitCplt;
        previous_deinit = interface->DeInit;
        interface->Receive = receive_complete;
        interface->TransmitCplt = transmit_complete;
        interface->DeInit = interface_deinit;
        callbacks_installed = true;
    }
    taskEXIT_CRITICAL();
    return TRC_SUCCESS;
}

traceResult prvTraceCDCReceive(void* data, uint32_t size, int32_t* bytes_received)
{
    if (bytes_received == NULL)
    {
        return TRC_FAIL;
    }
    *bytes_received = 0;
    if (data == NULL || rx_buffer == NULL || transport_failed)
    {
        return TRC_FAIL;
    }

    /* Protect the short command copy and compaction against the USB RX IRQ. */
    taskENTER_CRITICAL();
    uint32_t count = size < rx_buffer->idx ? size : rx_buffer->idx;
    memcpy(data, rx_buffer->bufferUSB, count);
    rx_buffer->idx -= count;
    memmove(rx_buffer->bufferUSB, &rx_buffer->bufferUSB[count], rx_buffer->idx);
    taskEXIT_CRITICAL();
    *bytes_received = (int32_t)count;
    return TRC_SUCCESS;
}

traceResult prvTraceCDCTransmit(void* data, uint32_t size, int32_t* bytes_sent)
{
    if (bytes_sent == NULL)
    {
        return TRC_FAIL;
    }
    *bytes_sent = 0;
    if (size == 0U)
    {
        return TRC_SUCCESS;
    }
    osThreadId_t caller = osThreadGetId();
    if (data == NULL || size > INT32_MAX || caller == NULL || transport_failed)
    {
        return TRC_FAIL;
    }

    /* Serialize writers without allocating a mutex. Publish the waiter before
     * starting USB so even a completion before FlagsWait cannot be lost. */
    taskENTER_CRITICAL();
    if (transmit_thread != NULL)
    {
        taskEXIT_CRITICAL();
        return TRC_SUCCESS; /* No bytes consumed; the recorder retries later. */
    }
    transmit_thread = caller;
    taskEXIT_CRITICAL();
    (void)osThreadFlagsClear(TRACE_USB_TX_COMPLETE_FLAG);

    TickType_t started = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(TRC_CFG_STREAM_PORT_TX_TIMEOUT_MS);
    if (timeout == 0U)
    {
        timeout = 1U;
    }
    bool submitted = false;
    /* Temporary BUSY/disconnect conditions leave the recorder data queued. */

    for (;;)
    {
        taskENTER_CRITICAL();
        USBD_CDC_HandleTypeDef* cdc = cdc_handle();
        bool configured = trace_usb->dev_state == USBD_STATE_CONFIGURED && cdc != NULL;
        if (submitted && (transfer_cancelled || cdc == NULL))
        {
            taskEXIT_CRITICAL();
            break; /* Endpoints closed; retry queued data after reconnect. */
        }
        if (submitted && cdc->TxState == 0U)
        {
            /* The stack completed TX, even if its notification was lost. */
            *bytes_sent = (int32_t)size;
            taskEXIT_CRITICAL();
            break;
        }
        if (configured && !submitted && cdc->TxState == 0U)
        {
            /* A previous transfer may have completed between our initial
             * flag clear and this check. Clear its flag while USB IRQs are
             * masked so it cannot acknowledge the transfer we start below. */
            (void)osThreadFlagsClear(TRACE_USB_TX_COMPLETE_FLAG);
            transfer_cancelled = false;
            /* Never replace the buffer belonging to a BUSY transfer. */
            if (USBD_CDC_SetTxBuffer(trace_usb, data, size) == USBD_OK &&
                USBD_CDC_TransmitPacket(trace_usb) == USBD_OK)
            {
                submitted = true;
            }
            else
            {
                taskEXIT_CRITICAL();
                break;
            }
        }
        taskEXIT_CRITICAL();

        if (!configured && !submitted)
        {
            break;
        }
        TickType_t elapsed = xTaskGetTickCount() - started;
        if (elapsed >= timeout)
        {
            if (!submitted)
            {
                break;
            }
            /* Keep the in-flight memory and USB byte-stream framing intact.
             * Other tasks run; a full recorder buffer skips new events. */
            started = xTaskGetTickCount();
            elapsed = 0U;
        }
        uint32_t flags = osThreadFlagsWait(TRACE_USB_TX_COMPLETE_FLAG,
                                          osFlagsWaitAny, timeout - elapsed);
        if ((flags & osFlagsError) != 0U && !submitted)
        {
            break;
        }
        /* Timeouts trigger a fresh CDC state check, not an abort. Check
         * cancellation and TxState before acknowledging any bytes: a flag
         * may also belong to a different user's BUSY transfer. */
    }

    taskENTER_CRITICAL();
    transmit_thread = NULL;
    taskEXIT_CRITICAL();
    return TRC_SUCCESS;
}

#endif
