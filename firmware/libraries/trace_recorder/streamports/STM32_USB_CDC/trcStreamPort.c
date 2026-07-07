/*
 * Trace Recorder for Tracealyzer v4.11.0
 * Copyright 2025 Percepio AB
 * www.percepio.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Supporting functions for trace streaming ("stream ports").
 * This "stream port" sets up the recorder to use USB CDC as streaming channel.
 * The example is for STM32 using STM32Cube.
 */

#include <trcRecorder.h>

#include <usb_device.h>
#include <usbd_cdc_if.h>

#if (TRC_USE_TRACEALYZER_RECORDER == 1)

static void prvCDCInit(void);

static int8_t CDC_Receive_modified(uint8_t* pbuf, uint32_t *puiLength);

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_HS;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_HandleTypeDef hUsbDeviceHS;

static USBD_HandleTypeDef* volatile pxTraceUSBHandle = &DEBUG_TRACE_RECORDER_USB_HANDLE;
static int8_t(*CDC_Receive)(uint8_t* Buf, uint32_t* Len);
static uint32_t uiConsecutiveBusyTransmits = 0U;

TraceStreamPortBuffer_t* pxUSBBuffers TRC_CFG_RECORDER_DATA_ATTRIBUTE;

static int8_t CDC_Receive_modified(uint8_t* pBuffer, uint32_t *puiLength)
{
	for(uint32_t i = 0; i < *puiLength; i++)
	{
		if (pxUSBBuffers->idx >= sizeof(pxUSBBuffers->bufferUSB))
		{
			break;
		}

		pxUSBBuffers->bufferUSB[pxUSBBuffers->idx] = pBuffer[i];
		pxUSBBuffers->idx++;
	}

	CDC_Receive(pBuffer, puiLength);

	return (USBD_OK);
}

static USBD_CDC_ItfTypeDef* prvGetCDCInterface(void)
{
	if (pxTraceUSBHandle == &hUsbDeviceHS)
	{
		return &USBD_Interface_fops_HS;
	}

	return &USBD_Interface_fops_FS;
}

static USBD_CDC_HandleTypeDef* prvGetCDCHandle(void)
{
	if (pxTraceUSBHandle == 0)
	{
		return 0;
	}

	return (USBD_CDC_HandleTypeDef*)pxTraceUSBHandle->pClassDataCmsit[pxTraceUSBHandle->classId];
}

static int32_t prvTraceCDCTransmitData(uint8_t* pBuffer, uint32_t uiLength)
{
	USBD_CDC_HandleTypeDef* pxCDCHandle = prvGetCDCHandle();

	if (pxCDCHandle == 0 || pxTraceUSBHandle->dev_state != USBD_STATE_CONFIGURED)
	{
		return USBD_FAIL;
	}

	if (pxCDCHandle->TxState != 0U)
	{
		return USBD_BUSY;
	}

	if (USBD_CDC_SetTxBuffer(pxTraceUSBHandle, pBuffer, uiLength) != USBD_OK)
	{
		return USBD_FAIL;
	}

	return USBD_CDC_TransmitPacket(pxTraceUSBHandle);
}

static void prvCDCInit(void)
{
	USBD_CDC_ItfTypeDef* pxCDCInterface = prvGetCDCInterface();

	/* Store the original "Receive" function, from the static initialization */
	CDC_Receive = pxCDCInterface->Receive;

	/* Update the function pointer with our modified variant */
	pxCDCInterface->Receive = CDC_Receive_modified;

	pxUSBBuffers->idx = 0;
}

/* The READ function, used in trcStreamPort.h */
traceResult prvTraceCDCReceive(void *data, uint32_t uiSize, int32_t* piBytesReceived)
{
	uint32_t i, uiDiff;

	if(pxUSBBuffers->idx > 0)
	{
		if ((TraceUnsignedBaseType_t)uiSize >= pxUSBBuffers->idx) // More than what is stored, number of bytes will be .idx
		{
			TRC_MEMCPY(data, pxUSBBuffers->bufferUSB, pxUSBBuffers->idx);
			*piBytesReceived = (int32_t)pxUSBBuffers->idx;
			pxUSBBuffers->idx = 0; // Make the buffer ready for a new command
		}
		else  // If some data in the buffer is not read
		{
			uiDiff = pxUSBBuffers->idx - uiSize;
			TRC_MEMCPY(data, pxUSBBuffers->bufferUSB, uiSize);

			for(i = 0; i < uiDiff; i++)
			{
				pxUSBBuffers->bufferUSB[i] = pxUSBBuffers->bufferUSB[i + uiSize];
			}
			
			*piBytesReceived = uiSize;
			
			pxUSBBuffers->idx = uiDiff;
		}
	}
	else
	{
		*piBytesReceived = 0;
	}
	
	return TRC_SUCCESS;
}

/* The WRITE function, used in trcStreamPort.h */
traceResult prvTraceCDCTransmit(void* pvData, uint32_t uiSize, int32_t * piBytesSent )
{
	int32_t result;

	*piBytesSent = 0;

	result = prvTraceCDCTransmitData(pvData, uiSize);
	
	if (result == USBD_OK)
	{
		uiConsecutiveBusyTransmits = 0U;
		*piBytesSent = uiSize;
		return TRC_SUCCESS;
	}
	else if (result == USBD_BUSY)
	{
		uiConsecutiveBusyTransmits++;
		if (uiConsecutiveBusyTransmits >= 100U)
		{
			USBD_CDC_HandleTypeDef* pxCDCHandle = prvGetCDCHandle();
			if (pxCDCHandle != 0)
			{
				pxCDCHandle->TxState = 0U;
			}
			uiConsecutiveBusyTransmits = 0U;
		}
		xTraceKernelPortDelay(TRC_CFG_STREAM_PORT_DELAY_ON_BUSY);
		return TRC_SUCCESS;
	}

	uiConsecutiveBusyTransmits = 0U;
	return TRC_FAIL;
}

traceResult xTraceStreamPortInitialize(TraceStreamPortBuffer_t* pxBuffer)
{
	if (pxBuffer == 0)
	{
		return TRC_FAIL;
	}

	pxUSBBuffers = pxBuffer;

	prvCDCInit();

	return TRC_SUCCESS;
}

#endif  /*(TRC_USE_TRACEALYZER_RECORDER == 1)*/
