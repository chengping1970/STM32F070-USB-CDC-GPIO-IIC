/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  512
#define APP_TX_DATA_SIZE  512
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
//uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
//uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

PacketInfo_T RecPespInfo;
uint8_t PacketBuffer[CDC_I2C_MAX_PACKETS][CDC_I2C_PACKET_SZ];
uint8_t ResponeBuffer[CDC_I2C_MAX_PACKETS][CDC_I2C_PACKET_SZ];

uint32_t XferDelay = 9600;

uint32_t YellowDelay = 0;
uint32_t RedLEDDelay = 0;
/* USER CODE BEGIN PRIVATE_VARIABLES */
static const char *g_fwVersion = "AUC-20180408";
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  memset(&RecPespInfo, 0, sizeof(PacketInfo_T));
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, ResponeBuffer[0], 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, PacketBuffer[0]);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  if (RecPespInfo.ReceivePacketsCount < CDC_I2C_MAX_PACKETS)
  {
    RecPespInfo.ReceivePacketsCount++;
	RecPespInfo.ReceivePacketPosition++;
	if (RecPespInfo.ReceivePacketPosition == CDC_I2C_MAX_PACKETS)
	{
		RecPespInfo.ReceivePacketPosition = 0;
	}
  }
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, PacketBuffer[RecPespInfo.ReceivePacketPosition]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

static void CDC_Delay(uint32_t Delay)
{
	while (Delay--)
	{
		__NOP();
	}
}

static void _SCL_Delay_Short(void)
{
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}

static void _SCL_Delay(void)
{
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}

static inline void _SetSDAInput(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = IIC_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStruct);
}
static inline void _SetSDAOutput(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = IIC_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; 
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStruct);
}

static void _SWI2C_Start(void)
{
	HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET);
	_SCL_Delay();
	HAL_GPIO_WritePin(IIC_SDA_PORT,IIC_SDA_PIN, GPIO_PIN_RESET);
	_SCL_Delay();
	HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN, GPIO_PIN_RESET);
	_SCL_Delay();
}

static void _SWI2C_Stop(void)
{
	_SetSDAOutput();
	HAL_GPIO_WritePin(IIC_SDA_PORT,IIC_SDA_PIN, GPIO_PIN_RESET);
	_SCL_Delay();
	HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
	_SCL_Delay();
	HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET);
}

static HAL_StatusTypeDef _SWI2C_WriteByte(uint8_t Value)
{
	uint8_t count;
	
	for (count = 0;count < 8;count++)
	{
		_SCL_Delay_Short();
		if (Value&0x80)
		{
			HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(IIC_SDA_PORT,IIC_SDA_PIN, GPIO_PIN_RESET);
		}
		_SCL_Delay_Short();
		HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
		_SCL_Delay();
		HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN, GPIO_PIN_RESET);
		Value = Value<<1;
	}
	HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET);
	_SetSDAInput();
	HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
	for (count = 0;count < 20;count++)
	{
		_SCL_Delay_Short();
		if (HAL_GPIO_ReadPin(IIC_SDA_PORT, IIC_SDA_PIN) == 0)
		{
			HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN, GPIO_PIN_RESET);
			_SetSDAOutput();
			return HAL_OK;
		}
	}
	HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN, GPIO_PIN_RESET);	
	_SetSDAOutput();
	return HAL_ERROR;
}

static uint8_t _SWI2C_ReadByte(uint8_t SendAck)
{
	uint8_t count, read, value = 0;

	_SetSDAInput();
	for (count = 0;count < 8;count++)
	{
		_SCL_Delay_Short();
		HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
		_SCL_Delay();
		if (HAL_GPIO_ReadPin(IIC_SDA_PORT, IIC_SDA_PIN))
		{
			read = 1;
		}
		else
		{
			read = 0;
		}
		value = (value<<1)|read;
		HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN, GPIO_PIN_RESET);
		_SCL_Delay_Short();
	}		
	_SetSDAOutput();
	if (SendAck)
	{
		HAL_GPIO_WritePin(IIC_SDA_PORT,IIC_SDA_PIN, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(IIC_SDA_PORT,IIC_SDA_PIN, GPIO_PIN_SET);
	}		
	HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
	_SCL_Delay();
	HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN, GPIO_PIN_RESET);

	return value;
}

static HAL_StatusTypeDef SWI2C_Read(uint8_t Addr, uint8_t SubAddr, uint8_t * pData, uint8_t Length, uint8_t Option)
{
	uint8_t i;
	HAL_StatusTypeDef result;
	
	_SWI2C_Start();

	if ((Option&CDC_I2C_TRANSFER_OPTIONS_SUB_ADDRESS) == CDC_I2C_TRANSFER_OPTIONS_SUB_ADDRESS)
	{
		result = _SWI2C_WriteByte(Addr);
		if (result != HAL_OK)
		{
			_SWI2C_Stop();
			return result;
		}

		result = _SWI2C_WriteByte(SubAddr);
		if (result != HAL_OK)
		{
			_SWI2C_Stop();
			return result;
		}

		_SWI2C_Start(); // restart
	}

	if ((Option&CDC_I2C_TRANSFER_OPTIONS_NO_ADDRESS) == 0)
	{
		result = _SWI2C_WriteByte(Addr|0x01);
		if (result != HAL_OK)
		{
			_SWI2C_Stop();
			return result;
		}
	}
	
	for (i = 0; i < Length; i++)
	{
		if (i == (Length - 1))
			pData[i] = _SWI2C_ReadByte(0);
		else
			pData[i] = _SWI2C_ReadByte(1);
	}
	_SWI2C_Stop();
	return result;
}

static HAL_StatusTypeDef SWI2C_Write(uint8_t Addr, uint8_t SubAddr, uint8_t * pData, uint8_t Length, uint8_t Option)
{		
	uint8_t i;
	HAL_StatusTypeDef result;
	
	_SWI2C_Start();

	if ((Option&CDC_I2C_TRANSFER_OPTIONS_NO_ADDRESS) == 0)
	{
		result = _SWI2C_WriteByte(Addr);
		if (result != HAL_OK)
		{
			_SWI2C_Stop();
			return result;
		}
	}
	
	if ((Option&CDC_I2C_TRANSFER_OPTIONS_SUB_ADDRESS) == CDC_I2C_TRANSFER_OPTIONS_SUB_ADDRESS)
	{
		result = _SWI2C_WriteByte(SubAddr);
		if (result != HAL_OK)
		{
			_SWI2C_Stop();
			return result;
		}
	}
	
	for (i = 0; i < Length; i++)
	{
		result = _SWI2C_WriteByte(pData[i]);
		if (result != HAL_OK)
		{
			_SWI2C_Stop();
			return result;
		}
	}
	_SWI2C_Stop();
	return result;
}

static HAL_StatusTypeDef SWI2C_XferRead(uint8_t Addr, uint8_t * pData, uint8_t Length)
{
	uint8_t i;
	HAL_StatusTypeDef result;
	
	_SWI2C_Start();

	result = _SWI2C_WriteByte(Addr|0x01);
	if (result != HAL_OK)
	{
		_SWI2C_Stop();
		return result;
	}
	
	for (i = 0; i < Length; i++)
	{
		if (i == (Length - 1))
			pData[i] = _SWI2C_ReadByte(0);
		else
			pData[i] = _SWI2C_ReadByte(1);
	}
	_SWI2C_Stop();
	return result;
}

static HAL_StatusTypeDef SWI2C_XferWrite(uint8_t Addr, uint8_t * pData, uint8_t Length)
{		
	uint8_t i;
	HAL_StatusTypeDef result;
	
	_SWI2C_Start();
	
	result = _SWI2C_WriteByte(Addr);
	if (result != HAL_OK)
	{
		_SWI2C_Stop();
		return result;
	}
		
	for (i = 0; i < Length; i++)
	{
		result = _SWI2C_WriteByte(pData[i]);
		if (result != HAL_OK)
		{
			_SWI2C_Stop();
			return result;
		}
	}
	_SWI2C_Stop();
	return result;
}


/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
#define I2C_RETRY_DELAY  9600
#define I2C_RERY_COUNT   10
void CDC_I2C_Process(IWDG_HandleTypeDef * pIWDG)
{
  if (RecPespInfo.ReceivePacketsCount)
  {
  	CDC_I2C_OUT_REPORT_T * pCDCI2COutput = (CDC_I2C_OUT_REPORT_T *)PacketBuffer[RecPespInfo.ProcessReceivePositon];
	CDC_I2C_IN_REPORT_T * pCDCI2CInput = (CDC_I2C_IN_REPORT_T *)ResponeBuffer[RecPespInfo.ProcessReceivePositon];
	
	RecPespInfo.ReceivePacketsCount--;
	RecPespInfo.ProcessReceivePositon++;
	if (RecPespInfo.ProcessReceivePositon == CDC_I2C_MAX_PACKETS)
	{
		RecPespInfo.ProcessReceivePositon = 0;
	}
	memcpy(pCDCI2CInput, pCDCI2COutput, CDC_I2C_HEADER_SZ - 1);
	pCDCI2CInput->resp = CDC_I2C_RES_INVALID_CMD;
	switch (pCDCI2COutput->req)
	{
		case CDC_I2C_REQ_RESET:
			_SWI2C_Start();
			_SWI2C_WriteByte(0xFF);
			_SWI2C_Stop();
			pCDCI2CInput->resp = CDC_I2C_RES_OK;
			break;
			
		case CDC_I2C_REQ_DEINIT_PORT:
			pCDCI2CInput->resp = CDC_I2C_RES_OK;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
			break;
			
		case CDC_I2C_REQ_INIT_PORT:
			{
				CDC_I2C_PORTCONFIG_T * pConfig;

				pConfig = (CDC_I2C_PORTCONFIG_T *) &pCDCI2COutput->data[0];
				XferDelay = pConfig->xferDelay;
				pCDCI2CInput->resp = CDC_I2C_RES_OK;
				memcpy((uint8_t *)pCDCI2CInput->data, g_fwVersion, strlen(g_fwVersion));
				pCDCI2CInput->length = CDC_I2C_HEADER_SZ + strlen(g_fwVersion);
				{
					pIWDG->Instance = IWDG;
					pIWDG->Init.Prescaler = IWDG_PRESCALER_32;
					pIWDG->Init.Window = 4095;
					pIWDG->Init.Reload = 4095;
					if (HAL_IWDG_Init(pIWDG) != HAL_OK)
					{
						_Error_Handler(__FILE__, __LINE__);
					}
				}
			}
			break;
			
		case CDC_I2C_REQ_DEVICE_WRITE:
			{
				CDC_I2C_RW_PARAMS_T * pWriteParam = (CDC_I2C_RW_PARAMS_T *) &pCDCI2COutput->data[0];
				HAL_StatusTypeDef Ret;

				pCDCI2CInput->length = CDC_I2C_HEADER_SZ;
				pCDCI2CInput->resp = CDC_I2C_RES_SLAVE_NAK;
				{
					uint8_t Retry = I2C_RERY_COUNT;
					while (Retry--)
					{
						Ret = SWI2C_Write(pWriteParam->slaveAddr<<1, 0, (uint8_t *)&pCDCI2CInput->data[0], (uint8_t)pWriteParam->length, pWriteParam->options);
						if (Ret == HAL_OK)
						{
							pCDCI2CInput->resp = CDC_I2C_RES_OK;
							break;
						}
						CDC_Delay(I2C_RETRY_DELAY);
					}
				}
				HAL_IWDG_Refresh(pIWDG);
			}
			break;
			
		case CDC_I2C_REQ_DEVICE_READ:
			{
				CDC_I2C_RW_PARAMS_T * pReadParam = (CDC_I2C_RW_PARAMS_T *) &pCDCI2COutput->data[0];
				HAL_StatusTypeDef Ret;

				pCDCI2CInput->length = CDC_I2C_HEADER_SZ;
				pCDCI2CInput->resp = CDC_I2C_RES_SLAVE_NAK;
				{
					uint8_t Retry = I2C_RERY_COUNT;
					while (Retry--)
					{
						Ret = SWI2C_Read(pReadParam->slaveAddr<<1, 0, (uint8_t *)&pCDCI2CInput->data[0], (uint8_t)pReadParam->length, pReadParam->options);
						if (Ret == HAL_OK)
						{
							pCDCI2CInput->resp = CDC_I2C_RES_OK;
							pCDCI2CInput->length += pReadParam->length;	
							break;
						}
						CDC_Delay(I2C_RETRY_DELAY);
					}
				}
				HAL_IWDG_Refresh(pIWDG);
			}
			break;

		case CDC_I2C_REQ_DEVICE_XFER:
			{
				CDC_I2C_XFER_PARAMS_T * pXfrParam = (CDC_I2C_XFER_PARAMS_T *) &pCDCI2COutput->data[0];
				HAL_StatusTypeDef Ret;

				pCDCI2CInput->length = CDC_I2C_HEADER_SZ;
				pCDCI2CInput->resp = CDC_I2C_RES_SLAVE_NAK;
				{
					uint8_t Retry = I2C_RERY_COUNT;
					while (Retry--)
					{
						Ret = SWI2C_XferWrite(pXfrParam->slaveAddr<<1, (uint8_t *)&pXfrParam->data[0], (uint8_t)pXfrParam->txLength);
						if (Ret == HAL_OK)
						{
							pCDCI2CInput->resp = CDC_I2C_RES_OK;
							break;
						}
						CDC_Delay(I2C_RETRY_DELAY);
					}
				}
				if (Ret == HAL_OK)
				{
					pCDCI2CInput->resp = CDC_I2C_RES_OK;
					if (pXfrParam->rxLength != 0)
					{
						CDC_Delay(XferDelay);
						pCDCI2CInput->resp = CDC_I2C_RES_SLAVE_NAK;
						{
							uint8_t Retry = I2C_RERY_COUNT;
							while (Retry--)
							{
								Ret = SWI2C_XferRead(pXfrParam->slaveAddr<<1, (uint8_t *)&pCDCI2CInput->data[0], (uint8_t)pXfrParam->rxLength);
								if (Ret == HAL_OK)
								{
									pCDCI2CInput->resp = CDC_I2C_RES_OK;
									pCDCI2CInput->length += pXfrParam->rxLength;	
									break;
								}
								CDC_Delay(I2C_RETRY_DELAY);
							}
						}
					}
				}
				else
				{
					pCDCI2CInput->resp = CDC_I2C_RES_SLAVE_NAK;
				}	
				if (pCDCI2CInput->resp == CDC_I2C_RES_OK)
				{
					YellowDelay = 200;
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
				}
				else
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
					RedLEDDelay = 200;
				}
				HAL_IWDG_Refresh(pIWDG);
			}
			break;
			
		case CDC_I2C_REQ_DEVICE_INFO:
			memset(&pCDCI2CInput->data[0], 0 ,7);
			pCDCI2CInput->data[0] = 0x88;
			pCDCI2CInput->data[1] = 0x01;
			pCDCI2CInput->length += 7;
			pCDCI2CInput->resp = CDC_I2C_RES_OK;
			break;
			
		default:
			break;
	}
  }

  if (RecPespInfo.ProcessReceivePositon != RecPespInfo.ProcessResponePositon)
  {
    CDC_I2C_OUT_REPORT_T * pCDCI2CInput = (CDC_I2C_OUT_REPORT_T *)ResponeBuffer[RecPespInfo.ProcessResponePositon];
    uint8_t result;
	
    result = CDC_Transmit_FS(ResponeBuffer[RecPespInfo.ProcessResponePositon], pCDCI2CInput->length);
	if (result == USBD_OK)
	{
		RecPespInfo.ProcessResponePositon++;
		if (RecPespInfo.ProcessResponePositon == CDC_I2C_MAX_PACKETS)
		{
			RecPespInfo.ProcessResponePositon = 0;
		}
	}
  }

  if (YellowDelay == 1)
  {
  	YellowDelay = 0;
  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  }
  if (RedLEDDelay == 1)
  {
  	RedLEDDelay = 0;
  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  }
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

void CDC_UpdateTimer(void)
{
	if (YellowDelay > 1)
		YellowDelay--;
	if (RedLEDDelay > 1)
		RedLEDDelay--;
}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
