/**
  ******************************************************************************
  * @file           : usbd_cdc_if.h
  * @version        : v2.0_Cube
  * @brief          : Header for usbd_cdc_if.c file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief For Usb device.
  * @{
  */
  
/** @defgroup USBD_CDC_IF USBD_CDC_IF
  * @brief Usb VCP device module
  * @{
  */ 

/** @defgroup USBD_CDC_IF_Exported_Defines USBD_CDC_IF_Exported_Defines
  * @brief Defines.
  * @{
  */
/* USER CODE BEGIN EXPORTED_DEFINES */

#define CDC_I2C_MAX_PACKETS         8			/*!< Maximum packets allowed in processing queue */
#define CDC_I2C_PACKET_SZ           64			/*!< Packet size of each I2C command packet */
#define CDC_I2C_HEADER_SZ           4			/*!< Size of the header in I2C command packet */

/* CDC_I2C Requests */
#define CDC_I2C_REQ_RESET           0xa0		/*!< Request to abort and flush all pending requests */
#define CDC_I2C_REQ_INIT_PORT       0xa1		/*!< Request to initialize the I2C port */
#define CDC_I2C_REQ_DEINIT_PORT     0xa2		/*!< Request to de-initialize the I2C port */
#define CDC_I2C_REQ_DEVICE_WRITE    0xa3		/*!< Request to write data to the I2C port */
#define CDC_I2C_REQ_DEVICE_READ     0xa4		/*!< Request to read data from the I2C port */
#define CDC_I2C_REQ_DEVICE_XFER     0xa5		/*!< Request to write and then read data from the I2C port */
#define CDC_I2C_REQ_DEVICE_INFO     0xa6		/*!< Request to get device information */

/** CDC_I2C responses. The response code below 0x10 should match with I2CM_STATUS codes. */
#define CDC_I2C_RES_OK              0x00		/*!< Requested Request was executed successfully. */
#define CDC_I2C_RES_ERROR           0x01		/*!< Unknown error condition. */
#define CDC_I2C_RES_NAK             0x02		/*!< No device responded for given slave address. */
#define CDC_I2C_RES_BUS_ERROR       0x03		/*!< I2C bus error */
#define CDC_I2C_RES_SLAVE_NAK       0x04		/*!< NAK received after SLA+W or SLA+R */
#define CDC_I2C_RES_ARBLOST         0x05		/*!< Arbitration lost */

#define CDC_I2C_RES_TIMEOUT         0x10		/*!< Transaction timed out. */
#define CDC_I2C_RES_INVALID_CMD     0x11		/*!< Invalid CDC_I2C Request or Request not supported in this version. */
#define CDC_I2C_RES_INVALID_PARAM   0x12		/*!< Invalid parameters are provided for the given Request. */
#define CDC_I2C_RES_PARTIAL_DATA    0x13		/*!< Partial transfer completed. */

/** I2C_IO_OPTIONS Options to I2C_DeviceWrite & I2C_DeviceRead routines
 * @{
 */
/** Generate start condition before transmitting */
#define CDC_I2C_TRANSFER_OPTIONS_START_BIT      0x0001

/** Generate stop condition at the end of transfer */
#define CDC_I2C_TRANSFER_OPTIONS_STOP_BIT       0x0002

/** Continue transmitting data in bulk without caring about Ack or nAck from device if this bit is
   not set. If this bit is set then stop transmitting the data in the buffer when the device nAcks*/
#define CDC_I2C_TRANSFER_OPTIONS_BREAK_ON_NACK  0x0004

/** lpcusbsio-I2C generates an ACKs for every byte read. Some I2C slaves require the I2C
   master to generate a nACK for the last data byte read. Setting this bit enables working with such
   I2C slaves */
#define CDC_I2C_TRANSFER_OPTIONS_NACK_LAST_BYTE 0x0008

/* Setting this bit would mean that the address field should be ignored.
    The address is either a part of the data or this is a special I2C
    frame that doesn't require an address. For example when transferring a
    frame greater than the USB_CDC packet this option can be used. */
#define CDC_I2C_TRANSFER_OPTIONS_NO_ADDRESS     0x0040

#define CDC_I2C_TRANSFER_OPTIONS_SUB_ADDRESS    0x0080
/** I2C_FAST_XFER_OPTIONS I2C master faster transfer options
 * @{
 */

/** Ignore NACK during data transfer. By default transfer is aborted. */
#define I2C_FAST_XFER_OPTION_IGNORE_NACK     	0x01
/** ACK last byte received. By default we NACK last byte we receive per I2C spec. */
#define I2C_FAST_XFER_OPTION_LAST_RX_ACK     	0x02

#define CDC_I2C_TX_BUSY        		1

#define CDC_I2C_STATE_INIT          0
#define CDC_I2C_STATE_DISCON        1
#define CDC_I2C_STATE_CONNECTED     2
#define CDC_I2C_STATE_NEEDRESET     3
#define CDC_I2C_STATE_UNKNOWN       4

#define IIC_SCL_PORT			GPIOA
#define IIC_SCL_PIN				GPIO_PIN_3
#define IIC_SDA_PORT			GPIOA
#define IIC_SDA_PIN				GPIO_PIN_2
/* USER CODE END EXPORTED_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Types USBD_CDC_IF_Exported_Types
  * @brief Types.
  * @{
  */

/* USER CODE BEGIN EXPORTED_TYPES */

/**
 * @brief	CDC to I2C bridge Request structure.
 *  Defines the structure of CDC to I2C Request packet. This is same as
 *  CDC OUT report.
 */

typedef struct __CDCI2C_OUT_REPORT {
	uint8_t length;		/*!< Length of the CDC_I2C Request structure including cmd, transId and length fields. */
	uint8_t transId;	/*!< I2C_CDC transaction identifier. Rolls over after 255. */
	uint8_t sesId;		/*!< I2C_CDC session identifier. */
	uint8_t req;		/*!< I2C_CDC Request */
	uint8_t data[];		/*!< Data corresponding to the Request */

} CDC_I2C_OUT_REPORT_T;

/**
 * @brief	CDC to I2C bridge response structure.
 *  Defines the structure of CDC to I2C Request packet. This is same as
 *  CDC OUT report.
 */
typedef struct __CDCI2C_IN_REPORT {
	uint8_t length;		/*!< Length of the CDC_I2C response structure including resp, transId and length fields.*/
	uint8_t transId;	/*!< I2C_CDC transaction identifier. */
	uint8_t sesId;		/*!< I2C_CDC session identifier. */
	uint8_t resp;		/*!< I2C_CDC reponse */
	uint8_t data[];		/*!< Data corresponding to the response */

} CDC_I2C_IN_REPORT_T;

/**
 * @brief	CDC to I2C bridge fast transfer parameters structure.
 *  Defines the parameters structure for CDC_I2C_REQ_DEVICE_XFER command.
 */
typedef struct __CDCI2C_XFER_PARAMS {
	uint8_t txLength;	/*!< Length of the Tx transfer.*/
	uint8_t rxLength;	/*!< Length of the Rx transfer. */
	uint8_t slaveAddr;	/*!< I2C slave device address. */	
	uint8_t SubAddr;	/*!< I2C slave sub address. */
	uint8_t data[];		/*!< Data corresponding to the response */

} CDC_I2C_XFER_PARAMS_T;

/**
 * @brief Port configuration information
 */
typedef struct __CDCI2C_PortConfig_t {
	uint32_t busSpeed;	/*!< I2C bus speed */
	uint32_t xferDelay;	/*!< Xfer delay */
} CDC_I2C_PORTCONFIG_T;

/**
 * @brief	CDC to I2C bridge read and write transfer parameters structure.
 *  Defines the structure of CDC to I2C read-write transfer parameters.
 */
typedef struct __CDCI2C_RW_PARAMS {
	uint8_t length;	/*!< Length of the transfer.*/
	uint8_t options;	/*!< check @ref I2C_IO_OPTIONS. */
	uint8_t slaveAddr;	/*!< I2C slave device address. */
	uint8_t SubAddr;	/*!< I2C slave sub address. */
	uint8_t data[];		/*!< Data corresponding to the response */

} CDC_I2C_RW_PARAMS_T;

typedef struct __PacketInfo {
	uint8_t ReceivePacketsCount;
	uint8_t ReceivePacketPosition;	
	uint8_t ProcessReceivePositon;	
	uint8_t ProcessResponePositon;	
} PacketInfo_T;
/* USER CODE END EXPORTED_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Macros USBD_CDC_IF_Exported_Macros
  * @brief Aliases.
  * @{
  */

/* USER CODE BEGIN EXPORTED_MACRO */

/* USER CODE END EXPORTED_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

/** CDC Interface callback. */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_FunctionsPrototype USBD_CDC_IF_Exported_FunctionsPrototype
  * @brief Public functions declaration.
  * @{
  */

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

/* USER CODE BEGIN EXPORTED_FUNCTIONS */

void CDC_I2C_Process(IWDG_HandleTypeDef * pIWDG);

void CDC_UpdateTimer(void);

/* USER CODE END EXPORTED_FUNCTIONS */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
