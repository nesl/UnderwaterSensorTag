/**********************************************************************************************
 * Filename:       dataTransferService.c
 *
 * Description:    This file contains the implementation of the service.
 *
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *************************************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "dataTransferService.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// dataTransferService Service UUID
CONST uint8_t dataTransferServiceUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DATATRANSFERSERVICE_SERV_UUID), HI_UINT16(DATATRANSFERSERVICE_SERV_UUID)
};

// Payload UUID
CONST uint8_t dataTransferService_PayloadUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DATATRANSFERSERVICE_PAYLOAD_UUID), HI_UINT16(DATATRANSFERSERVICE_PAYLOAD_UUID)
};
// PayloadCopy UUID
CONST uint8_t dataTransferService_PayloadCopyUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DATATRANSFERSERVICE_PAYLOADCOPY_UUID), HI_UINT16(DATATRANSFERSERVICE_PAYLOADCOPY_UUID)
};
// PayloadValid UUID
CONST uint8_t dataTransferService_PayloadValidUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DATATRANSFERSERVICE_PAYLOADVALID_UUID), HI_UINT16(DATATRANSFERSERVICE_PAYLOADVALID_UUID)
};
// PayloadCopyValid UUID
CONST uint8_t dataTransferService_PayloadCopyValidUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DATATRANSFERSERVICE_PAYLOADCOPYVALID_UUID), HI_UINT16(DATATRANSFERSERVICE_PAYLOADCOPYVALID_UUID)
};
// Request UUID
CONST uint8_t dataTransferService_RequestUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DATATRANSFERSERVICE_REQUEST_UUID), HI_UINT16(DATATRANSFERSERVICE_REQUEST_UUID)
};
// EndOfTransmission UUID
CONST uint8_t dataTransferService_EndOfTransmissionUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DATATRANSFERSERVICE_ENDOFTRANSMISSION_UUID), HI_UINT16(DATATRANSFERSERVICE_ENDOFTRANSMISSION_UUID)
};
// Erase UUID
CONST uint8_t dataTransferService_EraseUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DATATRANSFERSERVICE_ERASE_UUID), HI_UINT16(DATATRANSFERSERVICE_ERASE_UUID)
};
// WriteInput UUID
CONST uint8_t dataTransferService_WriteInputUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DATATRANSFERSERVICE_WRITEINPUT_UUID), HI_UINT16(DATATRANSFERSERVICE_WRITEINPUT_UUID)
};
// WriteACK UUID
CONST uint8_t dataTransferService_WriteACKUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DATATRANSFERSERVICE_WRITEACK_UUID), HI_UINT16(DATATRANSFERSERVICE_WRITEACK_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static dataTransferServiceCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t dataTransferServiceDecl = { ATT_BT_UUID_SIZE, dataTransferServiceUUID };

// Characteristic "Payload" Properties (for declaration)
static uint8_t dataTransferService_PayloadProps = GATT_PROP_READ;

// Characteristic "Payload" Value variable
static uint8_t dataTransferService_PayloadVal[DATATRANSFERSERVICE_PAYLOAD_LEN] = {0};
// Characteristic "PayloadCopy" Properties (for declaration)
static uint8_t dataTransferService_PayloadCopyProps = GATT_PROP_READ;

// Characteristic "PayloadCopy" Value variable
static uint8_t dataTransferService_PayloadCopyVal[DATATRANSFERSERVICE_PAYLOADCOPY_LEN] = {0};
// Characteristic "PayloadValid" Properties (for declaration)
static uint8_t dataTransferService_PayloadValidProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// Characteristic "PayloadValid" Value variable
static uint8_t dataTransferService_PayloadValidVal[DATATRANSFERSERVICE_PAYLOADVALID_LEN] = {0};

// Characteristic "PayloadValid" CCCD
static gattCharCfg_t *dataTransferService_PayloadValidConfig;
// Characteristic "PayloadCopyValid" Properties (for declaration)
static uint8_t dataTransferService_PayloadCopyValidProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// Characteristic "PayloadCopyValid" Value variable
static uint8_t dataTransferService_PayloadCopyValidVal[DATATRANSFERSERVICE_PAYLOADCOPYVALID_LEN] = {0};

// Characteristic "PayloadCopyValid" CCCD
static gattCharCfg_t *dataTransferService_PayloadCopyValidConfig;
// Characteristic "Request" Properties (for declaration)
static uint8_t dataTransferService_RequestProps = GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// Characteristic "Request" Value variable
static uint8_t dataTransferService_RequestVal[DATATRANSFERSERVICE_REQUEST_LEN] = {0};

// Characteristic "Request" CCCD
static gattCharCfg_t *dataTransferService_RequestConfig;
// Characteristic "EndOfTransmission" Properties (for declaration)
static uint8_t dataTransferService_EndOfTransmissionProps = GATT_PROP_READ;

// Characteristic "EndOfTransmission" Value variable
static uint8_t dataTransferService_EndOfTransmissionVal[DATATRANSFERSERVICE_ENDOFTRANSMISSION_LEN] = {0};
// Characteristic "Erase" Properties (for declaration)
static uint8_t dataTransferService_EraseProps = GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// Characteristic "Erase" Value variable
static uint8_t dataTransferService_EraseVal[DATATRANSFERSERVICE_ERASE_LEN] = {0};

// Characteristic "Erase" CCCD
static gattCharCfg_t *dataTransferService_EraseConfig;
// Characteristic "WriteInput" Properties (for declaration)
static uint8_t dataTransferService_WriteInputProps = GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// Characteristic "WriteInput" Value variable
static uint8_t dataTransferService_WriteInputVal[DATATRANSFERSERVICE_WRITEINPUT_LEN] = {0};

// Characteristic "WriteInput" CCCD
static gattCharCfg_t *dataTransferService_WriteInputConfig;
// Characteristic "WriteACK" Properties (for declaration)
static uint8_t dataTransferService_WriteACKProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// Characteristic "WriteACK" Value variable
static uint8_t dataTransferService_WriteACKVal[DATATRANSFERSERVICE_WRITEACK_LEN] = {0};

// Characteristic "WriteACK" CCCD
static gattCharCfg_t *dataTransferService_WriteACKConfig;

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t dataTransferServiceAttrTbl[] =
{
  // dataTransferService Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&dataTransferServiceDecl
  },
    // Payload Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &dataTransferService_PayloadProps
    },
      // Payload Characteristic Value
      {
        { ATT_BT_UUID_SIZE, dataTransferService_PayloadUUID },
        GATT_PERMIT_READ,
        0,
        dataTransferService_PayloadVal
      },
    // PayloadCopy Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &dataTransferService_PayloadCopyProps
    },
      // PayloadCopy Characteristic Value
      {
        { ATT_BT_UUID_SIZE, dataTransferService_PayloadCopyUUID },
        GATT_PERMIT_READ,
        0,
        dataTransferService_PayloadCopyVal
      },
    // PayloadValid Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &dataTransferService_PayloadValidProps
    },
      // PayloadValid Characteristic Value
      {
        { ATT_BT_UUID_SIZE, dataTransferService_PayloadValidUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        dataTransferService_PayloadValidVal
      },
      // PayloadValid CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&dataTransferService_PayloadValidConfig
      },
    // PayloadCopyValid Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &dataTransferService_PayloadCopyValidProps
    },
      // PayloadCopyValid Characteristic Value
      {
        { ATT_BT_UUID_SIZE, dataTransferService_PayloadCopyValidUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        dataTransferService_PayloadCopyValidVal
      },
      // PayloadCopyValid CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&dataTransferService_PayloadCopyValidConfig
      },
    // Request Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &dataTransferService_RequestProps
    },
      // Request Characteristic Value
      {
        { ATT_BT_UUID_SIZE, dataTransferService_RequestUUID },
        GATT_PERMIT_WRITE,
        0,
        dataTransferService_RequestVal
      },
      // Request CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&dataTransferService_RequestConfig
      },
    // EndOfTransmission Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &dataTransferService_EndOfTransmissionProps
    },
      // EndOfTransmission Characteristic Value
      {
        { ATT_BT_UUID_SIZE, dataTransferService_EndOfTransmissionUUID },
        GATT_PERMIT_READ,
        0,
        dataTransferService_EndOfTransmissionVal
      },
    // Erase Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &dataTransferService_EraseProps
    },
      // Erase Characteristic Value
      {
        { ATT_BT_UUID_SIZE, dataTransferService_EraseUUID },
        GATT_PERMIT_WRITE,
        0,
        dataTransferService_EraseVal
      },
      // Erase CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&dataTransferService_EraseConfig
      },
    // WriteInput Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &dataTransferService_WriteInputProps
    },
      // WriteInput Characteristic Value
      {
        { ATT_BT_UUID_SIZE, dataTransferService_WriteInputUUID },
        GATT_PERMIT_WRITE,
        0,
        dataTransferService_WriteInputVal
      },
      // WriteInput CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&dataTransferService_WriteInputConfig
      },
    // WriteACK Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &dataTransferService_WriteACKProps
    },
      // WriteACK Characteristic Value
      {
        { ATT_BT_UUID_SIZE, dataTransferService_WriteACKUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        dataTransferService_WriteACKVal
      },
      // WriteACK CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&dataTransferService_WriteACKConfig
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t dataTransferService_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 *pLen, uint16 offset,
                                           uint16 maxLen, uint8 method );
static bStatus_t dataTransferService_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint16 len, uint16 offset,
                                            uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t dataTransferServiceCBs =
{
  dataTransferService_ReadAttrCB,  // Read callback function pointer
  dataTransferService_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * DataTransferService_AddService- Initializes the DataTransferService service by registering
 *          GATT attributes with the GATT server.
 *
 */
bStatus_t DataTransferService_AddService( void )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  dataTransferService_PayloadValidConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( dataTransferService_PayloadValidConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, dataTransferService_PayloadValidConfig );
  // Allocate Client Characteristic Configuration table
  dataTransferService_PayloadCopyValidConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( dataTransferService_PayloadCopyValidConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, dataTransferService_PayloadCopyValidConfig );
  // Allocate Client Characteristic Configuration table
  dataTransferService_RequestConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( dataTransferService_RequestConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, dataTransferService_RequestConfig );
  // Allocate Client Characteristic Configuration table
  dataTransferService_EraseConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( dataTransferService_EraseConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, dataTransferService_EraseConfig );
  // Allocate Client Characteristic Configuration table
  dataTransferService_WriteInputConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( dataTransferService_WriteInputConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, dataTransferService_WriteInputConfig );
  // Allocate Client Characteristic Configuration table
  dataTransferService_WriteACKConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( dataTransferService_WriteACKConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, dataTransferService_WriteACKConfig );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( dataTransferServiceAttrTbl,
                                        GATT_NUM_ATTRS( dataTransferServiceAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &dataTransferServiceCBs );

  return ( status );
}

/*
 * DataTransferService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t DataTransferService_RegisterAppCBs( dataTransferServiceCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*
 * DataTransferService_SetParameter - Set a DataTransferService parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t DataTransferService_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case DATATRANSFERSERVICE_PAYLOAD:
      if ( len <= DATATRANSFERSERVICE_PAYLOAD_LEN ) //TODO I modified this to be <= from == (LEON)
      {
        memcpy(dataTransferService_PayloadVal, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DATATRANSFERSERVICE_PAYLOADCOPY:
      if ( len == DATATRANSFERSERVICE_PAYLOADCOPY_LEN )
      {
        memcpy(dataTransferService_PayloadCopyVal, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DATATRANSFERSERVICE_PAYLOADVALID:
      if ( len == DATATRANSFERSERVICE_PAYLOADVALID_LEN )
      {
        memcpy(dataTransferService_PayloadValidVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( dataTransferService_PayloadValidConfig, (uint8_t *)&dataTransferService_PayloadValidVal, FALSE,
                                    dataTransferServiceAttrTbl, GATT_NUM_ATTRS( dataTransferServiceAttrTbl ),
                                    INVALID_TASK_ID,  dataTransferService_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DATATRANSFERSERVICE_PAYLOADCOPYVALID:
      if ( len == DATATRANSFERSERVICE_PAYLOADCOPYVALID_LEN )
      {
        memcpy(dataTransferService_PayloadCopyValidVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( dataTransferService_PayloadCopyValidConfig, (uint8_t *)&dataTransferService_PayloadCopyValidVal, FALSE,
                                    dataTransferServiceAttrTbl, GATT_NUM_ATTRS( dataTransferServiceAttrTbl ),
                                    INVALID_TASK_ID,  dataTransferService_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DATATRANSFERSERVICE_REQUEST:
      if ( len == DATATRANSFERSERVICE_REQUEST_LEN )
      {
        memcpy(dataTransferService_RequestVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( dataTransferService_RequestConfig, (uint8_t *)&dataTransferService_RequestVal, FALSE,
                                    dataTransferServiceAttrTbl, GATT_NUM_ATTRS( dataTransferServiceAttrTbl ),
                                    INVALID_TASK_ID,  dataTransferService_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DATATRANSFERSERVICE_ENDOFTRANSMISSION:
      if ( len == DATATRANSFERSERVICE_ENDOFTRANSMISSION_LEN )
      {
        memcpy(dataTransferService_EndOfTransmissionVal, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DATATRANSFERSERVICE_ERASE:
      if ( len == DATATRANSFERSERVICE_ERASE_LEN )
      {
        memcpy(dataTransferService_EraseVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( dataTransferService_EraseConfig, (uint8_t *)&dataTransferService_EraseVal, FALSE,
                                    dataTransferServiceAttrTbl, GATT_NUM_ATTRS( dataTransferServiceAttrTbl ),
                                    INVALID_TASK_ID,  dataTransferService_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DATATRANSFERSERVICE_WRITEINPUT:
      if ( len == DATATRANSFERSERVICE_WRITEINPUT_LEN )
      {
        memcpy(dataTransferService_WriteInputVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( dataTransferService_WriteInputConfig, (uint8_t *)&dataTransferService_WriteInputVal, FALSE,
                                    dataTransferServiceAttrTbl, GATT_NUM_ATTRS( dataTransferServiceAttrTbl ),
                                    INVALID_TASK_ID,  dataTransferService_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DATATRANSFERSERVICE_WRITEACK:
      if ( len == DATATRANSFERSERVICE_WRITEACK_LEN )
      {
        memcpy(dataTransferService_WriteACKVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( dataTransferService_WriteACKConfig, (uint8_t *)&dataTransferService_WriteACKVal, FALSE,
                                    dataTransferServiceAttrTbl, GATT_NUM_ATTRS( dataTransferServiceAttrTbl ),
                                    INVALID_TASK_ID,  dataTransferService_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*
 * DataTransferService_GetParameter - Get a DataTransferService parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t DataTransferService_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case DATATRANSFERSERVICE_PAYLOADVALID:
      memcpy(value, dataTransferService_PayloadValidVal, DATATRANSFERSERVICE_PAYLOADVALID_LEN);
      break;

    case DATATRANSFERSERVICE_PAYLOADCOPYVALID:
      memcpy(value, dataTransferService_PayloadCopyValidVal, DATATRANSFERSERVICE_PAYLOADCOPYVALID_LEN);
      break;

    case DATATRANSFERSERVICE_REQUEST:
      memcpy(value, dataTransferService_RequestVal, DATATRANSFERSERVICE_REQUEST_LEN);
      break;

    case DATATRANSFERSERVICE_ERASE:
      memcpy(value, dataTransferService_EraseVal, DATATRANSFERSERVICE_ERASE_LEN);
      break;

    case DATATRANSFERSERVICE_WRITEINPUT:
      memcpy(value, dataTransferService_WriteInputVal, DATATRANSFERSERVICE_WRITEINPUT_LEN);
      break;

    case DATATRANSFERSERVICE_WRITEACK:
      memcpy(value, dataTransferService_WriteACKVal, DATATRANSFERSERVICE_WRITEACK_LEN);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          dataTransferService_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t dataTransferService_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                       uint8 *pValue, uint16 *pLen, uint16 offset,
                                       uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the Payload Characteristic Value
if ( ! memcmp(pAttr->type.uuid, dataTransferService_PayloadUUID, pAttr->type.len) )
  {
    if ( offset > DATATRANSFERSERVICE_PAYLOAD_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, DATATRANSFERSERVICE_PAYLOAD_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the PayloadCopy Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, dataTransferService_PayloadCopyUUID, pAttr->type.len) )
  {
    if ( offset > DATATRANSFERSERVICE_PAYLOADCOPY_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, DATATRANSFERSERVICE_PAYLOADCOPY_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the PayloadValid Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, dataTransferService_PayloadValidUUID, pAttr->type.len) )
  {
    if ( offset > DATATRANSFERSERVICE_PAYLOADVALID_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, DATATRANSFERSERVICE_PAYLOADVALID_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the PayloadCopyValid Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, dataTransferService_PayloadCopyValidUUID, pAttr->type.len) )
  {
    if ( offset > DATATRANSFERSERVICE_PAYLOADCOPYVALID_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, DATATRANSFERSERVICE_PAYLOADCOPYVALID_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Request Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, dataTransferService_RequestUUID, pAttr->type.len) )
  {
    if ( offset > DATATRANSFERSERVICE_REQUEST_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, DATATRANSFERSERVICE_REQUEST_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the EndOfTransmission Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, dataTransferService_EndOfTransmissionUUID, pAttr->type.len) )
  {
    if ( offset > DATATRANSFERSERVICE_ENDOFTRANSMISSION_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, DATATRANSFERSERVICE_ENDOFTRANSMISSION_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Erase Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, dataTransferService_EraseUUID, pAttr->type.len) )
  {
    if ( offset > DATATRANSFERSERVICE_ERASE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, DATATRANSFERSERVICE_ERASE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the WriteInput Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, dataTransferService_WriteInputUUID, pAttr->type.len) )
  {
    if ( offset > DATATRANSFERSERVICE_WRITEINPUT_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, DATATRANSFERSERVICE_WRITEINPUT_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the WriteACK Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, dataTransferService_WriteACKUUID, pAttr->type.len) )
  {
    if ( offset > DATATRANSFERSERVICE_WRITEACK_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, DATATRANSFERSERVICE_WRITEACK_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has READ permissions.
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}


/*********************************************************************
 * @fn      dataTransferService_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t dataTransferService_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                        uint8 *pValue, uint16 len, uint16 offset,
                                        uint8 method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;

  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  // See if request is regarding the PayloadValid Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, dataTransferService_PayloadValidUUID, pAttr->type.len) )
  {
    if ( offset + len > DATATRANSFERSERVICE_PAYLOADVALID_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == DATATRANSFERSERVICE_PAYLOADVALID_LEN)
        paramID = DATATRANSFERSERVICE_PAYLOADVALID;
    }
  }
  // See if request is regarding the PayloadCopyValid Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, dataTransferService_PayloadCopyValidUUID, pAttr->type.len) )
  {
    if ( offset + len > DATATRANSFERSERVICE_PAYLOADCOPYVALID_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == DATATRANSFERSERVICE_PAYLOADCOPYVALID_LEN)
        paramID = DATATRANSFERSERVICE_PAYLOADCOPYVALID;
    }
  }
  // See if request is regarding the Request Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, dataTransferService_RequestUUID, pAttr->type.len) )
  {
    if ( offset + len > DATATRANSFERSERVICE_REQUEST_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == DATATRANSFERSERVICE_REQUEST_LEN)
        paramID = DATATRANSFERSERVICE_REQUEST;
    }
  }
  // See if request is regarding the Erase Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, dataTransferService_EraseUUID, pAttr->type.len) )
  {
    if ( offset + len > DATATRANSFERSERVICE_ERASE_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == DATATRANSFERSERVICE_ERASE_LEN)
        paramID = DATATRANSFERSERVICE_ERASE;
    }
  }
  // See if request is regarding the WriteInput Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, dataTransferService_WriteInputUUID, pAttr->type.len) )
  {
    if ( offset + len > DATATRANSFERSERVICE_WRITEINPUT_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == DATATRANSFERSERVICE_WRITEINPUT_LEN)
        paramID = DATATRANSFERSERVICE_WRITEINPUT;
    }
  }
  // See if request is regarding the WriteACK Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, dataTransferService_WriteACKUUID, pAttr->type.len) )
  {
    if ( offset + len > DATATRANSFERSERVICE_WRITEACK_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == DATATRANSFERSERVICE_WRITEACK_LEN)
        paramID = DATATRANSFERSERVICE_WRITEACK;
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has WRITE permissions.
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
  if (paramID != 0xFF)
    if ( pAppCBs && pAppCBs->pfnChangeCb )
      pAppCBs->pfnChangeCb( paramID ); // Call app function from stack task context.

  return status;
}
