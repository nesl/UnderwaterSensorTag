/**********************************************************************************************
 * Filename:       dataTransferService.h
 *
 * Description:    This file contains the dataTransferService service definitions and
 *                 prototypes.
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


#ifndef _DATATRANSFERSERVICE_H_
#define _DATATRANSFERSERVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
* CONSTANTS
*/
// Service UUID
#define DATATRANSFERSERVICE_SERV_UUID 0xBABE

//  Characteristic defines
#define DATATRANSFERSERVICE_PAYLOAD      0
#define DATATRANSFERSERVICE_PAYLOAD_UUID 0xBE00
#define DATATRANSFERSERVICE_PAYLOAD_LEN  512

//  Characteristic defines
#define DATATRANSFERSERVICE_PAYLOADCOPY      1
#define DATATRANSFERSERVICE_PAYLOADCOPY_UUID 0xBE01
#define DATATRANSFERSERVICE_PAYLOADCOPY_LEN  512

//  Characteristic defines
#define DATATRANSFERSERVICE_PAYLOADVALID      2
#define DATATRANSFERSERVICE_PAYLOADVALID_UUID 0xBE02
#define DATATRANSFERSERVICE_PAYLOADVALID_LEN  1

//  Characteristic defines
#define DATATRANSFERSERVICE_PAYLOADCOPYVALID      3
#define DATATRANSFERSERVICE_PAYLOADCOPYVALID_UUID 0xBE03
#define DATATRANSFERSERVICE_PAYLOADCOPYVALID_LEN  1

//  Characteristic defines
#define DATATRANSFERSERVICE_REQUEST      4
#define DATATRANSFERSERVICE_REQUEST_UUID 0xBE04
#define DATATRANSFERSERVICE_REQUEST_LEN  1

//  Characteristic defines
#define DATATRANSFERSERVICE_ENDOFTRANSMISSION      5
#define DATATRANSFERSERVICE_ENDOFTRANSMISSION_UUID 0xBE05
#define DATATRANSFERSERVICE_ENDOFTRANSMISSION_LEN  1

//  Characteristic defines
#define DATATRANSFERSERVICE_ERASE      6
#define DATATRANSFERSERVICE_ERASE_UUID 0xBE06
#define DATATRANSFERSERVICE_ERASE_LEN  1

//  Characteristic defines
#define DATATRANSFERSERVICE_WRITEINPUT      7
#define DATATRANSFERSERVICE_WRITEINPUT_UUID 0xBE07
#define DATATRANSFERSERVICE_WRITEINPUT_LEN  8

//  Characteristic defines
#define DATATRANSFERSERVICE_WRITEACK      8
#define DATATRANSFERSERVICE_WRITEACK_UUID 0xBE08
#define DATATRANSFERSERVICE_WRITEACK_LEN  1

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*dataTransferServiceChange_t)( uint8 paramID );

typedef struct
{
  dataTransferServiceChange_t        pfnChangeCb;  // Called when characteristic value changes
} dataTransferServiceCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * DataTransferService_AddService- Initializes the DataTransferService service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t DataTransferService_AddService( void );

/*
 * DataTransferService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t DataTransferService_RegisterAppCBs( dataTransferServiceCBs_t *appCallbacks );

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
extern bStatus_t DataTransferService_SetParameter( uint8 param, uint8 len, void *value );

/*
 * DataTransferService_GetParameter - Get a DataTransferService parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t DataTransferService_GetParameter( uint8 param, void *value );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _DATATRANSFERSERVICE_H_ */