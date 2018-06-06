/*
 * Filename: spp_ble_server.c
 *
 * Description: This is the simple_peripheral example modified to send
 * data over BLE at a high throughput.
 *
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "hci_tl.h"
#include "gatt.h"
#include "linkdb.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

//#include <ti/mw/display/Display.h>
#include "board_key.h"
#include "board.h"

#include "inc/sdi_tl_uart.h"
#include "serial_port_service.h"
#include "spp_ble_server.h"
#include "inc/sdi_task.h"
//esl #include "inc/sdi_tl_uart.h"

/********************************EUNSUN ******************************/
//#include "dataTransferService.h"
/*********************************Includes from Previous Version 1/10/2018 *********/
//#include <ti/drivers/PIN.h>
//#include <ti/mw/display/Display.h>
//#include <gap.h>
#include <ti/sysbios/BIOS.h>
//esl #include <ti/drivers/UART.h>
#include "aon_rtc.h"
#include "scif.h"
#include <ti/drivers/SPI.h>
//114#include "dataTransferService.h"
#define BV(x)    (1 << (x))
//#include "bcomdef.h"
#define BUF_LEN 7
#define SNV_ID_APP 0x80
uint8 buf[BUF_LEN] = {0,0,0,0,0,0,0};

#define VER_3_0 0

#ifdef VER_3_0
#define STORAGE_MAX_ADDRESS 0x0001FFFF
#endif

#ifdef VER_3_1
#define STORAGE_MAX_ADDRESS 0x0000FFFF
#endif



 /********************************End previous includes*****************************/
/*********************************************************************/

/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     16

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     16
#else //!FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8
#endif // FEATURE_OAD

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          200

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               5000

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define SBP_TASK_PRIORITY                     1


#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   644
#endif

// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_PERIODIC_EVT                      0x0004
#define SBP_CONN_EVT_END_EVT                  0x0008
#define SBP_KEY_CHANGE_EVT                    0x0010

#define APP_SUGGESTED_PDU_SIZE 251
#define APP_SUGGESTED_TX_TIME 2120

/*********************************************************************
 * TYPEDEFS
 */
// RTOS queue for profile/app messages.
typedef struct _queueRec_
{
  Queue_Elem _elem;          // queue element
  uint8_t *pData;            // pointer to app data
} queueRec_t;

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
} sbpEvt_t;

// App event passed from profiles.
//esl //typedef struct
//{
//  uint8_t event;  // Type of event
//  uint8_t *pData;  // New data
//  uint8_t length; // New status
//} sbpUARTEvt_t;
/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
//Display_Handle dispHandle = NULL;

// Global pin resources
PIN_State pinGpioState;
PIN_Handle hGpioPin;

uint16 currentMTUSize;

uint8_t runOnce = 0;
//esl uint8_t thePayload[128];
uint8_t phone = 0;

/********************************EUNSUN ******************************/
int selectedMode = 0;
int readresult = 0;
static UInt peripheralNum = 0;
static SPI_Params spiParams;
static SPI_Handle spi;

static PIN_Handle CSPinHandle;
static PIN_State CSPinState;
#define PIN_SPI0_CS IOID_7

const PIN_Config CSPinTable[] = {
    PIN_SPI0_CS   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH   | PIN_PUSHPULL,
    PIN_TERMINATE
};

uint32_t pageblockadd = 0x00000000;
uint32_t lastpagewritten = 0;
SPI_Transaction spiTransaction;
uint16_t columnadd = 0x0000;
uint8_t passArray[2048] = {[0 ... 2047] = 0x00};
//uint8_t passArray_temp[300] = {[0 ... 299] = 0x00};
int countarray = 0;
uint32_t sec;
uint32_t frac;

uint8_t valueReceived = 0;
int countapage=0;

Semaphore_Handle hSemMainLoop;
Semaphore_Struct semMainLoop;
//uint8_t tempval = 1;
#define LED_PIN IOID_9
PIN_Handle ledPinHandle;
PIN_State ledPinState;
PIN_Config ledPinTable[] = {
    LED_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};


uint8_t numSCBuf =0;
uint8_t runOnce2 =0;
//uint8 bdAddress[B_ADDR_LEN] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05 };

#define AQUAMOTE_0 0

#ifdef AQUAMOTE_0
uint8_t bdAddress[B_ADDR_LEN] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x5};
#endif

#ifdef AQUAMOTE_1
uint8_t bdAddress[B_ADDR_LEN] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x6};
#endif

#ifdef AQUAMOTE_2
uint8_t bdAddress[B_ADDR_LEN] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x7};
#endif

#ifdef AQUAMOTE_3
uint8_t bdAddress[B_ADDR_LEN] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x8};
#endif

#ifdef AQUAMOTE_4
uint8_t bdAddress[B_ADDR_LEN] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x9};
#endif

#ifdef AQUAMOTE_5
uint8_t bdAddress[B_ADDR_LEN] = {0x0, 0x1, 0x2, 0x3, 0x4, 0xA};
#endif

#ifdef AQUAMOTE_6
uint8_t bdAddress[B_ADDR_LEN] = {0x0, 0x1, 0x2, 0x3, 0x4, 0xB};
#endif

#ifdef AQUAMOTE_7
uint8_t bdAddress[B_ADDR_LEN] = {0x0, 0x1, 0x2, 0x3, 0x4, 0xC};
#endif

#ifdef AQUAMOTE_8
uint8_t bdAddress[B_ADDR_LEN] = {0x0, 0x1, 0x2, 0x3, 0x4, 0xD};
#endif

#ifdef AQUAMOTE_9
uint8_t bdAddress[B_ADDR_LEN] = {0x0, 0x1, 0x2, 0x3, 0x4, 0xE};
#endif



#define ONGPS IOID_8
PIN_Handle ongpsPinHandle;
PIN_State ongpsPinState;
char input;
static UART_Handle uart;
static UART_Params uartParams;
uint8_t GPGGAinput[60];
int mloc;

PIN_Config ongpsPinTable[] = {
     ONGPS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
/*********************************************************************/

/*********************************************************************
 * LOCAL VARIABLES
 */

/********************************EUNSUN ******************************/
void GPScollect(UArg arg0);
static void DataTransferSession(void);
static void EndOfTransmission(void);
void SPI_params_init();
int readfromapage(UArg arg0);

//
static void SimpleBLEPeripheral_performPeriodicTask(void);
//
//void GPScollect(UArg arg0);
void scCtrlReadyCallback(void);
void scTaskAlertCallback(void);
int Mem_SF(uint8_t);
//void Mem_lock();
void processTaskAlert(UArg arg0);
void savetoapage(UArg arg0);
void createarray();
//
void mem_fin(int len);
//
void CS_init();
void CS_LOW();
void CS_HIGH();
//int Mem_ID_Check();
int Mem_GF();
int Mem_RtC();
void Mem_BE();
void Mem_PP();
void Mem_PP2();
void Mem_WE();
void blockerasefxn();
void getlastpageadd();
void savelastpageadd();
/*********************************************************************/

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Queue object used for UART messages
//static Queue_Struct appUARTMsg;
//static Queue_Handle appUARTMsgQueue;

//esl2
//#if defined(FEATURE_OAD)
//// Event data from OAD profile.
//static Queue_Struct oadQ;
//static Queue_Handle hOadQ;
//#endif //FEATURE_OAD

// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

int connected = 0;

// Value to write
static uint8_t charVal = 0x41;

// Profile state and parameters
static gaprole_States_t gapProfileState = GAPROLE_INIT;

// Connection handle of current connection
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x0F,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'S',
  'P',
  'P',
  ' ',
  'B',
  'L',
  'E',
  ' ',
  'S',
  'E',
  'R',
  'V',
  'E',
  'R',

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
#if !defined(FEATURE_OAD) || defined(FEATURE_OAD_ONCHIP)
  0x03,   // length of this data
#else //OAD for external flash
  0x05,  // lenght of this data
#endif //FEATURE_OAD
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
#ifdef FEATURE_OAD
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID),
#endif //FEATURE_OAD
#ifndef FEATURE_OAD_ONCHIP
  LO_UINT16(SERIALPORTSERVICE_SERV_UUID),
  HI_UINT16(SERIALPORTSERVICE_SERV_UUID)
#endif //FEATURE_OAD_ONCHIP
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "SPP BLE Server";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// Pins that are actively used by the application
static PIN_Config SPPBLEAppPinTable[] =
{
    Board_RLED       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off             */
    Board_GLED       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off             */

    PIN_TERMINATE
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SPPBLEServer_init( void );
static void SPPBLEServer_taskFxn(UArg arg0, UArg a1);
static void SPPBLEServer_handleKeys(uint8_t shift, uint8_t keys);
static uint8_t SPPBLEServer_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SPPBLEServer_processGATTMsg(gattMsgEvent_t *pMsg);
static void SPPBLEServer_processAppMsg(sbpEvt_t *pMsg);
static void SPPBLEServer_processStateChangeEvt(gaprole_States_t newState);
static void SPPBLEServer_processCharValueChangeEvt(uint8_t paramID);
static void SPPBLEServer_performPeriodicTask(void);
static void SPPBLEServer_clockHandler(UArg arg);

static void SPPBLEServer_sendAttRsp(void);
static void SPPBLEServer_freeAttRsp(uint8_t status);

static void SPPBLEServer_stateChangeCB(gaprole_States_t newState);
#ifndef FEATURE_OAD_ONCHIP
static void SPPBLEServer_charValueChangeCB(uint8_t paramID);
#endif //!FEATURE_OAD_ONCHIP
static void SPPBLEServer_enqueueMsg(uint8_t event, uint8_t state);
//esl void SPPBLEServer_enqueueUARTMsg(uint8_t event, uint8_t *data, uint8_t len);
//esl2 #ifdef FEATURE_OAD
//void SPPBLEServer_processOadWriteCB(uint8_t event, uint16_t connHandle,
//                                           uint8_t *pData);
//#endif //FEATURE_OAD
char* convInt32ToText(int32 value);
void SPPBLEServer_keyChangeHandler(uint8 keys);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t SPPBLEServer_gapRoleCBs =
{
  SPPBLEServer_stateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t SPPBLEServer_BondMgrCBs =
{
  NULL, // Passcode callback (not used by application)
  NULL  // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
#ifndef FEATURE_OAD_ONCHIP
static SerialPortServiceCBs_t SPPBLEServer_SerialPortService_CBs =
{
  SPPBLEServer_charValueChangeCB // Characteristic value change callback
};
#endif //!FEATURE_OAD_ONCHIP

//esl2
//#ifdef FEATURE_OAD
//static oadTargetCBs_t SPPBLEServer_oadCBs =
//{
//  //esl2 SPPBLEServer_processOadWriteCB // Write Callback.
//};
//#endif //FEATURE_OAD

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SPPBLEServer_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void SPPBLEServer_createTask(void)
{


  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;
  taskParams.arg0 = 1000000 / Clock_tickPeriod;

  Task_construct(&sbpTask, SPPBLEServer_taskFxn, &taskParams, NULL);

  //EUNSUN
  // Semaphore initialization
  Semaphore_Params semParams;
  Semaphore_Params_init(&semParams);
  Semaphore_construct(&semMainLoop, 0, &semParams);
  hSemMainLoop = Semaphore_handle(&semMainLoop);


  ledPinHandle = PIN_open(&ledPinState, ledPinTable);
  if(!ledPinHandle) {
      //System_abort("Error initializing board LED pins\n");
  }

}

/*******************************************************************************
 * @fn      SPPBLEServer_blinkLed
 *
 * @brief   Blinks a led 'n' times, duty-cycle 50-50
 * @param   led - led identifier
 * @param   nBlinks - number of blinks
 *
 * @return  none
 */
void SPPBLEServer_blinkLed(uint8_t led, uint8_t nBlinks)
{
  uint8_t i;

  for (i=0; i<nBlinks; i++)
  {
    PIN_setOutputValue(hGpioPin, led, 1);
    delay_ms(BLINK_DURATION);
    PIN_setOutputValue(hGpioPin, led, 0);
    delay_ms(BLINK_DURATION);
  }
}

/*******************************************************************************
 * @fn      SPPBLEClient_toggleLed
 *
 * @brief   Blinks a led 'n' times, duty-cycle 50-50
 * @param   led - led identifier
 * @param   nBlinks - number of blinks
 *
 * @return  none
 */
void SPPBLEServer_toggleLed(uint8_t led, uint8_t state)
{
    uint8_t nextLEDstate = 0;

    if(state == Board_LED_TOGGLE)
    {
      nextLEDstate = !(PIN_getOutputValue(led));
    }
    else
    {
      nextLEDstate = state;
    }

    PIN_setOutputValue(hGpioPin, led, nextLEDstate);
}

/*********************************************************************
 * @fn      SPPBLEServer_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SPPBLEServer_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  //TODO: comment the code below
  // Hard code the BD Address till CC2650 board gets its own IEEE address
  HCI_EXT_SetBDADDRCmd(bdAddress);

  // Handling of LED
  hGpioPin = PIN_open(&pinGpioState, SPPBLEAppPinTable);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);
  //appUARTMsgQueue = Util_constructQueue(&appUARTMsg);
  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, SPPBLEServer_clockHandler,
                      SBP_PERIODIC_EVT_PERIOD, 0, false, SBP_PERIODIC_EVT);

  Board_initKeys(SPPBLEServer_keyChangeHandler);

//  dispHandle = Display_open(Display_Type_UART, NULL);

  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = 0; // passkey "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

#ifndef FEATURE_OAD_ONCHIP
  //SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
  SerialPortService_AddService(GATT_ALL_SERVICES);  //SerialPortBLE service
#endif //!FEATURE_OAD_ONCHIP

  //114DataTransferService_AddService();

//esl2
//#ifdef FEATURE_OAD
//  VOID OAD_addService();                 // OAD Profile
//  OAD_register((oadTargetCBs_t *)&SPPBLEServer_oadCBs);
//  hOadQ = Util_constructQueue(&oadQ);
//#endif //FEATURE_OAD

#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE

#ifndef FEATURE_OAD_ONCHIP
  // Setup the Profile Characteristic Values
  {

  }

  // Register callback with SimpleGATTprofile
  SerialPortService_RegisterAppCBs(&SPPBLEServer_SerialPortService_CBs);
#endif //!FEATURE_OAD_ONCHIP

  // Start the Device
  VOID GAPRole_StartDevice(&SPPBLEServer_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&SPPBLEServer_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  //Register to receive UART messages
//esl   SDITask_registerIncomingRXEventAppCB(SPPBLEServer_enqueueUARTMsg);

  //esl uint8_t hello[] = "Hello from SPP BLE Server! With Data Length Extension support!\n\r";
  //esl DEBUG(hello);

#if defined FEATURE_OAD
#if defined (HAL_IMAGE_A)
  //Display_print0(dispHandle, 0, 0, "BLE Peripheral A");
#else
  //Display_print0(dispHandle, 0, 0, "BLE Peripheral B");
#endif // HAL_IMAGE_A
#else
  //Display_print0(dispHandle, 0, 0, "SPP BLE Server");
#endif // FEATURE_OAD

  SPPBLEServer_blinkLed(Board_RLED, 1);
  Board_initSPI();
  SPI_params_init();
//  passArray2[0] = 1;
//  if(passArray2[0] == 1){
//      DEBUG("passarray2 ");
//  }
}

/*********************************************************************
 * @fn      SPPBLEServer_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */



static void SPPBLEServer_taskFxn(UArg arg0, UArg a1)
{
  PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
  Task_sleep((UInt)arg0);
  PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
  Task_sleep((UInt)arg0);
  PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
  Task_sleep((UInt)arg0);
  PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
  Task_sleep((UInt)arg0);

  // Initialize application
  SPPBLEServer_init();
  //SCS Initialization
  scifOsalInit();
  scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
  scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
  scifInit(&scifDriverSetup);
  // Set the Sensor Controller task tick interval to 1 second
  uint32_t rtc_Hz = 40;  // 1Hz RTC
  scifStartRtcTicksNow(0x00010000 / rtc_Hz);
  scifStartTasksNbl(BV(SCIF_I2CIMUPRESSUREWARRAY1111_TASK_ID));

  //GPScollect(arg0);

//    uint8_t nv_status = NV_OPER_FAILED;
//
//    do{
//        nv_status = osal_snv_read(SNV_ID_APP,BUF_LEN, (uint8 *)buf);
//    }while(nv_status != SUCCESS);
//
//    if(buf[3] == 9 ){
//        while(1){
//            PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
//        }
//    }
//
//    if(buf[3] == 3){
//        PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
//        Task_sleep((UInt)arg0);
//        PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
//        Task_sleep((UInt)arg0);
//        PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
//        Task_sleep((UInt)arg0);
//        PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
//        Task_sleep((UInt)arg0);
//    }
//
//    //if buf3 is not 3, write 0 - 9
//
//    nv_status = NV_OPER_FAILED;
//    do{
//        nv_status = osal_snv_write(SNV_ID_APP,BUF_LEN,(uint8 *)buf);
//    }while(nv_status != SUCCESS);
//
//
//
//
 // Mem_SF(0x00);    //Unlock all blocks
 // blockerasefxn();
//
  //createarray();
  //savetoapage();
  //int es = 0;
  //for (es = 0; es <2048; es++){
  //    passArray[es] = 0;
  //}
  //savetoapage();

  //pageblockadd = 0;
  getlastpageadd();
  lastpagewritten = pageblockadd;
  pageblockadd = 0;
  //savelastpageadd();

  int chunkCounter = 0;
  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & SBP_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              SPPBLEServer_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = SPPBLEServer_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      phone = 1; //(124)
      if(phone == 2){
          if(runOnce < 16 && connected ==1 && valueReceived == 1&&readresult == 0)
                {
                    //First Send the Number of pages about to transfer
                    bStatus_t retVal3 = FAILURE;
                    if(runOnce2 == 0){
                        runOnce2 = 1;

                        do{
                            retVal3 = SerialPortService_SetParameter(SERIALPORTSERVICE_CHAR_DATA, B_ADDR_LEN , bdAddress);
                        }while(retVal3 != SUCCESS);
                        SerialPortService_AddStatusTXBytes(B_ADDR_LEN);

                        buf[0] = lastpagewritten & 0xFF;
                        buf[1] = (lastpagewritten >>  8) & 0xFF;
                        buf[2] = (lastpagewritten >> 16) & 0xFF;
                        buf[4] = (lastpagewritten >> 24) & 0xFF;

                        do{
                            retVal3 = SerialPortService_SetParameter(SERIALPORTSERVICE_CHAR_DATA, 4, buf);
                        }while(retVal3 != SUCCESS);

                        SerialPortService_AddStatusTXBytes(4);

                    }
                    if(runOnce == 0){
                        //Task_sleep(150000);
                        //load in the next page into a buffer: readfromapage called
                        readresult = readfromapage(arg0);
                    }
//                    //shift read page in chunks and enqueue them
//                    for(int ii =0; ii<126; ii++)
//                    {
//                        passArray[ii] = (runOnce % 26) + 0x41;
//                        //esl thePayload[ii] = (runOnce % 26) + 0x41;
//                    }

                    //SPPBLEServer_enqueueUARTMsg(SBP_UART_DATA_EVT, thePayload, 128);
                    retVal3 = FAILURE;
                    do{
                        retVal3 = SerialPortService_SetParameter(SERIALPORTSERVICE_CHAR_DATA, 128, (passArray + chunkCounter*128)); //esl thePayload);
                    }while(retVal3 != SUCCESS);
                    SerialPortService_AddStatusTXBytes(128);
                    valueReceived = 0;
                    //Task_sleep(6000);
                    runOnce++;
                    chunkCounter++;
                    //ICall_signal(sem);
                }else if(runOnce == 16 && connected ==1)
                {
//                    if(readresult == 1){    //checkforthelastpage
//                        EndOfTransmission();
//                        //DEBUG("ENDOFTRANSMISSION");
//                        while(1){
//                            PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
//                            Task_sleep((UInt)arg0);
//                            PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
//                            Task_sleep((UInt)arg0);
//                            PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
//                            Task_sleep((UInt)arg0);
//                            PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
//                            Task_sleep((UInt)arg0*2);
//                        }
//                    }

                    //readresult = 1;
                    chunkCounter = 0;
//                    passArray[0] = 'N';
//                    passArray[1] = 'E';
//                    passArray[2] = 'W';
//                    passArray[3] = ' ';
//                    passArray[4] = 'P';
//                    passArray[5] = 'A';
//                    passArray[6] = 'G';
//                    passArray[7] = 'E';
//                    passArray[8] = '\r';
//                    passArray[9] = '\n';
                    //ESL
//                    thePayload[0] = 'N';
//                    thePayload[1] = 'E';
//                    thePayload[2] = 'W';
//                    thePayload[3] = ' ';
//                    thePayload[4] = 'P';
//                    thePayload[5] = 'A';
//                    thePayload[6] = 'G';
//                    thePayload[7] = 'E';
//                    thePayload[8] = '\r';
//                    thePayload[9] = '\n';

                    //ESL
//                    bStatus_t retVal3 = FAILURE;
//                    do{
//                    retVal3 = SerialPortService_SetParameter(SERIALPORTSERVICE_CHAR_DATA, 10, passArray); //esl
//                    }while(retVal3 != SUCCESS);
//
//                    SerialPortService_AddStatusTXBytes(10);
                    //SPPBLEServer_enqueueUARTMsg(SBP_UART_DATA_EVT, thePayload, 10);

                    runOnce = 0;
                }


      }else if(phone == 1){
          //DEBUG("CHOSE A MODE");
          //114DataTransferSession();
          //phone = 0;

          SimpleBLEPeripheral_performPeriodicTask();//This turns off bluetooth

          SDITLUART_closeUART();

          Mem_SF(0x00);    //Unlock all blocks

          getlastpageadd(); //retrieve from non volatile storage the last page address
          pageblockadd++;

              //TODO: uncomment this GPS, it was commented to save time while testing
              //GPScollect(arg0);
          if(pageblockadd < 2){
              passArray[0] = scifTaskData.i2cimupressurewarray1111.output.pressureinit[0]>>8;
              passArray[1] = scifTaskData.i2cimupressurewarray1111.output.pressureinit[0];
              passArray[2] = scifTaskData.i2cimupressurewarray1111.output.pressureinit[1]>>8;
              passArray[3] = scifTaskData.i2cimupressurewarray1111.output.pressureinit[1];
              passArray[4] = scifTaskData.i2cimupressurewarray1111.output.pressureinit[2]>>8;
              passArray[5] = scifTaskData.i2cimupressurewarray1111.output.pressureinit[2];
              passArray[6] = scifTaskData.i2cimupressurewarray1111.output.pressureinit[3]>>8;
              passArray[7] = scifTaskData.i2cimupressurewarray1111.output.pressureinit[3];
              passArray[8] = scifTaskData.i2cimupressurewarray1111.output.pressureinit[4]>>8;
              passArray[9] = scifTaskData.i2cimupressurewarray1111.output.pressureinit[4];
              passArray[10] = scifTaskData.i2cimupressurewarray1111.output.pressureinit[5]>>8;
              passArray[11] = scifTaskData.i2cimupressurewarray1111.output.pressureinit[5];
              passArray[12] = scifTaskData.i2cimupressurewarray1111.output.pressureinit[6]>>8;
              passArray[13] = scifTaskData.i2cimupressurewarray1111.output.pressureinit[6];

//            passArray[0] = scifTaskData.i2cimupressurewarrayAbcd.output.pressureinit[0]>>8;
//            passArray[1] = scifTaskData.i2cimupressurewarrayAbcd.output.pressureinit[0];
//            passArray[2] = scifTaskData.i2cimupressurewarrayAbcd.output.pressureinit[1]>>8;
//            passArray[3] = scifTaskData.i2cimupressurewarrayAbcd.output.pressureinit[1];
//            passArray[4] = scifTaskData.i2cimupressurewarrayAbcd.output.pressureinit[2]>>8;
//            passArray[5] = scifTaskData.i2cimupressurewarrayAbcd.output.pressureinit[2];
//            passArray[6] = scifTaskData.i2cimupressurewarrayAbcd.output.pressureinit[3]>>8;
//            passArray[7] = scifTaskData.i2cimupressurewarrayAbcd.output.pressureinit[3];
//            passArray[8] = scifTaskData.i2cimupressurewarrayAbcd.output.pressureinit[4]>>8;
//            passArray[9] = scifTaskData.i2cimupressurewarrayAbcd.output.pressureinit[4];
//            passArray[10] = scifTaskData.i2cimupressurewarrayAbcd.output.pressureinit[5]>>8;
//            passArray[11] = scifTaskData.i2cimupressurewarrayAbcd.output.pressureinit[5];
//            passArray[12] = scifTaskData.i2cimupressurewarrayAbcd.output.pressureinit[6]>>8;
//            passArray[13] = scifTaskData.i2cimupressurewarrayAbcd.output.pressureinit[6];
            savetoapage(arg0);
          }


          passArray[2044] = 'G';
          passArray[2045] = 'G';
          passArray[2046] = '\r';
          passArray[2047] = '\n';

          //mem_fin(7);

          while(1){
              GPScollect(arg0);


              //update passArrray
//              for(int xyz=0; xyz<1024; xyz++)
//              {
//                  passArray[xyz*2] = (pageblockadd >> 8) & 0xFF;
//                  passArray[xyz*2 + 1] = pageblockadd & 0xFF;
//              }
//              savetoapage();

            //TODO: uncomment the lines(116)
//              Semaphore_pend(hSemMainLoop, BIOS_WAIT_FOREVER);
//              sec = AONRTCSecGet();
//              frac = AONRTCFractionGet();
//              processTaskAlert(arg0);

          }


      }else if(phone == 3){
          SimpleBLEPeripheral_performPeriodicTask();//This turns off bluetooth
          pageblockadd = 0;
          Mem_SF(0x00);
          blockerasefxn();
          savelastpageadd();
          while(1){
              PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
          }
      }

      //TODO: increase 20 to 60
      if((AONRTCSecGet() > 10) && (phone == 0)){
          phone = 1;
          //SimpleBLEPeripheral_performPeriodicTask();//This turns off bluetooth
          //PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
          //Task_sleep((UInt)arg0);
          //          PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
      }


//      if(selectedMode == 2){
//          //DEBUG("DATA COLLECTION");
//          ICall_signal(sem);
//      }else if(selectedMode == 3){
//         // DEBUG("DATA ERASE");
//          ICall_signal(sem);
//
//      }




     /* if(runOnce < 2000 && connected ==1 )//&& valueReceived == 1)
      {
          if(runOnce == 0){Task_sleep(150000);}
          for(int ii =0; ii<126; ii++)
          {
              thePayload[ii] = (runOnce % 26) + 0x41;
          }
          thePayload[126] = '\r';
          thePayload[127] = '\n';
          SPPBLEServer_enqueueUARTMsg(SBP_UART_DATA_EVT, thePayload, 128);
          //valueReceived = 0;
          Task_sleep(3000);
      }else if(runOnce == 2000 && connected ==1)
      {
          thePayload[0] = 'E';
          thePayload[1] = 'n';
          thePayload[2] = 'd';
          bStatus_t retVal2 = FAILURE;
          do
          {
              retVal2 = SerialPortService_SetParameter(SERIALPORTSERVICE_CHAR_DATA, 3, thePayload);
          }while(retVal2 != SUCCESS);
      }
      runOnce++;*/

      //esl      // If RTOS queue is not empty, process app message.
//      if (!Queue_empty(appUARTMsgQueue))
//      {
//        //Get the message at the front of the queue but still keep it in the queue
//        queueRec_t *pRec = Queue_head(appUARTMsgQueue);
//        sbpUARTEvt_t *pMsg = (sbpUARTEvt_t *)pRec->pData;
//
//        if (pMsg && ((gapProfileState == GAPROLE_CONNECTED) || (gapProfileState == GAPROLE_CONNECTED_ADV)))
//        {
//            bStatus_t retVal = FAILURE;
//
//            switch(pMsg->event)
//            {
//              case SBP_UART_DATA_EVT:
//              {
//                //Send the notification
//                retVal = SerialPortService_SetParameter(SERIALPORTSERVICE_CHAR_DATA, pMsg->length, pMsg->pData);
//
//                if(retVal != SUCCESS)
//                {
//                  //Display_print1(dispHandle, 5, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
//                  //Display_print1(dispHandle, 4, 0, " %d", retVal);
//                }
//                else
//                {
//                  //Increment TX status counter
//                  SerialPortService_AddStatusTXBytes(pMsg->length);
//
//                  //Remove from queue
//                  Util_dequeueMsg(appUARTMsgQueue);
//
//                  //Toggle LED to indicate data received from UART terminal and sent over the air
//                  //SPPBLEServer_toggleLed(Board_GLED, Board_LED_TOGGLE);
//
//                  //Deallocate data payload being transmitted.
//                  ICall_freeMsg(pMsg->pData);
//                  // Free the space from the message.
//                  ICall_free(pMsg);
//                }
//
//                  if(!Queue_empty(appUARTMsgQueue))
//                  {
//                    // Wake up the application to flush out any remaining UART data in the queue.
//                    Semaphore_post(sem);
//                  }
//                break;
//              }
//            default:
//              break;
//          }
//        }
//      }

//(124)
      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          SPPBLEServer_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }

    }





    if (events & SBP_PERIODIC_EVT)
    {
      events &= ~SBP_PERIODIC_EVT;

      Util_startClock(&periodicClock);

      // Perform periodic application task
      SPPBLEServer_performPeriodicTask();
    }

    ICall_signal(sem);

/*//esl2
    //ADD NEW STUFF
#ifdef FEATURE_OAD
    while (!Queue_empty(hOadQ))
    {
      oadTargetWrite_t *oadWriteEvt = Queue_dequeue(hOadQ);

      // Identify new image.
      if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
      {
        OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }
      // Write a next block request.
      else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
      {
        OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }

      // Free buffer.
      ICall_free(oadWriteEvt);
    }
#endif //FEATURE_OAD*/
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SPPBLEServer_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SPPBLEServer_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            break;

          default:
            break;
        }
      }
      break;

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}


/*********************************************************************
 * @fn      SPPBLEServer_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SPPBLEServer_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      SPPBLEServer_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    //Display_print1(dispHandle, 5, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    currentMTUSize = pMsg->msg.mtuEvt.MTU;
    SDITask_setAppDataSize(currentMTUSize);
    //Display_print1(dispHandle, 5, 0, "MTU Size: %d", currentMTUSize);
    //esl DEBUG("MTU Size: "); DEBUG((uint8_t*)convInt32ToText((int)currentMTUSize)); DEBUG_NEWLINE();
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SPPBLEServer_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void SPPBLEServer_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

      // We're done with the response message
      SPPBLEServer_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      //Display_print1(dispHandle, 5, 0, "Rsp send retry: %d", rspTxRetry);
    }
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void SPPBLEServer_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      //Display_print1(dispHandle, 5, 0, "Rsp sent retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      //Display_print1(dispHandle, 5, 0, "Rsp retry failed: %d", rspTxRetry);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SPPBLEServer_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBP_STATE_CHANGE_EVT:
      SPPBLEServer_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;

    case SBP_KEY_CHANGE_EVT:
      SPPBLEServer_handleKeys(0, pMsg->hdr.state);
      break;

    case SBP_CHAR_CHANGE_EVT:
      SPPBLEServer_processCharValueChangeEvt(pMsg->hdr.state);
      break;

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SPPBLEServer_stateChangeCB(gaprole_States_t newState)
{
  SPPBLEServer_enqueueMsg(SBP_STATE_CHANGE_EVT, newState);
}

/*********************************************************************
 * @fn      SPPBLEServer_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SPPBLEServer_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Display device address
       // Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(ownAddress));
        //Display_print0(dispHandle, 2, 0, "Initialized");
      }
      break;

    case GAPROLE_ADVERTISING:
     // Display_print0(dispHandle, 2, 0, "Advertising");
      //esl DEBUG("Advertising..."); DEBUG_NEWLINE();
      break;

#ifdef PLUS_BROADCASTER
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of
     * state to the application.  These are then disabled here so that sending
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;

        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);

        advertEnabled = TRUE;

        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);

        // Reset flag for next connection.
        firstConnFlag = false;

        SPPBLEServer_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER

    case GAPROLE_CONNECTED:
      {
        linkDBInfo_t linkInfo;
        uint8_t numActive = 0;

        Util_startClock(&periodicClock);

        numActive = linkDB_NumActive();

        connHandle = numActive - 1;

        // Use numActive to determine the connection handle of the last
        // connection
        if ( linkDB_GetInfo( numActive - 1, &linkInfo ) == SUCCESS )
        {
          //Display_print1(dispHandle, 2, 0, "Num Conns: %d", (uint16_t)numActive);
          //Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(linkInfo.addr));
          //esl DEBUG("CONNECTED..."); DEBUG_NEWLINE();
        }
        else
        {

        }

        uint8_t connectedAddr[B_ADDR_LEN];
        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, connectedAddr);
        //Checks the address of client
        if(connectedAddr[0] == 0x81 && connectedAddr[1] == 0xD7 && connectedAddr[2] == 0x7E && connectedAddr[3] == 0xAB && connectedAddr[4] == 0x78 && connectedAddr[5] == 0xCC)
        {
          phone = 2;
          pageblockadd = 0;

          //DEBUG("LAUNCHPAD CONNECTED");
        }else if(connectedAddr[0] == 0x80 && connectedAddr[1] == 0xEF && connectedAddr[2] == 0x7E && connectedAddr[3] == 0xAB && connectedAddr[4] == 0x78 && connectedAddr[5] == 0xCC){
          phone = 3;
          //DEBUG("PHONE CONNECTED");
        }else{
            SimpleBLEPeripheral_performPeriodicTask();//This turns off bluetooth
            while(1){
                PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
                Task_sleep((1000000 / Clock_tickPeriod)>>1);
                PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
                Task_sleep((1000000 / Clock_tickPeriod)>>1);

            }
        }

        connected = 1;
        SPPBLEServer_toggleLed(Board_GLED, Board_LED_TOGGLE);

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);

            // Set to true for non-connectabel advertising.
            advertEnabled = TRUE;

            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      //Display_print0(dispHandle, 2, 0, "Connected Advertising");
      break;

    case GAPROLE_WAITING:
      Util_stopClock(&periodicClock);
      SPPBLEServer_freeAttRsp(bleNotConnected);

      //Display_print0(dispHandle, 2, 0, "Disconnected");

      //esl DEBUG("DISCONNECTED..."); DEBUG_NEWLINE();
      // Clear remaining lines
      //Display_clearLines(dispHandle, 3, 5);
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      SPPBLEServer_freeAttRsp(bleNotConnected);

      //Display_print0(dispHandle, 2, 0, "Timed Out");
      //esl DEBUG("DISCONNECTED AFTER TIMEOUT..."); DEBUG_NEWLINE();
      // Clear remaining lines
      //Display_clearLines(dispHandle, 3, 5);

      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif //#ifdef (PLUS_BROADCASTER)
      break;

    case GAPROLE_ERROR:
      //Display_print0(dispHandle, 2, 0, "Error");
      break;

    default:
      //Display_clearLine(dispHandle, 2);
      break;
  }

  // Update the state
  gapProfileState = newState;
}

#ifndef FEATURE_OAD_ONCHIP
/*********************************************************************
 * @fn      SPPBLEServer_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SPPBLEServer_charValueChangeCB(uint8_t paramID)
{
  SPPBLEServer_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID);
}
#endif //!FEATURE_OAD_ONCHIP

/*********************************************************************
 * @fn      SPPBLEServer_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SPPBLEServer_processCharValueChangeEvt(uint8_t paramID)
{

    valueReceived = 1;
}

/*********************************************************************
 * @fn      SPPBLEServer_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SBP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SPPBLEServer_performPeriodicTask(void)
{

}

//esl
//#ifdef FEATURE_OAD
///*********************************************************************
// * @fn      SPPBLEServer_processOadWriteCB
// *
// * @brief   Process a write request to the OAD profile.
// *
// * @param   event      - event type:
// *                       OAD_WRITE_IDENTIFY_REQ
// *                       OAD_WRITE_BLOCK_REQ
// * @param   connHandle - the connection Handle this request is from.
// * @param   pData      - pointer to data for processing and/or storing.
// *
// * @return  None.
// */
// void SPPBLEServer_processOadWriteCB(uint8_t event, uint16_t connHandle,
//                                           uint8_t *pData)
//{
//  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
//                                             sizeof(uint8_t) * OAD_PACKET_SIZE);
//
//  if ( oadWriteEvt != NULL )
//  {
//    oadWriteEvt->event = event;
//    oadWriteEvt->connHandle = connHandle;
//
//    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
//    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);
//
//    Queue_enqueue(hOadQ, (Queue_Elem *)oadWriteEvt);
//
//    // Post the application's semaphore.
//    Semaphore_post(sem);
//  }
//  else
//  {
//    // Fail silently.
//  }
//}
//#endif //FEATURE_OAD

/*********************************************************************
 * @fn      SPPBLEServer_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SPPBLEServer_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SPPBLEServer_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event  - message event.
 * @param   status - message status.
 *
 * @return  None.
 */
/*//esl void SPPBLEServer_enqueueUARTMsg(uint8_t event, uint8_t *data, uint8_t len)
{
  /*sbpUARTEvt_t *pMsg;

  //Enqueue message only in a connected state
  if((gapProfileState == GAPROLE_CONNECTED) || (gapProfileState == GAPROLE_CONNECTED_ADV))
  {
    // Create dynamic pointer to message.
    if (pMsg = ICall_malloc(sizeof(sbpUARTEvt_t)))
    {

      pMsg->event = event;
      pMsg->pData = (uint8 *)ICall_allocMsg(len);
      if(pMsg->pData)
      {
        //payload
        memcpy(pMsg->pData , data, len);
      }
      pMsg->length = len;

      // Enqueue the message.
      Util_enqueueMsg(appUARTMsgQueue, sem, (uint8*)pMsg);
    }
  }
}*/

/*********************************************************************
 * @fn      SPPBLEServer_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static void SPPBLEServer_enqueueMsg(uint8_t event, uint8_t state)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void SPPBLEServer_handleKeys(uint8_t shift, uint8_t keys)
{
  (void)shift;  // Intentionally unreferenced parameter


  // Set Packet Length in a Connection
  if (keys & KEY_RIGHT)
  {
    //SPPBLEServer_toggleLed(Board_GLED, Board_LED_TOGGLE);

    if (gapProfileState == GAPROLE_CONNECTED )
    {

      //Request max supported size
      uint16_t requestedPDUSize = APP_SUGGESTED_PDU_SIZE;
      uint16_t requestedTxTime = APP_SUGGESTED_TX_TIME;

      //This API is documented in hci.h
      if(SUCCESS != HCI_LE_SetDataLenCmd(connHandle, requestedPDUSize, requestedTxTime))
      {
          //esl DEBUG("Data length update failed");
      }

    }
    return;
  }

  if (keys & KEY_LEFT)
  {
    //SPPBLEServer_toggleLed(Board_RLED, Board_LED_TOGGLE);

    // Start or stop discovery
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      uint8_t status;

      //Send the notification
      status = SerialPortService_SetParameter(SERIALPORTSERVICE_CHAR_DATA, 1, &charVal);

      if(status == SUCCESS){
        charVal++;
      }
    }

    return;
  }

}

/*********************************************************************
 * @fn      SPPBLEServer_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SPPBLEServer_keyChangeHandler(uint8 keys)
{
  SPPBLEServer_enqueueMsg(SBP_KEY_CHANGE_EVT, keys);
}

/*******************************************************************************
* @fn          convInt32ToText
*
* @brief       Converts 32 bit int to text
*
* @param       int32 value
*
* @return      char* - pointer to text buffer which is a file scope allocated array
*/
char* convInt32ToText(int32 value) {
    static char pValueToTextBuffer[12];
    char *pLast;
    char *pFirst;
    char last;
    uint8 negative;

    pLast = pValueToTextBuffer;

    // Record the sign of the value
    negative = (value < 0);
    value = ABS(value);

    // Print the value in the reverse order
    do {
        *(pLast++) = '0' + (uint8)(value % 10);
        value /= 10;
    } while (value);

    // Add the '-' when the number is negative, and terminate the string
    if (negative) *(pLast++) = '-';
    *(pLast--) = 0x00;

    // Now reverse the string
    pFirst = pValueToTextBuffer;
    while (pLast > pFirst) {
        last = *pLast;
        *(pLast--) = *pFirst;
        *(pFirst++) = last;
    }

    return pValueToTextBuffer;
}


/*********************************************************************
*********************************************************************/

/*114void DataTransferSession()
{
    bStatus_t ret;
    //wait for request
    uint8_t value = 0;
    while(1)
    {
        ret = DataTransferService_GetParameter(DATATRANSFERSERVICE_REQUEST, &value);
        if(ret == SUCCESS)
        {
            if(value == 1){
                selectedMode = 1;
                return;
            }else if(value ==2){
                selectedMode = 2;
                return; //Req turned non-zero by host, so begin
            }else if(value ==3){
                selectedMode = 3;
                return; //Req turned non-zero by host, so begin
            }
        }
    }

}*/

void EndOfTransmission(void){
    uint8_t theEnd[] = "END OF TRANSMISSION\r\n";
    //esl SPPBLEServer_enqueueUARTMsg(SBP_UART_DATA_EVT, theEnd, 3);

    bStatus_t retVal2 = FAILURE;
    do
    {
        retVal2 = SerialPortService_SetParameter(SERIALPORTSERVICE_CHAR_DATA, 21, theEnd);
    }while(retVal2 != SUCCESS);
}


//Initialize SPI
void SPI_params_init(){

    CS_init();

    SPI_Params_init(&spiParams);
    spiParams.transferMode = SPI_MODE_BLOCKING;
    spiParams.transferCallbackFxn = NULL;
    spiParams.bitRate = 1000000;
    spiParams.frameFormat = SPI_POL1_PHA1;          //clock pol and pha, mode0 : 0,0, mode1: 1,1
    spiParams.mode = SPI_MASTER;
    //spiParams.transferTimeout = 0x1;

    spi = SPI_open(peripheralNum, &spiParams);
}

void CS_init(){
    CSPinHandle = PIN_open(&CSPinState, CSPinTable);
    if(!CSPinHandle){
     //   System_printf("Cannot open CS Pin");
    }
}


int readfromapage(UArg arg0){
    int resultval;
       int ifbusybit;

       resultval = Mem_RtC(); //Read from memory array to cache
       resultval = Mem_GF();    //Check status register
       ifbusybit = (resultval&1);

       while(ifbusybit){        //Stay in the while loop until the execution is done
           resultval = Mem_GF();
           ifbusybit = (resultval&1);
       }

       /*PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
       Task_sleep((UInt)arg0);
       PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
       Task_sleep((UInt)arg0);
       PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
       Task_sleep((UInt)arg0);
       PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
       Task_sleep((UInt)arg0);*/
       //resultval = Mem_ID_Check();

       //resultval = Mem_RfC();
       Mem_PP2(); //Program


       if(pageblockadd == (lastpagewritten)){
           EndOfTransmission();
           //DEBUG("ENDOFTRANSMISSION");
           while(1){
               SimpleBLEPeripheral_performPeriodicTask();
               PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
               Task_sleep((UInt)arg0>>1);
               PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
               Task_sleep((UInt)arg0>>1);
               PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
               Task_sleep((UInt)arg0>>1);
               PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
               Task_sleep((UInt)arg0*2);
           }
           return 1;
       }else{
           pageblockadd = pageblockadd + 0x00000001;
           return 0;
       }
}


//Memory Read
int Mem_RtC(){
    uint8_t CMD_RtC[4]={0x13, 0x00, 0x00, 0x00};
    //blockadd update
    CMD_RtC[1] = pageblockadd >> 16;
    CMD_RtC[2] = pageblockadd >> 8;
    CMD_RtC[3] = pageblockadd;
    uint8_t Data[4];
    Bool TransferOK;
    //SPI_Transaction spiTransaction;
    spiTransaction.arg = NULL;
    spiTransaction.count = 4;
    spiTransaction.txBuf = CMD_RtC;
    spiTransaction.rxBuf = Data;

    CS_LOW();
    TransferOK = SPI_transfer(spi, &spiTransaction);
    CS_HIGH();

    int result[2] = {0x55, 0x55};
    return result[0];

}

//Get Feature
int Mem_GF(){
    uint8_t CMD_GF[3] = {0x0F, 0xC0, 0x00}; //Block lock: A0, Configuration: B0, Status: C0, Die Select: D0
    uint8_t Data[3];

    //SPI_Transaction spiTransaction;
    spiTransaction.arg = NULL;
    spiTransaction.count = 3;
    spiTransaction.txBuf = CMD_GF;
    spiTransaction.rxBuf = Data;

    CS_LOW();
    SPI_transfer(spi, &spiTransaction);
    CS_HIGH();
    return Data[2];

}


////Memory Program LOAD + Program EXECUTE
void Mem_PP2(){
   // Mem_WE();   //Write Enable

    uint8_t CMD_PL[20] = {[0 ... 19] 0x00};
    CMD_PL[0] = 0x03;
    //CMD_PL[1] = 0x00;
    //CMD_PL[2] = 0x00;
    CMD_PL[1] = columnadd >>8;
    CMD_PL[2] = columnadd;


    //uint8_t CMD_PL[12] = {0x02, 0x00,0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66}; //0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};
   /* int i;
    for(i=3; i<20; i++){
        CMD_PL[i] = passArray[i-3];
    }*/
  /*  if(programcount == 1){
        CMD_PL[3] = 0x55;
        CMD_PL[4] = 0x55;
        CMD_PL[5] = 0x55;
        CMD_PL[6] = 0x55;
        CMD_PL[7] = 0x55;
        CMD_PL[8] = 0x55;
        CMD_PL[9] = 0x55;
        CMD_PL[10] = 0x55;
        CMD_PL[11] = 0x55;

    }*/

    uint8_t Data_PL[20];
    Bool TransferOK;
    //SPI_Transaction spiTransaction;
    spiTransaction.arg = NULL;
    spiTransaction.count = 20;
    spiTransaction.txBuf = CMD_PL;
    spiTransaction.rxBuf = Data_PL;
    int t = 0;
    //Program Load
    CS_LOW();
    TransferOK = SPI_transfer(spi, &spiTransaction);
    int k;
    for (k = 0; k<16 ; k++){
        passArray[k] = Data_PL[k+4];
    }
   // memcpy(&passArray[0], Data_PL[4], sizeof(Data_PL)-4);
  //  memcpy(&CMD_PL, &passArray[(20*t)- 3], sizeof(CMD_PL));
    //this for loop runs 102 times getting passArray 0-17, and 17 to 2036

    CMD_PL[0] = 0x00;
    for(t = 1; t <102; t++){
        TransferOK = SPI_transfer(spi, &spiTransaction);
        for (k = 0; k<20 ; k++){
            passArray[(20*t)-4+k] = Data_PL[k];
        }
       // memcpy(&passArray[(20*t)-4], Data_PL, sizeof(CMD_PL));
    }

    spiTransaction.count = 12;
    TransferOK = SPI_transfer(spi, &spiTransaction);

    //memcpy(&passArray[2036], Data_PL, sizeof(Data_PL)-8);
    for(k = 0; k<12; k++){
            passArray[2036+k] = Data_PL[k];
        }
    CS_HIGH();
}


//Chip Select Low
void CS_LOW(){
    PIN_setOutputValue(CSPinHandle, PIN_SPI0_CS, 0);
}

//Chip Select High
void CS_HIGH(){
    PIN_setOutputValue(CSPinHandle, PIN_SPI0_CS, 1);
}


void savetoapage(UArg arg0){

    if(pageblockadd > STORAGE_MAX_ADDRESS){
      while(1){
          PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
          Task_sleep((UInt)arg0);
          PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
          Task_sleep((UInt)arg0);
      }
    }

    //TODO: remove led blinking
    //PIN_setOutputValue(ledPinHandle, LED_PIN, tempval);
    //tempval ^= 1;

    int resultval;
    int ifbusybit;
//    if(pageblockadd == 0x00000000){
//      //  resultval = Mem_SF(0x00);    //Unlock all blocks
//        blockerasefxn();
//    }

   Mem_PP(); //Program

    resultval = Mem_GF();    //Check status register
    //PROGRAM EXECUTE, PAGE READ, READ PAGE CACHE LAST, BLOCK ERASE, READ PAGE CACHE RANDOM bit can be checked
    ifbusybit = (resultval&1);

    while(ifbusybit){        //Stay in the while loop until the execution is done
        resultval = Mem_GF();
        ifbusybit = (resultval&1);
    }
    //Mem_lock();

    pageblockadd = pageblockadd + 0x00000001;
    savelastpageadd();  //saves the next page address to be written in non-volatile storage
    //PIN_setOutputValue(ledPinHandle, LED_PIN, !PIN_getOutputValue(LED_PIN));


}

void createarray(){
    //uint8_t initialval = 0x00;
    //pageval = pageval + 1;
    int i;
    int ii;
    for (i = 0; i <16; i++){
        for(ii =0; ii<126; ii++)
        {
            passArray[128*i + ii] = i%26 + 0x41;
        }
        passArray[128*i + ii] = '\r';
        passArray[128*i + ii + 1] = '\n';
        //passArray[i] =  i%26 + 0x41;//pageval;//

    }
}

//Set Feature
int Mem_SF(uint8_t lockcmd){
    uint8_t CMD_SF[3] = {0x1F, 0xA0, 0x00}; //Block lock: A0, Configuration: B0, Status: C0, Die Select: D0
    CMD_SF[2] = lockcmd;
    uint8_t Data[3];

    //SPI_Transaction spiTransaction;
    spiTransaction.arg = NULL;
    spiTransaction.count = 3;
    spiTransaction.txBuf = CMD_SF;
    spiTransaction.rxBuf = Data;

    CS_LOW();
    SPI_transfer(spi, &spiTransaction);
    CS_HIGH();
    return Data[2];

}

void Mem_PP(){
    Mem_WE();   //Write Enable
    uint8_t CMD_PL[20] = {[0 ... 19] 0x00};
    CMD_PL[0] = 0x02;
    //CMD_PL[1] = 0x00;
    //CMD_PL[2] = 0x00;
    CMD_PL[1] = columnadd >>8;
    CMD_PL[2] = columnadd;


    //uint8_t CMD_PL[12] = {0x02, 0x00,0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66}; //0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};
    int i;
    for(i=3; i<20; i++){
        CMD_PL[i] = passArray[i-3];
    }
  /*  if(programcount == 1){
        CMD_PL[3] = 0x55;
        CMD_PL[4] = 0x55;
        CMD_PL[5] = 0x55;
        CMD_PL[6] = 0x55;
        CMD_PL[7] = 0x55;
        CMD_PL[8] = 0x55;
        CMD_PL[9] = 0x55;
        CMD_PL[10] = 0x55;
        CMD_PL[11] = 0x55;

    }*/

    uint8_t Data_PL[20];
    Bool TransferOK;
    //SPI_Transaction spiTransaction;
    spiTransaction.arg = NULL;
    spiTransaction.count = 20;
    spiTransaction.txBuf = CMD_PL;
    spiTransaction.rxBuf = Data_PL;
    int t = 0;
    //Program Load
    CS_LOW();
    TransferOK = SPI_transfer(spi, &spiTransaction);
  //  memcpy(&CMD_PL, &passArray[(20*t)- 3], sizeof(CMD_PL));
    //this for loop runs 102 times getting passArray 0-17, and 17 to 2036
    for(t = 1; t <102; t++){
        memcpy(&CMD_PL, &passArray[(20*t)- 3], sizeof(CMD_PL));
        TransferOK = SPI_transfer(spi, &spiTransaction);
    }
    spiTransaction.count = 11;
    memcpy(&CMD_PL, &passArray[2037], sizeof(CMD_PL));
    TransferOK = SPI_transfer(spi, &spiTransaction);
    //add 12 more data
    CS_HIGH();

    uint8_t CMD_PE[4] = {0x10, 0x00,0x00, 0x00};
    //blockadd update
    CMD_PE[1] = pageblockadd >> 16;
    CMD_PE[2] = pageblockadd >> 8;
    CMD_PE[3] = pageblockadd;

    uint8_t Data_PE[4];
    spiTransaction.arg = NULL;
    spiTransaction.count = 4;
    spiTransaction.txBuf = CMD_PE;
    spiTransaction.rxBuf = Data_PE;
    //Program Load
    CS_LOW();
    TransferOK = SPI_transfer(spi, &spiTransaction);

    CS_HIGH();

}


void blockerasefxn(){
    int resultval;
    int ifbusybit;
    //uint32_t currenttemp = pageblockadd;
    pageblockadd = 0x00000000;
    while(pageblockadd < STORAGE_MAX_ADDRESS){
        if(countapage == 0){
            Mem_BE();    //Block Erase

            resultval = Mem_GF();    //Check status register
            //PROGRAM EXECUTE, PAGE READ, READ PAGE CACHE LAST, BLOCK ERASE, READ PAGE CACHE RANDOM bit can be checked
            ifbusybit = (resultval&1);

            while(ifbusybit){        //Stay in the while loop until the execution is done
                resultval = Mem_GF();
                ifbusybit = (resultval&1);
            }
        }
        countapage ++;
        if(countapage == 64){
            countapage = 0;
        }
        pageblockadd = pageblockadd + 0x00000001;
    }
    pageblockadd  = 0x00000000;
}


//Memory Block Erase
void Mem_BE(){
    Mem_WE(); //Write Enable
    uint8_t CMD_SE[4] = {0xD8,0x00,0x00,0x00};

    //New address update
    CMD_SE[1] = pageblockadd >> 16;
    CMD_SE[2] = pageblockadd >> 8;
    CMD_SE[3] = pageblockadd;

   /* if(becount == 1){
        CMD_SE[3] = 0x02;
    }*/
    uint8_t Data[4];
   // Bool TransferOK;
    //SPI_Transaction spiTransaction;
    spiTransaction.arg = NULL;
    spiTransaction.count = 4;
    spiTransaction.txBuf = CMD_SE;
    spiTransaction.rxBuf = Data;

    CS_LOW();
    SPI_transfer(spi, &spiTransaction);
    CS_HIGH();
  //  becount = 1;
}


//Memory Write Enable
void Mem_WE(){
    uint8_t CMD_WEE[3] = 0x06;
    uint8_t Data[3];
    Bool TransferOK;
    //SPI_Transaction spiTransaction;
    spiTransaction.arg = NULL;
    spiTransaction.count = 3;
    spiTransaction.txBuf = CMD_WEE;
    spiTransaction.rxBuf = Data;

    CS_LOW();
    TransferOK = SPI_transfer(spi, &spiTransaction);
    CS_HIGH();
/*
    if(!TransferOK){
        return 1;
    }else{
        CS_HIGH();
        return 0;
    }
*/

}

void scCtrlReadyCallback(void)
{

} // scCtrlReadyCallback

void scTaskAlertCallback(void)
{
    // Post to main loop semaphore
    Semaphore_post(hSemMainLoop);
} // scTaskAlertCallback


void processTaskAlert(UArg arg0)
{
    // Clear the ALERT interrupt source
    scifClearAlertIntSource();

    memcpy(&passArray[numSCBuf*292], &scifTaskData.i2cimupressurewarray1111.output.result[0], 284);
    passArray[(numSCBuf+1)*292-8] = sec >> 24;
    passArray[(numSCBuf+1)*292-7] = sec >> 16;
    passArray[(numSCBuf+1)*292-6] = sec >> 8;
    passArray[(numSCBuf+1)*292-5] = sec;
    passArray[(numSCBuf+1)*292-4] = frac >> 24;
    passArray[(numSCBuf+1)*292-3] = frac >> 16;
    passArray[(numSCBuf+1)*292-2] = frac >> 8;
    passArray[(numSCBuf+1)*292-1] = frac;
    numSCBuf++;
    if(numSCBuf >= 7){
        numSCBuf = 0;
        savetoapage(arg0);
    }

    //mem_fin(288);

    // Acknowledge the ALERT event
    scifAckAlertEvents();
    //scifReinitTaskIo(BV(SCIF_I2CIMUPRESSUREWARRAY_TASK_ID));

} // processTaskAlert

static void SimpleBLEPeripheral_performPeriodicTask(void)
{
  //turn off advertising
   // Setup the GAP Peripheral Role Profile
    uint8_t initialAdvertEnable = FALSE;  // Advertise on power-up

    // Set advertisement enabled.
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initialAdvertEnable);
}

//void mem_fin(int len){
//    //global variables: temp_mem, mem_count
//    //check if the temp_mem + new_data >= 2048
//    //uint8_t *new_dataa = new_data;
//
//
//    int len_total = countarray + len;
//    if(len_total>=2048){    //if it exceeds
//        int amtfit = 2048 - countarray;
//        int amtexceed = len_total - 2048;
//        for(int i = 0; i<amtfit; i++){
//            passArray[countarray + i ] = passArray_temp[i];
//        }
//        //memcpy(&passArray[countarray], new_data[0], amtfit);
//        savetoapage();
//
//
//        PIN_setOutputValue(ledPinHandle, LED_PIN, tempval);
//        tempval ^= 1;
//
//        //memcpy(&passArray[0], new_data[amtfit], amtexceed);
//        for(int i = 0; i<amtexceed; i++){
//            passArray[i] = passArray_temp[amtfit+i];
//        }
//        countarray = amtexceed;
//
//    }else{    //else if it does not exceed
//        //memcpy(&passArray[countarray], &new_data[0], len_new_data);
//        for(int i = 0; i<len; i++){
//            passArray[countarray+i] = passArray_temp[i];
//        }
//        countarray = countarray + len;
//    }
//
//}


void getlastpageadd(){
   //savelastpageadd();
    uint8_t nv_status = NV_OPER_FAILED;

  do{
        nv_status = osal_snv_read(SNV_ID_APP, BUF_LEN, (uint8 *)buf);
 }while(nv_status != SUCCESS);

    pageblockadd = buf[0] +  (buf[1] << 8) + (buf[2] << 16) + (buf[3] << 24);
}

void savelastpageadd(){

    buf[0] = pageblockadd & 0xFF;
    buf[1] = (pageblockadd >>  8) & 0xFF;
    buf[2] = (pageblockadd >> 16) & 0xFF;
    buf[3] = (pageblockadd >> 24) & 0xFF;

    uint8_t nv_status = NV_OPER_FAILED;

    do{
        nv_status = osal_snv_write(SNV_ID_APP, BUF_LEN, (uint8 *)buf);
    }while(nv_status != SUCCESS);
}




int gpsfirst = 0;

//void GPScollect(UArg arg0){
//
//   // const char echoPrompt[] = "\fEchoing characters:\r\n";
//if(gpsfirst == 0){
//    CPUdelay(8000*50);
//   // Task_sleep((UInt)arg0*20);
//    PIN_setOutputValue(ongpsPinHandle, ONGPS,!PIN_getOutputValue(ONGPS));
//    CPUdelay(8000*50);//Task_sleep((UInt)arg0);
//
//    if(PIN_getOutputValue(ONGPS)){
//    //    PIN_setOutputValue(ongpsPinHandle, ONGPS,!PIN_getOutputValue(ONGPS));
//        CPUdelay(8000*50);// Task_sleep((UInt)arg0);
//    }else{
//        PIN_setOutputValue(ongpsPinHandle, ONGPS,1);
//        CPUdelay(8000*50);//Task_sleep((UInt)arg0);
//    }
//
//    CPUdelay(8000*500);
//
//    //falling
//    if(PIN_getOutputValue(ONGPS)){
//        PIN_setOutputValue(ongpsPinHandle, ONGPS,!PIN_getOutputValue(ONGPS));
//        CPUdelay(8000*50);// Task_sleep((UInt)arg0);
//    }else{
//        PIN_setOutputValue(ongpsPinHandle, ONGPS,0);
//        CPUdelay(8000*50);//Task_sleep((UInt)arg0);
//    }
//
//    CPUdelay(8000*500);
//
//
//        PIN_setOutputValue(ongpsPinHandle, ONGPS,1);
//        CPUdelay(8000*100);// Task_sleep((UInt)arg0);
//        PIN_setOutputValue(ongpsPinHandle, ONGPS,0);
//        CPUdelay(8000*50);//Task_sleep((UInt)arg0);
//
//        CPUdelay(8000*500);
//
//
//
//        PIN_setOutputValue(ongpsPinHandle, ONGPS,1);
//        CPUdelay(8000*100);// Task_sleep((UInt)arg0);
//        PIN_setOutputValue(ongpsPinHandle, ONGPS,0);
//        CPUdelay(8000*50);//Task_sleep((UInt)arg0);
//
//        CPUdelay(8000*500);
//        /* Create a UART with data processing off. */
//           UART_Params_init(&uartParams);
//           uartParams.writeDataMode = UART_DATA_BINARY;
//           uartParams.readDataMode = UART_DATA_BINARY;
//           uartParams.readReturnMode = UART_RETURN_FULL;
//           uartParams.readEcho = UART_ECHO_OFF;
//           uartParams.baudRate = 4800;
//          uart = UART_open(Board_UART0, &uartParams);
//
//           if (uart == NULL) {
//               //System_abort("Error opening the UART");
//           }
//
//
//        gpsfirst = 1;
//}
//
//
//    //UART_write(uart, echoPrompt, sizeof(echoPrompt));
//    int i = 0;
//    int gpstimeout = 0;
//    int gpswhile = 0;
//    /* Loop forever echoing */
//    while ((gpswhile < 1)  &&  (gpstimeout<5)) {
//
//        UART_read(uart, &GPGGAinput, 60);
//        int g;
//        mloc = mloc+60;
//                      for(g=0;g<mloc;g++){
//                                  passArray[g] = GPGGAinput[g];
//                     }
//
//        if(mloc > 1900){
//            mloc = 0;
//            savetoapage(arg0);
//            PIN_setOutputValue(ledPinHandle, LED_PIN, !PIN_getOutputValue(LED_PIN));
//            Task_sleep(500);
//            PIN_setOutputValue(ledPinHandle, LED_PIN, !PIN_getOutputValue(LED_PIN));
//           // gpswhile = 1;
//        }
//   /*    result[i] = input;
//       i++;
//        if(i>299){
//       i =0;
//        }
//
//*/
//
//
////        if(input == 'G'){
////          //  result[0] = input;
////            UART_read(uart, &input, 1);
////            if(input == 'P'){
////              //  result[1] = input;
////                UART_read(uart, &input, 1);
////                if(input == 'G'){
////                 //   result[2] = input;
////                    UART_read(uart, &input, 1);
////                    if(input == 'G'){
////                       // result[3] = input;
////                        UART_read(uart, &input, 1);
////                        if(input == 'A'){
////                           // result[4] = input;
////                            UART_read(uart, &GPGGAinput, 60);
////                            UART_read(uart, &input, 1);
////
////                            sec =         AONRTCSecGet();
////                            frac =        AONRTCFractionGet();
////                            //gpswhile = 1;
////                            //TODO: change this to 40 maybe?
////                            /*if(sec > 40){
////                                gpswhile = 6;
////                            }*/
////                            int u;
////                            for (u =0 ; u<60;u++){
////                                if(GPGGAinput[u] == 'M'){
////                                    if(GPGGAinput[u-1] == ','){
////                                        if(GPGGAinput[u+1] == ','){
////                                            mloc = u;
////
////                                            break;
////                                        }
////                                    }
////
////                                }
////                            }
////                            int y;
////                            int comma = 0;
////                            for(y = mloc; y>0;y--){
////                                if(GPGGAinput[y] == ','){
////                                    comma = comma +1;
////                                }
////                                if(comma == 4){
////                                    mloc = y-1;
////                                   // gpstimeout++;
////                                   // if(gpstimeout == 30){
//////                                       // mem_fin(mloc+4);
//////                                        PIN_setOutputValue(ledPinHandle, LED_PIN, !PIN_getOutputValue(LED_PIN));
//////                                        Task_sleep(500);
//////                                        PIN_setOutputValue(ledPinHandle, LED_PIN, !PIN_getOutputValue(LED_PIN));
////                                   // }
////                                   /* PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
////                                    Task_sleep((UInt)arg0);
////                                    PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
////                                    Task_sleep((UInt)arg0);*/
////
////                                    //Task_sleep((UInt)arg0);
////                                    //PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
////                                    //Task_sleep((UInt)arg0);
////                                    break;
////                                }
////                            }
////
////
////
////                            if(GPGGAinput[mloc]=='1'){
////                                int g;
////                                for(g=0;g<mloc;g++){
////                                    passArray[g] = GPGGAinput[g];
////                                }
////
////                                gpswhile = gpswhile + 1;
//////                                PIN_setOutputValue(ledPinHandle, LED_PIN, !PIN_getOutputValue(LED_PIN));
//////                                Task_sleep(500);
//////                                PIN_setOutputValue(ledPinHandle, LED_PIN, !PIN_getOutputValue(LED_PIN));
////                                PIN_setOutputValue(ledPinHandle, LED_PIN, !PIN_getOutputValue(LED_PIN));
////                                            Task_sleep(500);
////                                            PIN_setOutputValue(ledPinHandle, LED_PIN, !PIN_getOutputValue(LED_PIN));
////                                savetoapage(arg0);
////
////                                passArray[mloc] = sec >> 8;
////                                passArray[mloc+1] = sec;
////                                passArray[mloc+2] = frac >> 8;
////                                passArray[mloc+3] = frac;
////                                //TODO
////                                //mem_fin(mloc+4);
////                                /*int g;
////                                for(g=0;g<mloc;g++){
////                                    passArray[countarray+g] = GPGGAinput[g];
////                                }
////
////                                countarray = countarray + mloc;
////                                gpswhile = gpswhile + 1;
////
////                                passArray[countarray] = sec >> 8;
////                                passArray[countarray] = sec;
////                                passArray[countarray] = frac >> 8;
////                                passArray[countarray] = frac;
////                                countarray = countarray + 4;*/
////                            }
////
////                        }
////                    }
////                }
////            }
////        }
//
//        //PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
//
//
//      //  UART_write(uart, &input, 1);
//    }
//   //countarray = 70;
//    /*
//    PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
//    Task_sleep((UInt)arg0);
//    PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
//    Task_sleep((UInt)arg0);
//    PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
//    Task_sleep((UInt)arg0);
//    PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
//    Task_sleep((UInt)arg0);
//    PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
//    Task_sleep((UInt)arg0);
//    PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
//    Task_sleep((UInt)arg0);
//    PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
//    Task_sleep((UInt)arg0);
//    PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
//    Task_sleep((UInt)arg0);*/
//
//  /*  //PUT GPS onto sleep
//    CPUdelay(8000*500);
//    //Task_sleep((UInt)arg0*20);
//    if(PIN_getOutputValue(ONGPS)){
//    //    PIN_setOutputValue(ongpsPinHandle, ONGPS,!PIN_getOutputValue(ONGPS));
//        CPUdelay(8000*500);
//        //Task_sleep((UInt)arg0);
//    }else{
//        PIN_setOutputValue(ongpsPinHandle, ONGPS,1);
//        CPUdelay(8000*500);//Task_sleep((UInt)arg0);
//    }
//*/
//   // UART_close(uart);
//
//    //countarray = 0;
//}


/****************EUN SUN SECTION********************************/
void GPScollect(UArg arg0){

   // const char echoPrompt[] = "\fEchoing characters:\r\n";
    if(gpsfirst == 0){
        CPUdelay(8000*50);
        //Task_sleep((UInt)arg0*20);
        PIN_setOutputValue(ongpsPinHandle, ONGPS,!PIN_getOutputValue(ONGPS));
        CPUdelay(8000*50);//Task_sleep((UInt)arg0);

        if(PIN_getOutputValue(ONGPS)){
            //PIN_setOutputValue(ongpsPinHandle, ONGPS,!PIN_getOutputValue(ONGPS));
            CPUdelay(8000*50);
            //Task_sleep((UInt)arg0);
        }else{
            PIN_setOutputValue(ongpsPinHandle, ONGPS,1);
            CPUdelay(8000*50);
            //Task_sleep((UInt)arg0);
        }

        CPUdelay(8000*500);

        //falling
        if(PIN_getOutputValue(ONGPS)){
            PIN_setOutputValue(ongpsPinHandle, ONGPS,!PIN_getOutputValue(ONGPS));
            CPUdelay(8000*50);
            //Task_sleep((UInt)arg0);
        }else{
            PIN_setOutputValue(ongpsPinHandle, ONGPS,0);
            CPUdelay(8000*50);
            //Task_sleep((UInt)arg0);
        }

        CPUdelay(8000*500);
        PIN_setOutputValue(ongpsPinHandle, ONGPS,1);
        CPUdelay(8000*100);
        //Task_sleep((UInt)arg0);
        PIN_setOutputValue(ongpsPinHandle, ONGPS,0);
        CPUdelay(8000*50);
        //Task_sleep((UInt)arg0);
        CPUdelay(8000*500);

        PIN_setOutputValue(ongpsPinHandle, ONGPS,1);
        CPUdelay(8000*100);
        //Task_sleep((UInt)arg0);
        PIN_setOutputValue(ongpsPinHandle, ONGPS,0);
        CPUdelay(8000*50);
        //Task_sleep((UInt)arg0);
        CPUdelay(8000*500);

        /* Create a UART with data processing off. */
        UART_Params_init(&uartParams);
        uartParams.writeDataMode = UART_DATA_BINARY;
        uartParams.readDataMode = UART_DATA_BINARY;
        uartParams.readReturnMode = UART_RETURN_FULL;
        uartParams.readEcho = UART_ECHO_OFF;
        uartParams.baudRate = 4800;
        uart = UART_open(Board_UART0, &uartParams);

        if (uart == NULL) {
           uart++;
        }

        gpsfirst = 1;
    }


    //UART_write(uart, echoPrompt, sizeof(echoPrompt));
    int i = 0;
    int gpstimeout = 0;
    int gpswhile = 0;

    /* Loop forever echoing */
    while((gpswhile < 1)  &&  (gpstimeout<5))
    {

        UART_read(uart, &input, 1);
        /*result[i] = input;
        i++;
        if(i>299)
        {
            i = 0;
        }*/
        if(input == 'G')
        {
            //result[0] = input;
            UART_read(uart, &input, 1);
            if(input == 'P')
            {
                //result[1] = input;
                UART_read(uart, &input, 1);
                if(input == 'G')
                {
                    //result[2] = input;
                    UART_read(uart, &input, 1);
                    if(input == 'G')
                    {
                        //result[3] = input;
                        UART_read(uart, &input, 1);
                        if(input == 'A')
                        {
                            //result[4] = input;
                            UART_read(uart, &GPGGAinput, 60);
                            UART_read(uart, &input, 1);

                            sec  = AONRTCSecGet();
                            frac = AONRTCFractionGet();
                            //gpswhile = 1;
                            //TODO: change this to 40 maybe?
                            /*if(sec > 40){
                                gpswhile = 6;
                            }*/
                            int u;
                            for(u =0 ; u<60; u++)
                            {
                                if(GPGGAinput[u] == 'M')
                                {
                                    if(GPGGAinput[u-1] == ',')
                                    {
                                        if(GPGGAinput[u+1] == ',')
                                        {
                                            mloc = u;
                                            break;
                                        }
                                    }
                                }
                            }
                            int y;
                            int comma = 0;
                            for(y = mloc; y>0; y--)
                            {
                                if(GPGGAinput[y] == ',')
                                {
                                    comma = comma +1;
                                }
                                if(comma == 4)
                                {
                                    mloc = y-1;
                                    /*gpstimeout++;
                                    if(gpstimeout == 30)
                                    {
                                        mem_fin(mloc+4);
                                        PIN_setOutputValue(ledPinHandle, LED_PIN, !PIN_getOutputValue(LED_PIN));
                                        Task_sleep(500);
                                        PIN_setOutputValue(ledPinHandle, LED_PIN, !PIN_getOutputValue(LED_PIN));
                                    }
                                    PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
                                    Task_sleep((UInt)arg0);
                                    PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
                                    Task_sleep((UInt)arg0);

                                    Task_sleep((UInt)arg0);
                                    PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
                                    Task_sleep((UInt)arg0);*/
                                    break;
                                }
                            }

                            if(GPGGAinput[mloc]=='1')
                            {
                                int g;
                                for(g=0; g<mloc; g++)
                                {
                                    passArray[g] = GPGGAinput[g];
                                }

                                gpswhile = gpswhile + 1;
                                //PIN_setOutputValue(ledPinHandle, LED_PIN, !PIN_getOutputValue(LED_PIN));
                                //Task_sleep(500);
                                //PIN_setOutputValue(ledPinHandle, LED_PIN, !PIN_getOutputValue(LED_PIN));
                                PIN_setOutputValue(ledPinHandle, LED_PIN, !PIN_getOutputValue(LED_PIN));
                                Task_sleep(500);
                                PIN_setOutputValue(ledPinHandle, LED_PIN, !PIN_getOutputValue(LED_PIN));
                                savetoapage(arg0);

                                passArray[mloc] = sec >> 8;
                                passArray[mloc+1] = sec;
                                passArray[mloc+2] = frac >> 8;
                                passArray[mloc+3] = frac;
                                //TODO
                                //mem_fin(mloc+4);
                                /*int g;
                                for(g=0;g<mloc;g++){
                                    passArray[countarray+g] = GPGGAinput[g];
                                }

                                countarray = countarray + mloc;
                                gpswhile = gpswhile + 1;

                                passArray[countarray] = sec >> 8;
                                passArray[countarray] = sec;
                                passArray[countarray] = frac >> 8;
                                passArray[countarray] = frac;
                                countarray = countarray + 4;*/
                            }

                        }
                    }
                }
            }
        }

        //PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
        //UART_write(uart, &input, 1);
    }
    //countarray = 70;
    /*
    PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
    Task_sleep((UInt)arg0);
    PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
    Task_sleep((UInt)arg0);
    PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
    Task_sleep((UInt)arg0);
    PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
    Task_sleep((UInt)arg0);
    PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
    Task_sleep((UInt)arg0);
    PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
    Task_sleep((UInt)arg0);
    PIN_setOutputValue(ledPinHandle, LED_PIN, 1);
    Task_sleep((UInt)arg0);
    PIN_setOutputValue(ledPinHandle, LED_PIN, 0);
    Task_sleep((UInt)arg0);*/

    /*//PUT GPS onto sleep
    CPUdelay(8000*500);
    //Task_sleep((UInt)arg0*20);
    if(PIN_getOutputValue(ONGPS)){
        //PIN_setOutputValue(ongpsPinHandle, ONGPS,!PIN_getOutputValue(ONGPS));
        CPUdelay(8000*500);
        //Task_sleep((UInt)arg0);
    }else{
        PIN_setOutputValue(ongpsPinHandle, ONGPS,1);
        CPUdelay(8000*500);//Task_sleep((UInt)arg0);
    }*/
    //UART_close(uart);
    //countarray = 0;
}
