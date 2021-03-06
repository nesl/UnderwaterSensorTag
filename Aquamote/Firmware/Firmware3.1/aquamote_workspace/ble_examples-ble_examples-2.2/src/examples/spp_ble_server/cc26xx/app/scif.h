/** \mainpage Driver Overview
  *
  * \section section_drv_info Driver Information
  * This Sensor Controller Interface driver has been generated by the Texas Instruments Sensor Controller
  * Studio tool:
  * - <b>Project name</b>:     I2C Sensors fullarray1111
  * - <b>Project file</b>:     C:/UnderwaterSensorTag/Aquamote/Firmware/I2c_sensors_fullarray - 1111/I2c_sensors_fullarray1111.scp
  * - <b>Code prefix</b>:      -
  * - <b>Operating system</b>: TI-RTOS
  * - <b>Tool version</b>:     1.4.1.54
  * - <b>Tool patches</b>:     None
  * - <b>Target chip</b>:      CC2650, revision -, package QFN32 5x5 RHB
  * - <b>Created</b>:          2018-05-03 19:48:38.152
  * - <b>Computer</b>:         DESKTOP-6Q8NGS0
  * - <b>User</b>:             lkozinakov
  *
  * No user-provided resource definitions were used to generate this driver.
  *
  * No user-provided procedure definitions were used to generate this driver.
  *
  * Do not edit the generated source code files other than temporarily for debug purposes. Any
  * modifications will be overwritten by the Sensor Controller Studio when generating new output.
  *
  * \section section_drv_modules Driver Modules
  * The driver is divided into three modules:
  * - \ref module_scif_generic_interface, providing the API for:
  *     - Initializing and uninitializing the driver
  *     - Task control (for starting, stopping and executing Sensor Controller tasks)
  *     - Task data exchange (for producing input data to and consume output data from Sensor Controller
  *       tasks)
  * - \ref module_scif_driver_setup, containing:
  *     - The AUX RAM image (Sensor Controller code and data)
  *     - I/O mapping information
  *     - Task data structure information
  *     - Driver setup data, to be used in the driver initialization
  *     - Project-specific functionality
  * - \ref module_scif_osal, for flexible OS support:
  *     - Interfaces with the selected operating system
  *
  * It is possible to use output from multiple Sensor Controller Studio projects in one application. Only
  * one driver setup may be active at a time, but it is possible to switch between these setups. When
  * using this option, there is one instance of the \ref module_scif_generic_interface and
  * \ref module_scif_osal modules, and multiple instances of the \ref module_scif_driver_setup module.
  * This requires that:
  * - The outputs must be generated using the same version of Sensor Controller Studio
  * - The outputs must use the same operating system
  * - The outputs must use different source code prefixes (inserted into all globals of the
  *   \ref module_scif_driver_setup)
  *
  *
  * \section section_project_info Project Description
  * No description entered
  *
  *
  * \subsection section_io_mapping I/O Mapping
  * Task I/O functions are mapped to the following pins:
  * - i2cimupressurewarray1111:
  *     - <b>I2C SCL</b>: DIO3
  *     - <b>I2C SDA</b>: DIO2
  *
  *
  * \section section_task_info Task Description(s)
  * This driver supports the following task(s):
  *
  *
  * \subsection section_task_desc_i2cimupressurewarray1111 i2cimupressurewarray1111
  * No description entered
  *
  */




/** \addtogroup module_scif_driver_setup Driver Setup
  *
  * \section section_driver_setup_overview Overview
  *
  * This driver setup instance has been generated for:
  * - <b>Project name</b>:     I2C Sensors fullarray1111
  * - <b>Code prefix</b>:      -
  *
  * The driver setup module contains the generated output from the Sensor Controller Studio project:
  * - Location of task control and scheduling data structures in AUX RAM
  * - The AUX RAM image, and the size the image
  * - Task data structure information (location, size and buffer count)
  * - I/O pin mapping translation table
  * - Task resource initialization and uninitialization functions
  *
  * @{
  */
#ifndef SCIF_H
#define SCIF_H

#include <stdint.h>
#include <stdbool.h>
#include "scif_framework.h"
#include "scif_osal_tirtos.h"


/// Target chip name
#define SCIF_TARGET_CHIP_NAME_CC2650
/// Target chip package
#define SCIF_TARGET_CHIP_PACKAGE_QFN32_5X5_RHB

/// Number of tasks implemented by this driver
#define SCIF_TASK_COUNT 1

/// i2cimupressurewarray1111: Task ID
#define SCIF_I2CIMUPRESSUREWARRAY1111_TASK_ID 0


/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ACCEL_CONFIG 20
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ACCEL_SIZE 30
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ACCEL_XOUT_H 45
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ARRAY_SIZE8 8
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_BIN_COUNT 10
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_CNTL2 49
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_HXL 17
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ICM_I2C_ADDR 208
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_INT_PIN_CFG 15
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_MAG_I2C_ADDRESS 24
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_MS5837_ADC_READ 0
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_MS5837_CONV_D1_4096 74
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_MS5837_CONV_D2_4096 88
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_MS5837_I2C_ADDR 236
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_MS5837_PROM 160
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_MS5837_RESET 30
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ONEPAGEARRAY2048 2048
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_PWR_MGMT_1 6
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_REG_MAGNETOMETER_ID 1
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_REG_MANUFACTURER_ID 0
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_REG_USERBANKSELECT 127
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ARRAY100 100
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ARRAY142 142
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ARRAY284 284
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ARRAY300 300
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ARRAY50 50
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ARRAY500 500
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ARRAY512 512
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ARRAY6 6
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ARRAY60 60
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ARRAY7 7
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ARRAY70 70
/// i2cimupressurewarray1111: 
#define SCIF_I2CIMUPRESSUREWARRAY1111_ARRAY90 90
/// i2cimupressurewarray1111 I/O mapping: I2C SCL
#define SCIF_I2CIMUPRESSUREWARRAY1111_DIO_I2C_SCL 3
/// i2cimupressurewarray1111 I/O mapping: I2C SDA
#define SCIF_I2CIMUPRESSUREWARRAY1111_DIO_I2C_SDA 2


// All shared data structures in AUX RAM need to be packed
#pragma pack(push, 2)


/// i2cimupressurewarray1111: Task input data structure
typedef struct {
    uint16_t prom;    ///< 
    uint16_t testval; ///< 
} SCIF_I2CIMUPRESSUREWARRAY1111_INPUT_T;


/// i2cimupressurewarray1111: Task output data structure
typedef struct {
    uint16_t pressureinit[7]; ///< 
    uint16_t result[142];     ///< 
    uint16_t temperature[6];  ///< 
    uint16_t tempresult[6];   ///< 
} SCIF_I2CIMUPRESSUREWARRAY1111_OUTPUT_T;


/// i2cimupressurewarray1111: Task state structure
typedef struct {
    uint16_t count1;        ///< 
    uint16_t i2cStatus;     ///< I2C master status
    uint16_t magcount;      ///< 
    uint16_t ncount;        ///< 
    uint16_t outputcount;   ///< 
    uint16_t pressurecount; ///< 
    uint16_t prom;          ///< 
    uint16_t timecount;     ///< 
} SCIF_I2CIMUPRESSUREWARRAY1111_STATE_T;


/// Sensor Controller task data (configuration, input buffer(s), output buffer(s) and internal state)
typedef struct {
    struct {
        SCIF_I2CIMUPRESSUREWARRAY1111_INPUT_T input;
        SCIF_I2CIMUPRESSUREWARRAY1111_OUTPUT_T output;
        SCIF_I2CIMUPRESSUREWARRAY1111_STATE_T state;
    } i2cimupressurewarray1111;
} SCIF_TASK_DATA_T;

/// Sensor Controller task generic control (located in AUX RAM)
#define scifTaskData    (*((volatile SCIF_TASK_DATA_T*) 0x400E00E8))


// Initialized internal driver data, to be used in the call to \ref scifInit()
extern const SCIF_DATA_T scifDriverSetup;


// Restore previous struct packing setting
#pragma pack(pop)


// AUX I/O re-initialization functions
void scifReinitTaskIo(uint32_t bvTaskIds);


// RTC-based tick generation control
void scifStartRtcTicks(uint32_t tickStart, uint32_t tickPeriod);
void scifStartRtcTicksNow(uint32_t tickPeriod);
void scifStopRtcTicks(void);


#endif
//@}


// Generated by DESKTOP-6Q8NGS0 at 2018-05-03 19:48:38.152
