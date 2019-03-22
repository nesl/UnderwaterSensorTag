/**
 *   @defgroup  eMPL
 *   @brief     Embedded Motion Processing Library
 *
 *   @{
 *       @file      mllite_test.c
 *       @brief     Test app for eMPL using the eMA and Motion Driver DMP image.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "msp430.h"
/* USB related header files*/
#include "USB_eMPL/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/types.h"
#include "USB_API/USB_Common/usb.h"
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "usbConstructs.h"

/* Clock related header files*/
#include "F5xx_F6xx_Core_Lib/HAL_UCS.h"
#include "F5xx_F6xx_Core_Lib/HAL_PMM.h"
#include "F5xx_F6xx_Core_Lib/HAL_FLASH.h"
#include "msp430_clock.h"
/* I2C related header file*/
#include "msp430_i2c.h"
/* UART related header file*/
#include "msp430_uart.h"
/* SPI related header file*/
#include "msp430_spi.h"
/* Interrupt related header file*/
#include "msp430_interrupt.h"
/* Environmental sensors related header file*/
#include "sensor_probe.h"
/* External flash related header file*/
#include "MX25_APP.h"
#include "MX25_CMD.h"
/* Battery management related header file*/
#include "max17048_driver.h"
/* eMPL related header file*/
#include "mag_disturb.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"

/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_OTHER_SENSORS	(0x100)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)
/* Default MPU HZ is limited here to 20Hz
 * to get best performance for this platform (MSP430) with
 * 12 MHz clock, blocking UART.
 * If decided to change the sample frequency recommend
 * making UART non-blocking, increase the baud rate
 * make the clock can run at 25 MHz instead of 12 MHz.
 */
#define DEFAULT_MPU_HZ  (50)  // See above notes.
#define BLINK_RATE_RED  (150) // DEFAULT_MPU_HZ*3 = 3 seconds
#define RED_ON_TIME     (5)   // 1/50*5 = 100ms
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)
#define SUCCESS 0
/* External sensor or environment sensor update rate*/
#define SAMPLE_RATE 5

/* Debug LED blinking */
#define BLINK_RATE_BLUE (15)  // SAMPLE_RATE *3 = 3 seconds
#define BLUE_ON_TIME    (1)   // 1/5*5 = 100ms
unsigned char flag_byte_1;
#define FLAG_SAMPLE_RATE 0x01

// Default used to indicate data ready
#define TOGGLE_RED_LED  (P1OUT ^= BIT2)
#define SET_RED_LED     (P1OUT |=BIT2)
#define CLEAR_RED_LED   (P1OUT &=~ BIT2)

// Default used to indicate motion detect
#define TOGGLE_BLUE_LED  (P1OUT ^=BIT4)
#define SET_BLUE_LED     (P1OUT |=BIT4)
#define CLEAR_BLUE_LED   (P1OUT &=~ BIT4)

// Default used to indicate Failure
#define TOGGLE_BOTH_LED (P1OUT ^=  (BIT4 +BIT2))
#define SET_BOTH_LED    (P1OUT |=  (BIT4 +BIT2))
#define CLEAR_BOTH_LED  (P1OUT &=~ (BIT4 +BIT2))

#define DISABLE_COMPASS
#undef DISABLE_COMPASS
#define DISABLE_EXTERNAL_CRYSTAL
#undef DISABLE_EXTERNAL_CRYSTAL

#define BATTERY_MANAGEMENT
//#undef BATTERY_MANAGEMENT
#define SOFTWARE_BSL

struct int_param_s int_param;

volatile uint8_t buttonsPressed = 0;
uint8_t button2PressedCount = 0;
uint8_t button1PressedCount = 0;
uint16_t sec3Count = 0;


unsigned char tcntr;
static volatile unsigned char enough_delay_flag;

uint8_t NoMotion_flag =0;

long count=0;
char s[5];

void checkForBSLclick();
void checkClick();
void checkClicked();
int msp430_timerA1_enable(void);
void msp430_timerA1_disable();
void init_timer_A1(unsigned int sample_rate);
void Init_TimerA2(unsigned int sample_rate);
void Stop_TimerA2();
unsigned char is_uart_enable = 0;
void check_for_usb_state();
int setup_all_sensors();
//void make_protocol(unsigned char *pressure, unsigned char *humidity,
//		unsigned char *temperature, unsigned char *light, unsigned char *uv);

void send_status_compass();
void get_eMPL_version();
int test_flash(void);
void showBatteryStatus();
void load_calibration(void);
void tellTime();

struct rx_s {
	unsigned char header[3];
	unsigned char cmd;
};

struct hal_s {
	unsigned char lp_accel_mode;
	unsigned char sensors;
	unsigned char dmp_on;
	unsigned char wait_for_tap;
	unsigned char new_gyro;
	unsigned char motion_int_mode;
	unsigned long no_dmp_hz;
	unsigned long next_pedo_ms;
	unsigned long next_temp_ms;
	unsigned long next_compass_ms;
	unsigned short report;
	unsigned char dmp_features;
	struct rx_s rx;
};
// initializing
static struct hal_s hal = { 0,0,0,0,0,0,0,0,0,0,0,0,0 };

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

/* Should be defined by your compass driver. */
extern struct driver_if_s compass_if;

//unsigned char *mpl_key = (unsigned char*)"Wrong Key";
unsigned char *mpl_key = (unsigned char*) "eMPL 5.1";

// Blink rate
unsigned int cntr_blink_red = 0;
unsigned int cntr_blink_blue = 0;


//++code generating to put inva
int val = 0;
int val2 = 0;

/* Get data from MPL.
 *
 * between new and stale data.
 */

static void read_from_mpl(void) {
	long msg = 0, data[9] = { 0 };
	int8_t accuracy = { 0 };
	unsigned long timestamp;



	long test[4]={0};
	if (inv_get_sensor_type_quat(test, &accuracy, (inv_time_t*) &timestamp)) {
		/* Sends a quaternion packet to the PC. Since this is used by the Python
		 * test app to visually represent a 3D quaternion, it's sent each time
		 * the MPL has new data.
		 */
		eMPL_send_quat(test);
	}


	if (hal.report & PRINT_ACCEL) {
		if (inv_get_sensor_type_accel(data, &accuracy,
				(inv_time_t*) &timestamp))
			eMPL_send_data(PACKET_DATA_ACCEL, data);
	}
	if (hal.report & PRINT_GYRO) {
		if (inv_get_sensor_type_gyro(data, &accuracy, (inv_time_t*) &timestamp))
			eMPL_send_data(PACKET_DATA_GYRO, data);
	}
	if (hal.report & PRINT_COMPASS) {
		if (inv_get_sensor_type_compass(data, &accuracy,
				(inv_time_t*) &timestamp)) {
			eMPL_send_data(PACKET_DATA_COMPASS, data);
		}
	}
	if (hal.report & PRINT_EULER) {
		if (inv_get_sensor_type_euler(data, &accuracy,
				(inv_time_t*) &timestamp))
			eMPL_send_data(PACKET_DATA_EULER, data);
	}
	if (hal.report & PRINT_ROT_MAT) {
		if (inv_get_sensor_type_rot_mat(data, &accuracy,
				(inv_time_t*) &timestamp))
			eMPL_send_data(PACKET_DATA_ROT, data);
	}
	if (hal.report & PRINT_HEADING) {
		if (inv_get_sensor_type_heading(data, &accuracy,
				(inv_time_t*) &timestamp))
			eMPL_send_data(PACKET_DATA_HEADING, data);
	}
	if (hal.report & PRINT_QUAT) {
		if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*) &timestamp))
			eMPL_send_data(PACKET_DATA_QUAT, data);
	}
	if (hal.report & PRINT_PEDO) {
		unsigned long timestamp_;
		msp430_get_clock_ms(&timestamp_);
		if (timestamp_ > hal.next_pedo_ms) {
			hal.next_pedo_ms = timestamp_ + PEDO_READ_MS;
			unsigned long step_count, walk_time;
			dmp_get_pedometer_step_count(&step_count);
			dmp_get_pedometer_walk_time(&walk_time);
			MPL_LOGI("Walked %ld steps over %ld milliseconds..\n",
					step_count, walk_time);
		}
	}

	/* Whenever the MPL detects a change in motion state, the application can
	 * be notified. For this example, we use an LED to represent the current
	 * motion state.
	 */
	msg = inv_get_message_level_0(
			INV_MSG_MOTION_EVENT | INV_MSG_NO_MOTION_EVENT);
	if (msg) {
		if (msg & INV_MSG_MOTION_EVENT) {
			MPL_LOGI("Motion!\n");
			NoMotion_flag =0;
			SET_BLUE_LED;
		} else if (msg & INV_MSG_NO_MOTION_EVENT) {
			MPL_LOGI("No motion!\n");
			NoMotion_flag = 1;
			CLEAR_BLUE_LED;
		} else if (msg & INV_MSG_NEW_GB_EVENT) {
			//MPL_LOGI("neither!\n");
			long gyro_bias[3], temp;
			inv_get_gyro_bias(gyro_bias, &temp);
			dmp_set_gyro_bias(gyro_bias);
		}
	}
}

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
	signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * IMPORTANT: The following matrices refer to the configuration on an internal test
 * board at InvenSense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 * The present matrix is updated to match the CA-SDK board
 * according to the board definition of XYZ. (Z is coming out)
 *
 *    |------------------------------------|
 *    |       	        	[	   .]   |J |  (-Y Board)
 *   [|                     |  BT   |   |T |    ^
 *    |                     |       |   |A |    |
 *    |         			---------   |G |    |
 *    |               					   |  -------> (-X Board)
 *    |                                    |
 *    |                               ---- |
 *   [|                              |    ||
 *    |                              |.   ||
 *  .[|                		9250<-->-|----
 *    |                               1    |
 *    |									   |
 *    |---------------[ USB  ]-------------|
 *
 *
 */
static struct platform_data_s gyro_pdata = { .orientation = { 0, 1, 0, -1, 0, 0,
		0, 0, 1 } };

#if defined MPU9150 || defined MPU9250

#ifndef DISABLE_COMPASS
//static struct platform_data_s compass_pdata = { .orientation = { 0, -1, 0, -1, 0, 0, 0, 0, -1 } };
static struct platform_data_s compass_pdata = { .orientation = { 1, 0, 0, 0, -1,
		0, 0, 0, -1 } };
#endif

#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
	.orientation = {-1, 0, 0,
		0, 1, 0,
		0, 0,-1}
};
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {.orientation = {-1, 0, 0, 0,
		-1, 0, 0, 0, 1}};
#endif

static void setup_gyro(void) {
	unsigned char mask = 0, lp_accel_was_on = 0;
	if (hal.sensors & ACCEL_ON)
		mask |= INV_XYZ_ACCEL;
	if (hal.sensors & GYRO_ON) {
		mask |= INV_XYZ_GYRO;
		lp_accel_was_on |= hal.lp_accel_mode;
	}
	if (hal.sensors & COMPASS_ON) {
		mask |= INV_XYZ_COMPASS;
		lp_accel_was_on |= hal.lp_accel_mode;
	}
	/* If you need a power transition, this function should be called with a
	 * mask of the sensors still enabled. The driver turns off any sensors
	 * excluded from this mask.
	 */
	mpu_set_sensors(mask);
	mpu_configure_fifo(mask);
	if (lp_accel_was_on) {
		unsigned short rate;
		hal.lp_accel_mode = 0;
		/* Switching out of LP accel, notify MPL of new accel sampling rate. */
		mpu_get_sample_rate(&rate);
		inv_set_accel_sample_rate(1000000L / rate);
	}
}

static void tap_cb(unsigned char direction, unsigned char count) {
	switch (direction) {
	case TAP_X_UP:
		MPL_LOGI("Tap X+ ");
		break;
	case TAP_X_DOWN:
		MPL_LOGI("Tap X- ");
		break;
	case TAP_Y_UP:
		MPL_LOGI("Tap Y+ ");
		break;
	case TAP_Y_DOWN:
		MPL_LOGI("Tap Y- ");
		break;
	case TAP_Z_UP:
		MPL_LOGI("Tap Z+ ");
		break;
	case TAP_Z_DOWN:
		MPL_LOGI("Tap Z- ");
		break;
	default:
		return;
	}
	MPL_LOGI("x%d\n", count);
	return;
}

static void orient_cb(unsigned char orientation) {
	if (orientation & ORIENTATION_X_UP)
		MPL_LOGI("Orientation X+");
	else if (orientation & ORIENTATION_X_DOWN)
		MPL_LOGI("Orientation X-");
	else if (orientation & ORIENTATION_Y_UP)
		MPL_LOGI("Orientation Y+");
	else if (orientation & ORIENTATION_Y_DOWN)
		MPL_LOGI("Orientation Y-");
	else if (orientation & ORIENTATION_Z_UP)
		MPL_LOGI("Orientation Z+");
	else if (orientation & ORIENTATION_Z_DOWN)
		MPL_LOGI("Orientation Z-");

	if (orientation & ORIENTATION_FLIP)
		MPL_LOGI(" (FLIP!)\n");
	else
		MPL_LOGI("\n");
}
static void msp430_reset(void) {
	PMMCTL0 |= PMMSWPOR;
}

static void handle_input(void) {
	char c;
	int result;
	unsigned short accel_sens;
	float gyro_sens;
	size_t store_size;
	const unsigned char header[3] = "inv";
	long gyro_bias[3], accel_bias[3];

	/* Read incoming byte and check for header.
	 * Technically, the MSP430 USB stack can handle more than one byte at a
	 * time. This example allows for easily switching to UART if porting to a
	 * different microcontroller.
	 */
	rx_new = 0;

	if (is_uart_enable)
		msp430_get_uart_rx_data((/*unsigned char*/BYTE*) &c);
	else
		cdcReceiveDataInBuffer((BYTE*) &c, 1, CDC0_INTFNUM);

	if (hal.rx.header[0] == header[0]) {
		if (hal.rx.header[1] == header[1]) {
			if (hal.rx.header[2] == header[2]) {
				memset(&hal.rx.header, 0, sizeof(hal.rx.header));
				hal.rx.cmd = c;
			} else if (c == header[2])
				hal.rx.header[2] = c;
			else
				memset(&hal.rx.header, 0, sizeof(hal.rx.header));
		} else if (c == header[1])
			hal.rx.header[1] = c;
		else
			memset(&hal.rx.header, 0, sizeof(hal.rx.header));
	} else if (c == header[0])
		hal.rx.header[0] = header[0];

//++ code I am putting to generate inva invg automatically
	if(val2 == 0){
	    hal.rx.cmd = 'a';
	    val2 = 1;
	}else if (val2 ==1){
	    hal.rx.cmd = 'g';
	    val2 = 2;
	}else if (val2 == 2){
	    hal.rx.cmd = 'c';
	    val2 = 3;
	}

	if (!hal.rx.cmd)
		return;

	switch (hal.rx.cmd) {
	/* These commands turn off individual sensors. */
	case '8':
		hal.sensors ^= ACCEL_ON;
		setup_gyro();
		if (!(hal.sensors & ACCEL_ON))
			inv_accel_was_turned_off();

		break;
	case '9':
		hal.sensors ^= GYRO_ON;
		setup_gyro();
		if (!(hal.sensors & GYRO_ON))
			inv_gyro_was_turned_off();
		break;
	case '0':
		hal.sensors ^= COMPASS_ON;
		setup_gyro();
		if (!(hal.sensors & COMPASS_ON))
			inv_compass_was_turned_off();
		break;
		/* The commands send individual sensor data or fused data to the PC. */
	case 'a':
		hal.report ^= PRINT_ACCEL;
		break;
	case 'g':
		hal.report ^= PRINT_GYRO;
		break;
	case 'c':
		hal.report ^= PRINT_COMPASS;
		break;
	case 'e':
		hal.report ^= PRINT_EULER;
		break;
	case 'r':
		hal.report ^= PRINT_ROT_MAT;
		break;
	case 'q':
		hal.report ^= PRINT_QUAT;
		break;
	case 'h':
		hal.report ^= PRINT_HEADING;
		break;
	case 'k':
		hal.report ^= PRINT_OTHER_SENSORS;
		break;
	case 'm':
		if (inv_get_magnetic_disturbance_state())
			inv_stop_magnetic_disturbance();
		else
			inv_start_magnetic_disturbance();
		break;
	case 'n':
		get_eMPL_version();
		break;
	case 'w':
		send_status_compass();
		break;
		/* This command prints out the value of each gyro register for debugging.
		 * If logging is disabled, this function has no effect.
		 */
	case 'd':
		mpu_reg_dump();
		break;
		/* Test out low-power accel mode. */
	case 'p':
		if (hal.dmp_on)
			/* LP accel is not compatible with the DMP. */
			break;
		mpu_lp_accel_mode(20);
		/* When LP accel mode is enabled, the driver automatically configures
		 * the hardware for latched interrupts. However, the MCU sometimes
		 * misses the rising/falling edge, and the hal.new_gyro flag is never
		 * set. To avoid getting locked in this state, we're overriding the
		 * driver's configuration and sticking to unlatched interrupt mode.
		 *
		 * IMPORTANT: The MCU supports level-triggered interrupts.
		 */
		mpu_set_int_latched(0);
		hal.sensors &= ~(GYRO_ON | COMPASS_ON);
		hal.sensors |= ACCEL_ON;
		hal.lp_accel_mode = 1;
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();
		break;
		/* This snippet of code shows how to load and store calibration data from
		 * the MPL. For the MSP430, the flash segment must be unlocked before
		 * reading/writing and locked when no longer in use. When porting to a
		 * different microcontroller, flash memory might be accessible at anytime,
		 * or may not be available at all.
		 */
	case 'l':
		inv_get_mpl_state_size(&store_size);
		if (store_size > FLASH_SIZE) {
			MPL_LOGE("Calibration data exceeds available memory.\n");
			break;
		}
		FCTL3 = FWKEY;
		inv_load_mpl_states(FLASH_MEM_START, store_size);
		FCTL3 = FWKEY + LOCK;
		inv_accel_was_turned_off();
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();
		break;
	case 's':
		inv_get_mpl_state_size(&store_size);
		if (store_size > FLASH_SIZE) {
			MPL_LOGE("Calibration data exceeds available memory.\n");
			return;
		} else {
			unsigned char mpl_states[100], tries = 5, erase_result;
			inv_save_mpl_states(mpl_states, store_size);
			while (tries--) {
				/* Multiple attempts to erase current data. */
				Flash_SegmentErase((uint16_t*) FLASH_MEM_START);
				erase_result = Flash_EraseCheck((uint16_t*) FLASH_MEM_START,
						store_size >> 1);
				if (erase_result == FLASH_STATUS_OK)
					break;
			}
			if (erase_result == FLASH_STATUS_ERROR) {
				MPL_LOGE("Could not erase user page for calibration "
				"storage.\n");
				break;
			}
			FlashWrite_8(mpl_states, FLASH_MEM_START, store_size);

		}
		inv_accel_was_turned_off();
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();

		break;
		/* The hardware self test can be run without any interaction with the
		 * MPL since it's completely localized in the gyro driver. Logging is
		 * assumed to be enabled; otherwise, a couple LEDs could probably be used
		 * here to display the test results.
		 */
	case 't':
		CLEAR_BOTH_LED;
		result = mpu_run_self_test(gyro_bias, accel_bias);
		if (result == 0x7) {
			MPL_LOGI("Passed!\r\n");
			MPL_LOGI("accel: %7.4f %7.4f %7.4f\r\n",
					accel_bias[0]/65536.f, accel_bias[1]/65536.f, accel_bias[2]/65536.f);
			MPL_LOGI("gyro: %7.4f %7.4f %7.4f\r\n",
					gyro_bias[0]/65536.f, gyro_bias[1]/65536.f, gyro_bias[2]/65536.f);
			/* MPL expects biases in hardware units << 16, but self test returns
			 * biases in g's << 16.
			 */
			mpu_get_accel_sens(&accel_sens);
			accel_bias[0] *= accel_sens;
			accel_bias[1] *= accel_sens;
			accel_bias[2] *= accel_sens;
			inv_set_accel_bias(accel_bias, 3);
			mpu_get_gyro_sens(&gyro_sens);
			gyro_bias[0] = (long) (gyro_bias[0] * gyro_sens);
			gyro_bias[1] = (long) (gyro_bias[1] * gyro_sens);
			gyro_bias[2] = (long) (gyro_bias[2] * gyro_sens);
			inv_set_gyro_bias(gyro_bias, 3);
		} else {
			if (!(result & 0x1))
				MPL_LOGE("Gyro failed.\n");
			if (!(result & 0x2))
				MPL_LOGE("Accel failed.\n");
			if (!(result & 0x4))
				MPL_LOGE("Compass failed.\n");
		}
		inv_accel_was_turned_off();
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();
		SET_BOTH_LED;
		msp430_delay_ms(250);
		CLEAR_BOTH_LED;
		msp430_delay_ms(250);
		SET_BOTH_LED;
		msp430_delay_ms(250);
		CLEAR_BOTH_LED;
		msp430_delay_ms(250);

		/* Store the calibration as a part of application but could be seperate*/
		inv_get_mpl_state_size(&store_size);
		if (store_size > FLASH_SIZE) {
			MPL_LOGE("Calibration data exceeds available memory.\n");
			return;
		} else {
			unsigned char mpl_states[100], tries = 5, erase_result;
			inv_save_mpl_states(mpl_states, store_size);
			while (tries--) {
				/* Multiple attempts to erase current data. */
				Flash_SegmentErase((uint16_t*) FLASH_MEM_START);
				erase_result = Flash_EraseCheck((uint16_t*) FLASH_MEM_START,
						store_size >> 1);
				if (erase_result == FLASH_STATUS_OK)
					break;
			}
			if (erase_result == FLASH_STATUS_ERROR) {
				MPL_LOGE("Could not erase user page for calibration "
				"storage.\n");
				break;
			}
			FlashWrite_8(mpl_states, FLASH_MEM_START, store_size);

		}
		/* Let MPL know that contiguity was broken. */
		inv_accel_was_turned_off();
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();
		SET_BOTH_LED;
		msp430_delay_ms(250);
		CLEAR_BOTH_LED;
		msp430_delay_ms(250);
		SET_BOTH_LED;
		msp430_delay_ms(250);
		CLEAR_BOTH_LED;
		msp430_delay_ms(250);

		break;
		/* Depending on your application, sensor data may be needed at a faster or
		 * slower rate. These commands can speed up or slow down the rate at which
		 * the sensor data is pushed to the MPL.
		 *
		 * In this example, the compass rate is never changed.
		 */
	case '1':
		if (hal.dmp_on) {
			dmp_set_fifo_rate(10);
			inv_set_quat_sample_rate(100000L);
		} else
			mpu_set_sample_rate(10);
		inv_set_gyro_sample_rate(100000L);
		inv_set_accel_sample_rate(100000L);
		break;
	case '2':
		if (hal.dmp_on) {
			dmp_set_fifo_rate(20);
			inv_set_quat_sample_rate(50000L);
		} else
			mpu_set_sample_rate(20);
		inv_set_gyro_sample_rate(50000L);
		inv_set_accel_sample_rate(50000L);
		break;
	case '3':
		if (hal.dmp_on) {
			dmp_set_fifo_rate(40);
			inv_set_quat_sample_rate(25000L);
		} else
			mpu_set_sample_rate(40);
		inv_set_gyro_sample_rate(25000L);
		inv_set_accel_sample_rate(25000L);
		break;
	case '4':
		if (hal.dmp_on) {
			dmp_set_fifo_rate(50);
			inv_set_quat_sample_rate(20000L);
		} else
			mpu_set_sample_rate(50);
		inv_set_gyro_sample_rate(20000L);
		inv_set_accel_sample_rate(20000L);
		break;
		/*	case '5':  Clock can not be set for more than 50 Hz!
		 if (hal.dmp_on) {
		 dmp_set_fifo_rate(100);
		 inv_set_quat_sample_rate(10000L);
		 } else
		 mpu_set_sample_rate(100);
		 inv_set_gyro_sample_rate(10000L);
		 inv_set_accel_sample_rate(10000L);
		 break;*/
	case ',':
		/* Set hardware to interrupt on gesture event only. */
		dmp_set_interrupt_mode(DMP_INT_GESTURE);
		break;
	case '.':
		/* Set hardware to interrupt periodically. */
		dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
		break;
	case '6':
		/* Toggle pedometer display. */
		hal.report ^= PRINT_PEDO;
		break;
	case '7':
		/* Reset pedometer. */
		dmp_set_pedometer_step_count(0);
		dmp_set_pedometer_walk_time(0);
		break;
	case 'f':
		if (hal.lp_accel_mode)
			/* LP accel is not compatible with the DMP. */
			return;
		/* Toggle DMP. */
		if (hal.dmp_on) {
			unsigned short dmp_rate;
			unsigned char mask = 0;
			hal.dmp_on = 0;
			mpu_set_dmp_state(0);
			/* Restore FIFO settings. */
			if (hal.sensors & ACCEL_ON)
				mask |= INV_XYZ_ACCEL;
			if (hal.sensors & GYRO_ON)
				mask |= INV_XYZ_GYRO;
			if (hal.sensors & COMPASS_ON)
				mask |= INV_XYZ_COMPASS;
			mpu_configure_fifo(mask);
			/* When the DMP is used, the hardware sampling rate is fixed at
			 * 200Hz, and the DMP is configured to downsample the FIFO output
			 * using the function dmp_set_fifo_rate. However, when the DMP is
			 * turned off, the sampling rate is set at 200Hz. This could be
			 * handled in inv_gyro.c, but it would need to know that
			 * inv_gyro_dmp_android.c exists. To avoid this, we'll just put the
			 * extra logic in the application layer.
			 *
			 */
			dmp_get_fifo_rate(&dmp_rate);
			mpu_set_sample_rate(dmp_rate);
			inv_quaternion_sensor_was_turned_off();
			MPL_LOGI("DMP disabled.\n");
		} else {
			unsigned short sample_rate;
			hal.dmp_on = 1;
			/* Preserve current FIFO rate. */
			mpu_get_sample_rate(&sample_rate);
			dmp_set_fifo_rate(sample_rate);
			inv_set_quat_sample_rate(1000000L / sample_rate);
			mpu_set_dmp_state(1);
			MPL_LOGI("DMP enabled.\n");
		}
		break;
	case 'x':
		msp430_reset();
		break;
	case 'v':
		/* Toggle LP quaternion. */
		hal.dmp_features ^= DMP_FEATURE_LP_QUAT;
		dmp_enable_feature(hal.dmp_features);
		if (!(hal.dmp_features & DMP_FEATURE_LP_QUAT)) {
			inv_quaternion_sensor_was_turned_off();
			MPL_LOGI("LP quaternion disabled.\n");
		} else
			MPL_LOGI("LP quaternion enabled.\n");
		break;
	case 'i':
		hal.motion_int_mode = 1;
		break;

	case 'y':
		showBatteryStatus();
		tellTime();
		break;

	default:
		break;
	}
	hal.rx.cmd = 0;
}

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
static void gyro_data_ready_cb(void) {
	hal.new_gyro = 1;
	cntr_blink_red++;
	if (cntr_blink_red <= RED_ON_TIME) {
		SET_RED_LED;
	} else {
		CLEAR_RED_LED;
	}
	if (cntr_blink_red >= BLINK_RATE_RED) {
		cntr_blink_red = 0;
	}
}



/**
 * CA-SDK release board
 * Please refer to the WCAP_B_20130320.pdf for more details
 * Port 1: 1.1=> Input, others => Output low
 * Port 2: 0,1,2,3,6,5 => Input, while 4 is output high always power on
 * Port 3: 3,4=UART peripheral, reset will be enabled during
 */
static inline void init_ports(void) {

	P1SEL = 0x00; // standard I/o;
	P1DIR &= ~BIT1; // Battery management IC
	P1REN |= BIT1;  //Pull-up/Pull down resistor enable
	P1OUT |= BIT1;	// Select Pull-up mode

	P1DIR |= (BIT0 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7);
	P1OUT &= ~(BIT0 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7);

	P2SEL = 0x00; // standard I/o;
	P2DIR &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT5+ BIT6);
	P2DIR |= (BIT4 + BIT7);
	P2OUT &= ~(BIT7);
	/* SW1, can be used to power up the board if JP1 is
	 * not loaded and P2.4 high could keep the board always
	 * on. In the present SW we assume JP1 is loaded. P2.4 is made
	 * just in case there is loose connection on JP1.
	 */
	P2OUT |= BIT4;

	P3SEL = 0x00; // Standard I/o
	P3DIR = 0xFF; // output
	P3OUT = 0x00; // output low
	P3SEL |= (BIT3 + BIT4); // UART
	P3DIR |= BIT3; // UART Tx
	P3DIR &= ~BIT4; // UART Rx

	P4SEL = 0x00; // standard I/o
	P4DIR = 0xFF; // output direction
	P4OUT = 0x00; // output
	//P4SEL    |= (BIT1+BIT2+BIT3);
	//P4DIR &=~ (BIT2);//SOMI
	//P4OUT |= BIT0; // Turn off Flash


	P5SEL = 0x00; // standard I/o
	P5OUT = 0x00; // output
	P5DIR = 0xFF; // output low

	P6SEL = 0x00; // standard I/o
	P6DIR = 0x00; // output
	P6OUT = 0x00; // output low

}


/* Set up MSP430 peripherals. */
static inline void platform_init(void) {
	WDTCTL = WDTPW | WDTHOLD;
	init_ports();
	SetVCore(2);
#ifndef DISABLE_EXTERNAL_CRYSTAL
	msp430_clock_init(12000000L, 2);
#else
	msp430_clock_init(12000000L, 0);
#endif

	/* Initialize the SPI */
	if (USB_init() != kUSB_succeed)
		msp430_reset();

	msp430_i2c_enable();
	msp430_uart_init();
	msp430_int_init();


#ifndef DISABLE_EXTERNAL_CRYSTAL
	USB_setEnabledEvents(kUSB_allUsbEvents);
	if (USB_connectionInfo() & kUSB_vbusPresent) {
		if (USB_enable() == kUSB_succeed) {
			USB_reset();
			USB_connect();
		} else
			msp430_reset();
	}
#endif
}

unsigned long prev_timestamp = 0;

void run_self_test() {
	int result;
	unsigned short accel_sens;
	float gyro_sens;
	long gyro_bias[3], accel_bias[3];

	unsigned long current_timestamp = 0;

	msp430_get_clock_ms(&current_timestamp);

	if (current_timestamp - prev_timestamp <= 5000) {
		CLEAR_BOTH_LED;
		result = mpu_run_self_test(gyro_bias, accel_bias);
		if (result == 0x7) {
			MPL_LOGI("Passed!\r\n");
			MPL_LOGI("accel: %7.4f %7.4f %7.4f\r\n",
					accel_bias[0]/65536.f, accel_bias[1]/65536.f, accel_bias[2]/65536.f);
			MPL_LOGI("gyro: %7.4f %7.4f %7.4f\r\n",
					gyro_bias[0]/65536.f, gyro_bias[1]/65536.f, gyro_bias[2]/65536.f);
			/* MPL expects biases in hardware units << 16, but self test returns
			 * biases in g's << 16.
			 */
			mpu_get_accel_sens(&accel_sens);
			accel_bias[0] *= accel_sens;
			accel_bias[1] *= accel_sens;
			accel_bias[2] *= accel_sens;
			inv_set_accel_bias(accel_bias, 3);
			mpu_get_gyro_sens(&gyro_sens);
			gyro_bias[0] = (long) (gyro_bias[0] * gyro_sens);
			gyro_bias[1] = (long) (gyro_bias[1] * gyro_sens);
			gyro_bias[2] = (long) (gyro_bias[2] * gyro_sens);
			inv_set_gyro_bias(gyro_bias, 3);
		} else {
			if (!(result & 0x1))
				MPL_LOGE("Gyro failed.\r\n");
			if (!(result & 0x2))
				MPL_LOGE("Accel failed.\r\n");
			if (!(result & 0x4))
				MPL_LOGE("Compass failed.\r\n");
		}
		/* Let MPL know that contiguity was broken. */
		inv_accel_was_turned_off();
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();

		SET_BOTH_LED;
		msp430_delay_ms(250);
		CLEAR_BOTH_LED;
		msp430_delay_ms(250);
		SET_BOTH_LED;
		msp430_delay_ms(250);
		CLEAR_BOTH_LED;
		msp430_delay_ms(250);
	}
}

void Buttons_interruptEnable(uint16_t buttonsMask, uint8_t isRisingEdge) {
	if (isRisingEdge)
		P2IES |= buttonsMask; //select rising edge trigger
	else
		P2IES &= ~buttonsMask; //select falling edge trigger

	P2IFG &= ~buttonsMask; //clear flags
	P2IE |= buttonsMask; //enable interrupts
}
void Buttons_interruptDisable(uint16_t buttonsMask) {
	P2IE &= ~buttonsMask;
}




/*
 * @brief, flash alive test. To make sure HW is working
 * The driver code was re-written from the existing code
 * from www.macronix.com reference low level driver. Please
 * check for latest driver at macrnoix website.
 */
int test_flash(void){
	unsigned char status=SUCCESS;
	unsigned char test_result;
	/*--------- TESTING BEGINS HERE ------------*/
		msp430_delay_ms(1000);
		/* TEST 1 FLASH TEST */
		 test_result=FlashID_Test();
		 if(test_result)
		 {
			 msp430_delay_ms(1000);
			 test_result=FlashReadWrite_Test(0);
			 if(test_result)
			 {
				 status=SUCCESS;
			 }
			 else
			 {
				 status=1;
			 }
		 }
		 else {
			 status = 1;
		 }

	return (status);
}







unsigned char pressure[2] = { 0 };
unsigned char humidity[2] = { 0 };
unsigned char temperture[2] = { 0 };
unsigned char light[2] = { 0 };
unsigned char uv_index[2] = { 0 };
volatile unsigned char fInvokeBSL = 0;

int testval;
void main(void) {
	inv_error_t result;

//++probably take out for the ones that are not used to disable compass, just define disable compass
	unsigned char accel_fsr;
#ifndef DISABLE_COMPASS
	unsigned char new_compass = 0;
#endif
	unsigned char new_temp = 0;
	unsigned short gyro_rate, gyro_fsr;
#ifndef DISABLE_COMPASS
	unsigned short compass_fsr;
#endif
	unsigned long timestamp;


	/* Set up MSP430 hardware. */
	platform_init();

	Initial_Spi();

	/* Set up gyro.
	 * Every function preceded by mpu_ is a driver function and can be found
	 * in inv_mpu.h.
	 */
	int_param.cb = gyro_data_ready_cb;
	int_param.pin = INT_PIN_P20;
	int_param.lp_exit = INT_EXIT_LPM0;
	int_param.active_low = 1;

	result = mpu_init(&int_param);
	if (result) {
		MPL_LOGE("Could not initialize gyro.\n");
		msp430_reset();
	}

	/*
	 * Will setup all the sensor and if not successful will
	 * block the code execution, restart.
	 */
//++ commented proximity sensor checking part in setup_all_sensors()
//	if (setup_all_sensors() != SUCCESS){
//		MPL_LOGE("External Sensor Failed failed.\n");
//		msp430_reset();
//	}
	CLEAR_BOTH_LED;

    #ifdef BATTERY_MANAGEMENT
        /*Battery management code*/
        if(max17048_probe() != SUCCESS){
            //MPL_LOGE("Battery Management IC failed.\n");
           // print("unsuccessful");
            msp430_reset();
        }
    #else
        // Do nothing, assume it is a pass.
        // Would not go through this part because battery management is defined
    #endif

    /*
     * Test the external flash using SPI. Sanity test to
     *  make sure hardware is working
     */
    if(test_flash()!=SUCCESS){
         MPL_LOGE("External Flash IC failed.\n");
         msp430_reset();
    }

    /* If you're not using an MPU9150 AND you're not using DMP features, this
     * function will place all slaves on the primary bus.
     * mpu_set_bypass(1);
     */
    result = inv_init_mpl();
    if (result) {
        MPL_LOGE("Could not initialize MPL.\n");
        msp430_reset();
    }

    /* Compute 6-axis and 9-axis quaternions. */
    inv_enable_quaternion();
    #ifndef DISABLE_COMPASS
        inv_enable_9x_sensor_fusion();
    #endif

    /* The MPL expects compass data at a constant rate (matching the rate
     * passed to inv_set_compass_sample_rate). If this is an issue for your
     * application, call this function, and the MPL will depend on the
     * timestamps passed to inv_build_compass instead.
     *
     * inv_9x_fusion_use_timestamps(1);
     */

    /* This function has been deprecated.
     * inv_enable_no_gyro_fusion();
     */

    /* Update gyro biases when not in motion.
     * WARNING: These algorithms are mutually exclusive.
     */
    inv_enable_fast_nomot();
    /* inv_enable_motion_no_motion(); */
    /* inv_set_no_motion_time(1000); */

    /* Update gyro biases when temperature changes. */
    inv_enable_gyro_tc();

    /* This algorithm updates the accel biases when in motion. A more accurate
     * bias measurement can be made when running the self-test (see case 't' in
     * handle_input), but this algorithm can be enabled if the self-test can't
     * be executed in your application.
     *
     * inv_enable_in_use_auto_calibration();
     */

    /* Compass calibration algorithms. */
    #ifndef DISABLE_COMPASS
       inv_enable_vector_compass_cal();
       inv_enable_magnetic_disturbance();
    #endif
    /* If you need to estimate your heading before the compass is calibrated,
     * enable this algorithm. It becomes useless after a good figure-eight is
     * detected, so we'll just leave it out to save memory.
     * inv_enable_heading_from_gyro();
     */

    /* Allows use of the MPL APIs in read_from_mpl. */
       inv_enable_eMPL_outputs();


    result = inv_start_mpl();
    if (result == INV_ERROR_NOT_AUTHORIZED) {
        while (1) {
            MPL_LOGE("Not authorized.\n");
            SET_BOTH_LED;
            msp430_delay_ms(5000);
            CLEAR_BOTH_LED;
            msp430_delay_ms(5000);
        }
    }

    if (result) {
        MPL_LOGE("Could not start the MPL.\n");
        msp430_reset();
    }

    /* Get/set hardware configuration. Start gyro. */
        /* Wake up all sensors. */
//++ wake up only the sensors that needs to be used
    #ifndef DISABLE_COMPASS
        mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    #else
        mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL );
    #endif

    /* Push both gyro and accel data into the FIFO. */
//++push only the one that is being used
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
    /* The compass sampling rate can be less than the gyro/accel sampling rate.
    * Use this function for proper power management.
    */
    #ifndef DISABLE_COMPASS
        mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
    #endif
    /* Read back configuration in case it was set improperly. */
//++only read back and check on the ones that we use.
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
    #ifndef DISABLE_COMPASS
        mpu_get_compass_fsr(&compass_fsr);
    #endif
        /* Sync driver configuration with MPL. */
        /* Sample rate expected in microseconds. */
        inv_set_gyro_sample_rate(1000000L / gyro_rate);
        inv_set_accel_sample_rate(1000000L / gyro_rate);
        /* The compass rate is independent of the gyro and accel rates. As long as
         * inv_set_compass_sample_rate is called with the correct value, the 9-axis
         * fusion algorithm's compass correction gain will work properly.
         */
    #ifndef DISABLE_COMPASS
        inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
    #endif
    //printf("done");
	//return;

    /* Set chip-to-body orientation matrix.
     * Set hardware units to dps/g's/degrees scaling factor.
     */
    inv_set_gyro_orientation_and_scale(
    inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long) gyro_fsr << 15);
    inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long) accel_fsr << 15);
    #ifndef DISABLE_COMPASS

        inv_set_compass_orientation_and_scale(
                inv_orientation_matrix_to_scalar(compass_pdata.orientation),
                (long) compass_fsr << 15);

        /* Initialize HAL state variables. */
        hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;

    #else
        hal.sensors = ACCEL_ON | GYRO_ON;
    #endif

    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

    /* Compass reads are handled by scheduler. */
//++ gets time!!!!!!!!!!!!!!!!!!!!
    msp430_get_clock_ms(&timestamp);

//++PART1 PART1 PART1 PART1 PART1 PART1 PART1

    //will check if the button 3 is pressed
    //for self calibration in the start
    unsigned long current_timestamp = 0;
    msp430_get_clock_ms(&current_timestamp);
    msp430_get_clock_ms(&timestamp);

    // if button click in first 3 seconds do calibration!
    Buttons_interruptDisable(BIT6);
    while ((timestamp - current_timestamp) < 3000) {
        if (!(P2IN & BIT6)) {
            run_self_test();
            break;
        }
        msp430_get_clock_ms(&timestamp);
    }

    msp430_reg_int_cb(checkClicked, INT_PIN_P26, INT_EXIT_LPM0, 0);

    __enable_interrupt();

    init_timer_A1((unsigned int) SAMPLE_RATE);


    msp430_get_clock_ms(&prev_timestamp);

    load_calibration();


    //all the setup ends here!

    while (1) {
//++connect to bluetooth
        check_for_usb_state();
//++PART2 PART2 PART2 PART2 PART2 PART2 PART2


        unsigned long sensor_timestamp;
        int new_data = 0;

//++code I am putting in to automatically run inva invg invc

        if(val < 3){
            rx_new = 1;
            val += 1;
        }

        if (rx_new) {
            /* A byte has been received via USB. See handle_input for a list of
             * valid commands.
             */
            handle_input();
        }
        msp430_get_clock_ms(&timestamp);

        /* We're not using a data ready interrupt for the compass, so we'll
                 * make our compass reads timer-based instead.
                 */
        #ifndef DISABLE_COMPASS
                if ((timestamp > hal.next_compass_ms) && !hal.lp_accel_mode
                        && hal.new_gyro && (hal.sensors & COMPASS_ON)) {
                    hal.next_compass_ms = timestamp + COMPASS_READ_MS;
                    new_compass = 1;
                }
        #endif


        /* Temperature data doesn't need to be read with every gyro sample.
         * Let's make them timer-based like the compass reads.
         */
        if (timestamp > hal.next_temp_ms) {
               hal.next_temp_ms = timestamp + TEMP_READ_MS;
               new_temp = 1;
        }

        if (hal.motion_int_mode) {
                    /* Enable motion interrupt. */
                    mpu_lp_motion_interrupt(500, 1, 5);
                    /* Notify the MPL that contiguity was broken. */
                    inv_accel_was_turned_off();
                    inv_gyro_was_turned_off();
                    inv_compass_was_turned_off();
                    inv_quaternion_sensor_was_turned_off();
                    /* Wait for the MPU interrupt. */
                    while (!hal.new_gyro)
                        __bis_SR_register(LPM0_bits + GIE);
                    /* Restore the previous sensor configuration. */
                    mpu_lp_motion_interrupt(0, 0, 0);
                    hal.motion_int_mode = 0;
       }

       if (!hal.sensors || !hal.new_gyro) {
                    /* Put the MSP430 to sleep until a timer interrupt or data ready
                     * interrupt is detected.
                     */
                    __bis_SR_register(LPM0_bits + GIE);
                    continue;
       }

//++PART3 PART3  PART3  PART3  PART3  PART3  PART3  //puts as : else if (hal.new_gyro)

//sensor values are reset through this function??
       if (hal.new_gyro) {
           short gyro[3], accel_short[3];
           unsigned char sensors, more;
           long accel[3], temperature;
           /* This function gets new data from the FIFO. The FIFO can contain
            * gyro, accel, both, or neither. The sensors parameter tells the
            * caller which data fields were actually populated with new data.
            * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
            * being filled with accel data. The more parameter is non-zero if
            * there are leftover packets in the FIFO. The HAL can use this
            * information to increase the frequency at which this function is
            * called.
            */

            hal.new_gyro = 0;
            mpu_read_fifo(gyro, accel_short, &sensor_timestamp, &sensors,
            &more);
            if (more)
            hal.new_gyro = 1;
            if (sensors & INV_XYZ_GYRO) {
                /* Push the new data to the MPL. */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp) {
                    new_temp = 0;
                    /* Temperature only used for gyro temp comp. */
                    mpu_get_temperature(&temperature, &sensor_timestamp);
                    inv_build_temp(temperature, sensor_timestamp);
                }
            }
            if (sensors & INV_XYZ_ACCEL) {
                accel[0] = (long) accel_short[0];
                accel[1] = (long) accel_short[1];
                accel[2] = (long) accel_short[2];
                inv_build_accel(accel, 0, sensor_timestamp);
                new_data = 1;
            }
         }
        #ifndef DISABLE_COMPASS
                if (new_compass) {
                    short compass_short[3];
                    long compass[3];
                    new_compass = 0;
                    /* For any MPU device with an AKM on the auxiliary I2C bus, the raw
                     * magnetometer registers are copied to special gyro registers.
                     */
                    if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) {
                        compass[0] = (long) compass_short[0];
                        compass[1] = (long) compass_short[1];
                        compass[2] = (long) compass_short[2];
                        /* NOTE: If using a third-party compass calibration library,
                         * pass in the compass data in uT * 2^16 and set the second
                         * parameter to INV_CALIBRATED | acc, where acc is the
                         * accuracy from 0 to 3.
                         */
                        inv_build_compass(compass, 0, sensor_timestamp);
                    }
                    new_data = 1;
                }
        #endif

        if (new_data) {
            inv_execute_on_data();
            /* This function reads bias-compensated sensor data and sensor
             * fusion outputs from the MPL. The outputs are formatted as seen
             * in eMPL_outputs.c. This function only needs to be called at the
             * rate requested by the host.
             */

            read_from_mpl();

//++ commented for sensors part
//            if ((flag_byte_1 & FLAG_SAMPLE_RATE) == FLAG_SAMPLE_RATE) {
//                flag_byte_1 &= ~FLAG_SAMPLE_RATE;
//                get_light_data(light);
//                get_uv_data(uv_index);
//                get_ht_data(humidity, temperture);
//                msp430_delay_ms(1);
//                get_pressure_data(pressure); // Pressure has to be updated more often.
//                make_protocol(pressure, humidity, temperture, light, uv_index);
//                cntr_blink_blue++;
//                if (cntr_blink_blue <= BLUE_ON_TIME) {
//                    if (NoMotion_flag == 0){
//                        SET_BLUE_LED;
//                        //MPL_LOGI("blue light on\n");
//                    }
//                }else {
//                    CLEAR_BLUE_LED;
//                    //MPL_LOGI("RED light on\n");
//                }
//                if (cntr_blink_blue >= BLINK_RATE_BLUE) {
//                    cntr_blink_blue = 0;
//                    //MPL_LOGI("counter to zero\n");
//                }
//            }

        }

    }


}


/*
 * @brief, To know the USB connection status and accordingly
 * switch to UART in that case.
 */
void check_for_usb_state() {

	BYTE usb_state = USB_connectionState();

	switch (usb_state) {
	/* This represents the main loop when USB is disconnected. Since the
	 * device is powered by VBUS, this case is unused.
	 */
	case ST_USB_DISCONNECTED:
		/* Idea here is to use this state to enable UART control
		 * Make sure the UART Rx and Tx interrupt is enabled once
		 * Modify the handle input based on the Uart Rx
		 */
		if (!is_uart_enable) {
			//MPL_LOGI("uart enable  1\n");
			msp430_uart_enable();
			is_uart_enable = 1;
		}
		break;
		/* Unused. */
	case ST_USB_CONNECTED_NO_ENUM:
		if (!is_uart_enable) {
			msp430_uart_enable();
			//MPL_LOGI("uart enabled\n");
			is_uart_enable = 1;
		}
		break;
		/* This represents the main loop when USB is connected. */
	case ST_ENUM_ACTIVE:
		/* This represents the main loop when the MSP430 is suspended by the PC.
		 * If the application needs to do anything at this time, take care to
		 * draw as little current as possible.
		 */
		//This mode is enabled when USB is connected. Ideally disable UART mode.

		if(is_uart_enable){
			is_uart_enable = 0; // Repeated every time.
			//MPL_LOGI("uart disbaled\n");
			msp430_uart_disable();
		}
		break;
	case ST_ENUM_SUSPENDED:
		__bis_SR_register(LPM3_bits + GIE);
		break;
		/* This represents the short time period when the MSP430 is being
		 * enumerated on the PC. Flash an LED or something if you're trying
		 * to be cool.
		 */
	case ST_ENUM_IN_PROGRESS:
		/* The MSP430 was suspended before it could be enumerated. */
		break;
	case ST_NOENUM_SUSPENDED:
		if (!is_uart_enable) {
			//MPL_LOGI("uart enable  1\n");
			msp430_uart_enable();
			is_uart_enable = 1;
		}
		break;
	case ST_ERROR:
		msp430_reset();
		break;
	default:
		break;
	}

}

/* * ======== Init_TimerA1 ========*/

void init_timer_A1(unsigned int sample_rate) {
	TA1CTL &= ~MC_1; //Turn off Timer
	TA1CCTL0 = CCIE; //CCR0 interrupt enabled
	TA1CTL = TASSEL_1 + TACLR; //ACLK, clear TAR
	if (sample_rate > 10000) {
		sample_rate = 10000;
	}
	if (sample_rate < 1) {
		sample_rate = 1;
	}
	TA1CCR0 = (unsigned int) (32767.0f / sample_rate - 1); // ~ 1 HZ frequency.
	TA1CTL |= MC_1; // Up mode. Count up to TA1CCR0
}

void msp430_timerA1_disable() {
	TA1CCTL0 &= ~CCIE;
}

int msp430_timerA1_enable(void) {
	TA1CCTL0 |= CCIE;
	return 0;
}

/*
 * ======== TIMER1_A0_ISR ========*/
//unsigned int timer_sec = 0;
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void) {
	flag_byte_1 |= FLAG_SAMPLE_RATE;
	/*if (++timer_sec > 4) {
	 timer_sec = 0;
	 flag_byte_1 |= FLAG_SAMPLE_RATE;
	 }*/
	__bic_SR_register_on_exit(LPM3_bits);
}

/**
 *  The task is to probe the sensor whenever who-am-i is
 *  available and do initial setup wherever required.
 *  External sensors are as follows
 *  1) UV sensor- CM3512
 *  2) Humidity-Temperature sensor - SHT21
 *  3) Pressure sensor- HSPPAD032A
 *  4) Light-Proximity sensor - cm36682
 */
int setup_all_sensors() {
	SET_BLUE_LED;
	SET_RED_LED;

	msp430_delay_ms(2000); // wait

	CLEAR_BOTH_LED; // Indicate test started.

	msp430_delay_ms(1000);

	/* UV sensor probe */
	if (probe_cm3512_uv_chip() == SUCCESS) {
		SET_BLUE_LED;
	} else {
		SET_RED_LED;
		MPL_LOGI("\nUV (CM3512) sensor failed\n");
		return -1;
	}

	msp430_delay_ms(500);
	CLEAR_BLUE_LED;
	CLEAR_RED_LED;
	msp430_delay_ms(500);

	/* Humidity-Temperature sensor probe */
	if (probe_sht21_th_chip() == SUCCESS) {
		SET_BLUE_LED;
	} else {
		SET_RED_LED;
		MPL_LOGI("\nHumidity-Temperature (SHT21)  sensor failed\n");
		return -1;
	}
	msp430_delay_ms(500);
	CLEAR_BLUE_LED;
	CLEAR_RED_LED;
	msp430_delay_ms(500);

	/* Pressure sensor probe */
	if (probe_alpsHS_p_chip() == SUCCESS) {
		SET_BLUE_LED;
	} else {
		SET_RED_LED;
		MPL_LOGI("\nPressure (Alps-HSPPAD032A) sensor failed\n");
		return -1;
	}
	msp430_delay_ms(500);
	CLEAR_BLUE_LED;
	CLEAR_RED_LED;
	msp430_delay_ms(500);

	/* Light-Proximity sensor probe */

//	if (probe_cm3668_lp_chip() == SUCCESS) {
//
//		SET_BLUE_LED;
//	} else {
//		SET_RED_LED;
//		MPL_LOGI("\nCapella CM36882 Light proximity sensor failed\n");
//		return -1;
//	}
//	msp430_delay_ms(500);
//	CLEAR_BLUE_LED;
//	CLEAR_RED_LED;
	msp430_delay_ms(500);
	SET_BOTH_LED;


	return 0;
}

//++ commented to take out sensor part
//#define PACKET_LENGTH 23
//
//void make_protocol(unsigned char *pressure, unsigned char *humidity,
//		unsigned char *temperature, unsigned char *light, unsigned char *uv) {
//	/*
//	 *header =1
//	 * p     =2
//	 * h     =2
//	 * t     =2
//	 * l     =2
//	 * uv    =2
//	 * footer=2
//	 * --------
//	 * Total =13
//	 * --------
//	 */
//	char out[PACKET_LENGTH], i, j;
//	out[0] = '$';
//	out[1] = PACKET_OTHER_SENSORS;
//	out[2] = PACKET_DATA_OTHER_SENSORS; //'1'; // 49 ,
//	for (i = 0, j = 0; i < 2; i++, j++) {
//		out[i + 3] = pressure[j];
//		out[i + 5] = humidity[j];
//		out[i + 7] = temperature[j];
//		out[i + 9] = light[j];
//		out[i + 11] = uv[j];
//	}
//	out[13] = '\r';
//	out[14] = '\n';
//
//	if (is_uart_enable == 0) {
//		cdcSendDataWaitTilDone((BYTE*) out, PACKET_LENGTH, CDC0_INTFNUM, 100);
//	} else {
//		msp430_uart_tx(PACKET_LENGTH, (char *) out);
//	}
//
//	if (hal.report & PRINT_OTHER_SENSORS)
//		eMPL_send_data(PACKET_DATA_OTHER_SENSORS, (long*)&out[3]);
//
//}


/*
 * ======== TIMER2_A0_ISR ========
 */
#pragma vector=TIMER2_A0_VECTOR
__interrupt void TIMER2_A0_ISR(void)
{
	tcntr++;
	if (tcntr >= (SAMPLE_RATE * 2))// 2 seconds of delay.
	{
		enough_delay_flag = 1;
		tcntr = 0;
	}

}

//Init_TimerA2
void Init_TimerA2(unsigned int sample_rate) {
	TA2CTL &= ~MC_1; //Turn off Timer
	TA2CCTL0 = CCIE; //CCR0 interrupt enabled
	TA2CTL = TASSEL_1 + TACLR; //ACLK, clear TAR

	if (sample_rate > 10000) {
		sample_rate = 10000;
	}
	if (sample_rate < 1) {
		sample_rate = 1;
	}

	TA2CCR0 = (unsigned int) (32767 / sample_rate - 1); // ~ 1 HZ frequency.
	TA2CTL |= MC_1; // Up mode. Count up to TA1CCR0
}

void Stop_TimerA2() {
	TA2CTL &= ~MC_1; //Turn off Timer
	TA2CCTL0 &= ~CCIE; //CCR0 interrupt enabled
}





unsigned long current_timestamp = 0;
unsigned long time_curr = 0;


void checkClicked() {
	if(button2PressedCount == 0)
		{
			Init_TimerA2((unsigned int)SAMPLE_RATE);
			tcntr=0;
			enough_delay_flag=0;
			Buttons_interruptEnable(BIT6,1);

		}


	button2PressedCount++;
}

//++checkclick functioin is only used for Software BSL
//void checkClick()
//{
//	if(button2PressedCount ==2)
//	{
//		if (enough_delay_flag == 1)
//		{
//		#ifdef SOFTWARE_BSL
//			fInvokeBSL = 1;
//		#endif
//		}
//		Stop_TimerA2();
//		button2PressedCount=0;
//		Buttons_interruptEnable(BIT6,0);
//	}
//}

void send_status_compass() {
	long data[3] = { 0 };
	int8_t accuracy = { 0 };
	unsigned long timestamp;
	inv_get_compass_set(data, &accuracy, (inv_time_t*) &timestamp);
	MPL_LOGI("Compass:\n %7.4f %7.4f %7.4f",
			data[0]/65536.f, data[1]/65536.f, data[2]/65536.f);
	MPL_LOGI("Accuracy= %d", accuracy);

}

void tellTime()
{
    unsigned long timestamp2_;
    msp430_get_clock_ms(&timestamp2_);
    MPL_LOGI("TIME = %u%", timestamp2_);

}

//show voltage of battery
void showBatteryVoltage()
{
#ifdef BATTERY_MANAGEMENT
    unsigned int value2=0;
    if(max17048_get_battery_voltage(&value2)==INV_SUCCESS){
        MPL_LOGI("BatteryVoltage = %u%", value2);
    }
//  if(max17048_get_battery_soc(&value)==INV_SUCCESS){
//      MPL_LOGI("Battery = %u%%", value);
//  }
#else
    /* In case there is no BM chip,
     * default a fake value for testing purpose
     * */
    static unsigned int value_fake=0;
    value_fake += 10; // Change this value everytime the status is called
    if(value_fake>=100){
        value_fake =0;
    }
    MPL_LOGI("Battery = %u%%", value_fake);
#endif


}

void showBatteryStatus()
{
#ifdef BATTERY_MANAGEMENT
	unsigned int value=0;

	if(max17048_get_battery_soc(&value)==INV_SUCCESS){
		MPL_LOGI("Battery = %u%%", value);
	}
#else
	/* In case there is no BM chip,
	 * default a fake value for testing purpose
	 * */
	static unsigned int value_fake=0;
	value_fake += 10; // Change this value everytime the status is called
	if(value_fake>=100){
		value_fake =0;
	}
	MPL_LOGI("Battery = %u%%", value_fake);
#endif


}


void get_eMPL_version() {
	char version[23] = { 0 };
	char **v = (char**) (&version);
	if (inv_get_version(v) == INV_SUCCESS) {
		MPL_LOGI("%s", *v);
	}
}


void load_calibration(void){
	size_t store_size;
	inv_get_mpl_state_size(&store_size);
	if (store_size > FLASH_SIZE) {
		MPL_LOGE("Calibration data exceeds available memory.\n");
	}else{
		FCTL3 = FWKEY;
		inv_load_mpl_states(FLASH_MEM_START, store_size);
		FCTL3 = FWKEY + LOCK;
		inv_accel_was_turned_off();
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();
	}
}


