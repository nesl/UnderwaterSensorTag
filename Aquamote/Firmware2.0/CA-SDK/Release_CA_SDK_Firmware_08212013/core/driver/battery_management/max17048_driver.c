/*
 * max17048_driver.c
 *
 *  Created on: Apr 19, 2013
 *      Author: Jibran Ahmed
 */

#include "max17048_driver.h"
#include "msp430_i2c.h"
#include "msp430_clock.h"

#define MAX17048_SLAVE_ADDR     0x36
#define MAX17048_VCELL          0x02
#define MAX17048_SOC            0x04
#define MAX17048_VER            0x08
#define MAX17048_HIBRT          0x0A
#define MAX17048_CONFIG         0x0C
#define MAX17048_OCV            0x0E
#define MAX17048_VLRT           0x14
#define MAX17048_VRESET         0x18
#define MAX17048_STATUS         0x1A
#define MAX17048_UNLOCK         0x3E
#define MAX17048_TABLE          0x40
#define MAX17048_RCOMPSEG1      0x80
#define MAX17048_RCOMPSEG2      0x90
#define MAX17048_CMD            0xFE
#define MAX17048_UNLOCK_VALUE   0x4a57
#define MAX17048_RESET_VALUE    0x5400
#define MAX17048_VERSION_NO     0x11

#define MAX17048_SUCCESS 0
#define MAX17048_FAILURE 1

/**
 * @Probe for MAX 17048 fuel gauge IC.
 */
int max17048_probe(void) {

	unsigned char result[2] = { 0 };
	int ret = 0;

	result[1] = MAX17048_RESET_VALUE & 0x00ff;
	result[0] = MAX17048_RESET_VALUE >> 8;

	ret = msp430_i2c_write(MAX17048_SLAVE_ADDR, MAX17048_CMD, 2, result);
	if (ret == -1) {
		return MAX17048_FAILURE;
	}

	msp430_delay_ms(50);

	result[1] = 0;
	result[0] = 0;

	ret = msp430_i2c_read(MAX17048_SLAVE_ADDR, MAX17048_VER, 2, result);
	if (ret == -1) {
		return MAX17048_FAILURE;
	}

	if (result[1] != MAX17048_VERSION_NO)
		return MAX17048_FAILURE;
	else
		return MAX17048_SUCCESS;
}

/**
 *  Get Battery Voltages from MAX17048. This function will
 *  return the voltages/cell. Battery voltages can be obtained
 *  by multiplying the result with 78.25uV
 */
int max17048_get_battery_voltage(unsigned int* d) {
	unsigned char result[2] = { 0 };
	int ret = 0;

	/**
	 * Battery voltages = vcell * 78.25uV/cell
	 */
	ret = msp430_i2c_read(MAX17048_SLAVE_ADDR, MAX17048_VCELL, 2, result);
	if (ret == -1) {
		d[0] = 0;
		return MAX17048_FAILURE;
	}else {
		d[0] = result[0];
		d[0] = (d[0] << 8) | result[1];
		return MAX17048_SUCCESS;
	}

}

/**
 * Get Battery State of Charge in %.
 * Basic implementation which does not include
 * temperature compensation, battery modeling -RCOMP, interrupt alert
 *
 * Indicates relative charge from the full charge cycle
 * for complete details please refer to the data-sheet
 */
int max17048_get_battery_soc(unsigned int* d) {
	unsigned char result[2] = { 0 };
	int ret = 0;

	ret = msp430_i2c_read(MAX17048_SLAVE_ADDR, MAX17048_SOC, 2, result);
	if (ret == -1) {
		d[0] = 0;
		return MAX17048_FAILURE;
	}else {
		d[0] = result[0];
		if(d[0]>100){
			d[0]=100;
		}
		return MAX17048_SUCCESS;
	}
}

int max17048_get_version(unsigned int* d) {
	unsigned char result[2] = { 0 };
	int ret = 0;

	ret = msp430_i2c_read(MAX17048_SLAVE_ADDR, MAX17048_VER, 2, result);

	if (ret == -1) {
		d[0] = 0;
		return MAX17048_FAILURE;
	}else {
		d[0] = result[0];
		d[0] = (d[0] << 8) | result[1];
		return MAX17048_SUCCESS;
	}
}
