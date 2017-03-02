/*
 $License:
 Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 * $Id: sensor_probe.c $
 *****************************************************************************/
/**
 *  @defgroup sample_apps
 *  @brief  other sensor api's
 *          To interface with humidity, temperature, pressure, UV, light sensors
 *
 *  @{
 *      @file       sensor_probe.h
 *      @brief      Functions to probe and get data from the sensors listed
 *      @details    Please read the data sheet for each of these sensors.
 */

#include "sensor_probe.h"
#include "msp430_i2c.h"
#include "msp430_clock.h"

//#define TEST_PROXIMITY_SENSOR

#define SUCCESS 0
#define ERROR -1

unsigned char data[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

ic_struct ic_sht21 = { 0x40, 0, 0xE7, 1, data };
ic_struct ic_hspad32 = { 0x48, 1, 0xAC, 1, data };
ic_struct ic_cm3512 = { 0x38, 1, 0x84, 1, data };
ic_struct ic_cm36682 = { 0x60, 0, 0x0C, 1, data };


/** Probe / alive - dead test for SHT21
 * @return SUCCESS=0, ERROR=-1
 */
int probe_sht21_th_chip(void) {
	int status = SUCCESS;
	read_data(&ic_sht21);
	if (ic_sht21.data[0] != wai_sht21) {
		status = ERROR;
	}
	ic_sht21.length = 1;
	ic_sht21.reg_option = 1;
	ic_sht21.data[0] = 0xF3;
	write_data(&ic_sht21);
	return status;
}

/** Probe / alive - dead test for ALPS HSPAD32
 *  @return SUCCESS=0, ERROR=-1
 */
int probe_alpsHS_p_chip(void) {
	int status = SUCCESS;
	ic_hspad32.data[0] = 0xac;
	write_data(&ic_hspad32);
	msp430_delay_ms(100);
	ic_hspad32.length = 5;
	read_data(&ic_hspad32);
	ic_hspad32.length = 1;
	if (ic_hspad32.data[0] != wai_hspad32 && ic_hspad32.data[0] != wai_hspad32_1) {
		status = ERROR;
	}
	ic_hspad32.length = 1;
	ic_hspad32.data[0] = 0xac;
	write_data(&ic_hspad32);
	return status;
}

/** setup for UV chip instead of probe as
 *  this chip does not have any bit to show who am i
 * @return SUCCESS=0, ERROR=-1
 */
int probe_cm3512_uv_chip(void) {
	int status = SUCCESS;
	ic_cm3512.address = 0x0C;
	read_data(&ic_cm3512);
	ic_cm3512.address = 0x38;
	ic_cm3512.data[0] = 0x84;
	ic_cm3512.length = 0x01;
	write_data(&ic_cm3512);
	msp430_delay_ms(1);
	ic_cm3512.reg_option = 0x01;
	ic_cm3512.data[0] = 0xC2; // Word mode and high gain.
	write_data(&ic_cm3512);
	return status;
}

/** Probe / alive - dead test for CM36882
* @return SUCCESS=0, ERROR=-1
*/
int probe_cm3668_lp_chip(void) {
	int status = SUCCESS;
	ic_cm36682.length=2;
	read_data(&ic_cm36682);
	if(ic_cm36682.data[0]!=wai_36682){
		status = ERROR;
	}

	ic_cm36682.reg_value = 0x00;
	ic_cm36682.data[0] = 0x00;
	ic_cm36682.data[1] = 0x00;
	ic_cm36682.length = 2;
	write_data(&ic_cm36682);

#ifdef TEST_PROXIMITY_SENSOR
	// Setting up PS configuration
	// PS_CONF1
	// Duty Ratio :1/40
	// Integration Time:1.6T
	// Persistence:1
	// Proximity Sensor : ON
	// 7 : 0 = 00100010

	// PS_CONF2
	// PS_ITB : 1 * PS_IT
	// PS_INT : ON both closing and away
	// 7:0 = 01000011

	ic_cm36682.reg_value = 0x03;
	ic_cm36682.data[0] = 0x22;
	ic_cm36682.data[1] = 0x41;
	ic_cm36682.length = 2;
	write_data(&ic_cm36682);

	// PS_CONF3
	// PS_MS
	ic_cm36682.reg_value = 0x04;
	ic_cm36682.data[0] = 0x00;
	ic_cm36682.data[1] = 0x10;
	ic_cm36682.length = 2;
	write_data(&ic_cm36682);

	//PS_CANC
	ic_cm36682.reg_value = 0x05;
	ic_cm36682.data[0] = 0x00;
	ic_cm36682.data[1] = 0x00;
	ic_cm36682.length = 2;
	write_data(&ic_cm36682);

	//PS_THDH
	//PS_THDL
	ic_cm36682.reg_value = 0x06;
	ic_cm36682.data[0] = 0x01;
	ic_cm36682.data[1] = 0x16;
	ic_cm36682.length = 2;
	write_data(&ic_cm36682);
#endif
	return status;
}


/** Pressure raw data in counts as in data sheet
* @param[out] data[0]=MSB, data[1]=LSB
* Conversion method hPa =
* @return     SUCCESS=0 or ERROR=-1
*
*  For conversion in your application:
*  Pressure = pressure[0] *256 + pressure[1]
*  Pressure = (Pressure * 60 / 65535 + 50) * 10
*
*
*/
int get_pressure_data(unsigned char *pressure) {
	int status;
	ic_hspad32.length = 5;
	ic_hspad32.data[0] = 0;
	ic_hspad32.data[1] = 0;
	if (read_data(&ic_hspad32) != SUCCESS) {
		msp430_i2c_recover();
		return (-1);
	}
	if (ic_hspad32.data[0] == 0x44 || ic_hspad32.data[0] == 0x40) {
		pressure[0] = ic_hspad32.data[1];
		pressure[1] = ic_hspad32.data[2];
		ic_hspad32.data[0] = 0xac;
		ic_hspad32.length = 1;
		write_data(&ic_hspad32);
		status = SUCCESS;
	} else {
		status = ERROR;
	}
	return status;
}

/** Humidity / temperature raw data out
 * Humidity and temperature are measured alternatively
 * in this function. Enabling start while measuring the
 * value of the other sensor.
 * @param[out] data[0]=MSB, data[1]=LSB or the previous value
 * @param[out] data[0]=MSB, data[1]=LSB or the previous value
 * Conversion method   RH% =
 * Conversion method  deg-c=
 * @return     SUCCESS=0 or ERROR=-1
 *
 **  For conversion in your application:
 * 	  Temperature = temperature[0] *256 + temperature[1]
 * 	  Temperature= (float) (-46.85 + 175.72 * (Tempreture /(2.0^16.0)))
 *
 * 	  Humidity = humidity[0] *256 + humidity[1]
 *	  1- First remove the last three bits
 *	     For Android : float hex = Integer.parseInt("FFF8", 16);
						Humidity = Float.intBitsToFloat(Float.floatToRawIntBits(Humidity)& Float.floatToRawIntBits(hex));
	  2- Humidity = (float) (-6 + 125 * (Humidity /(2 ^ 16)))
 *
*/
int get_ht_data(unsigned char *humidity, unsigned char *temperature) {
	static unsigned char sensor = 0;
	int status = SUCCESS;
	ic_sht21.length = 2;
	ic_sht21.reg_option = 1;

	if (read_data(&ic_sht21) != SUCCESS) {
		msp430_i2c_recover();
		return (ERROR);
	}
	sensor ^= 0x01;
	if (sensor == 0) {
		// read temperature and set humidity start
		temperature[0] = ic_sht21.data[0];
		temperature[1] = ic_sht21.data[1];
		ic_sht21.length = 1;
		ic_sht21.data[0] = 0xF5;
		write_data(&ic_sht21);
	} else {
		// read humidity and set temperature start
		humidity[0] = ic_sht21.data[0];
		humidity[1] = ic_sht21.data[1];
		ic_sht21.length = 1;
		ic_sht21.data[0] = 0xF3;
		write_data(&ic_sht21);
	}
	return (status);
}

/** Light sensor raw data (proximity is disabled by default for
 * power concerns but can be enabled by command or #define)
 * @param[out] data[0]=MSB, data[1]=LSB or the previous value
 * Conversion method lux=
 * @return     SUCCESS=0 or ERROR=-1
 *
 * For Conversion in your application:
 * currLight = light[0] *256 + light[1]
 * currLight = currLight * 0.06103f;
 * Light = (currLight * .8f) + (Light * .2f)
 *
 *
*/
int get_light_data(unsigned char *light) {
#ifdef TEST_PROXIMITY_SENSOR
		light[0] = 0;
		light[1] = 0;
		ic_cm36682.reg_value = 0x08;
		ic_cm36682.length = 0x02;
		ic_cm36682.data[0] = 0;
		ic_cm36682.data[1] = 0;
		if (read_data(&ic_cm36682) != SUCCESS) {
			msp430_i2c_recover();
			return (-1);
		}
		light[1] = ic_cm36682.data[0];
		light[0] = ic_cm36682.data[1];
#else
		light[0] = 0;
		light[1] = 0;
		ic_cm36682.reg_value = 0x0A;
		ic_cm36682.length = 0x02;
		ic_cm36682.data[0] = 0;
		ic_cm36682.data[1] = 0;
		if (read_data(&ic_cm36682) != SUCCESS) {
			msp430_i2c_recover();
			return (-1);
		}
		light[1] = ic_cm36682.data[0];
		light[0] = ic_cm36682.data[1];
#endif
	return SUCCESS;
}


/** UV index indicator
 * @param[out] data[0]=MSB, data[1]=LSB or the previous value
 * Conversion method UVI=
 *  Warning!! For interrupt mode, UV sensor will use 0x0C and
 *  will conflict with 0x0C address of AKM in by-pass mode enabled.
 *  Put the compass in by-pass disabled mode to avoid conflict.
 * @return     SUCCESS=0 or ERROR=-1
 *
*  For conversion in your application:
 *
 * UV = uv[0] *256 + uv[1]
 * UV = (UV * 0.022f) * 10
 *
 */
int get_uv_data(unsigned char *uv) {

	uv[0] = 0;
	uv[1] = 0;

	ic_cm3512.address = 0x39;
	ic_cm3512.length = 1;
	ic_cm3512.reg_option = 1;
	ic_cm3512.data[0] = 0;
	if (read_data_uv(&ic_cm3512) != SUCCESS) {
		msp430_i2c_recover();
		return (-1);
	}
	uv[0] = ic_cm3512.data[0];
	ic_cm3512.address = 0x38;
	ic_cm3512.length = 1;
	ic_cm3512.reg_option = 1;
	ic_cm3512.data[0] = 0;
	if (read_data_uv(&ic_cm3512) != SUCCESS) {
		msp430_i2c_recover();
		return (-1);
	}
	uv[1] = ic_cm3512.data[0];
	return SUCCESS;
}

/**
 * @brief, API for I2C construct and writing data
 */
int write_data(ic_struct * ic_str) {
	int status = SUCCESS;
	status = msp430_i2c_write_new(ic_str->address, ic_str->reg_option,
			ic_str->reg_value, ic_str->length, ic_str->data);
	return (status);
}

/**
 * @brief, API for I2C construct and reading data
 */

int read_data(ic_struct *ic_str) {
	int status = SUCCESS;
	status = msp430_i2c_read_new(ic_str->address, ic_str->reg_option,
			ic_str->reg_value, ic_str->length, ic_str->data);
	return (status);
}

/**
 * @brief, UV protocol needs I2C driver to be modified for reading
 */
int read_data_uv(ic_struct *ic_str) {
	int status = SUCCESS;
	status = msp430_i2c_read_uv(ic_str->address, ic_str->reg_option,
			ic_str->reg_value, ic_str->length, ic_str->data);
	return (status);
}

