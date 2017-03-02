/*
 $License:
 Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 * $Id: msp430_i2c.h $
 *****************************************************************************/
/**
 *  @defgroup MSP430_System_Layer MSP430 System Layer
 *  @brief  MSP430 System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file       msp430_i2c.h
 *      @brief      Serial communication functions needed by eMPL to
 *                  communicate to the MPU devices.
 *      @details    This driver assumes that eMPL is with a sub-master clock set
 *                  to 20MHz. The following MSP430s are supported:
 *
 *                  MSP430F5528
 *                  MSP430F5529
 */
#ifndef _MSP430_I2C_H_
#define _MSP430_I2C_H_

/**
 *  @brief	Set up the I2C port and configure the MSP430 as the master.
 *  @return	0 if successful.
 */
int msp430_i2c_enable(void);
/**
 *  @brief  Disable I2C communication.
 *  This function will disable the I2C hardware and should be called prior to
 *  entering low-power mode.
 *  @return 0 if successful.
 */
int msp430_i2c_disable(void);
/**
 *  @brief      Write to a device register.
 *
 *  @param[in]  slave_addr  Slave address of device.
 *  @param[in]  reg_addr	Slave register to be written to.
 *  @param[in]  length      Number of bytes to write.
 *  @param[out] data        Data to be written to register.
 *
 *  @return     0 if successful.
 */
int msp430_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
		unsigned char length, unsigned char const *data);
/**
 *  @brief      Read from a device.
 *
 *  @param[in]  slave_addr  Slave address of device.
 *  @param[in]  reg_addr	Slave register to be read from.
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Data from register.
 *
 *  @return     0 if successful.
 */
int msp430_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
		unsigned char length, unsigned char *data);

/**
 *  @brief      Read from a device.
 *
 *  @param[in]  slave_addr  Slave address of device.
 *  @param[in]  slave_reg_option	1=> Skip sendig reg value, 0=> Send reg value
 *  @param[in]  reg_addr	Slave register to be read from.
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Data from register.
 *
 *  @return     0 if successful.
 */
int msp430_i2c_read_new(unsigned char slave_addr,
		unsigned char slave_reg_option, unsigned char reg_addr,
		unsigned char length, unsigned char *data);

/**
 *  @brief      Write to a device register.
 *
 *  @param[in]  slave_addr  Slave address of device.
 *  @param[in]  slave_reg_option 1 => skip sending reg, 0=> will send reg
 *  @param[in]  reg_addr	Slave register to be written to.
 *  @param[in]  length      Number of bytes to write.
 *  @param[out] data        Data to be written to register.
 *
 *  @return     0 if successful.
 */
int msp430_i2c_write_new(unsigned char slave_addr,
		unsigned char slave_reg_option, unsigned char reg_addr,
		unsigned char length, unsigned char const *data);

int msp430_i2c_read_uv(unsigned char slave_addr,
					unsigned char slave_reg_option,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data);

void msp430_i2c_recover();
#endif  /* _MSP430_I2C_H_ */

/**
 * @}
 */
