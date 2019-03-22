/*
 * msp430_spi.h
 *
 *  Created on: Feb 6, 2012
 *      Author: kkatingari
 */

#ifndef MSP430_SPI_H_
#define MSP430_SPI_H_


void msp430_init_spi_usb1(void);
void msp430_spi_send(unsigned char data);
void set_spi_rx_interrupt_flag(unsigned char value);
unsigned char  get_spi_rx_interrupt_flag(void);

#endif /* MSP430_SPI_H_ */
