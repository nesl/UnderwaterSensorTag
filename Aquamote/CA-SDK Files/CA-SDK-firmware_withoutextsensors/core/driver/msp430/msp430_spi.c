/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 * $Id: msp430_spi.c $
 *****************************************************************************/
/**
 *  @defgroup MSP430-SL
 *  @brief  MSP430 System Layer APIs.
 *          To interface with any platform, uMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file       msp430_spi.c
 *      @brief      Serial communication functions needed by uMPL to
 *                  communicate to the MPU devices.
 *      @details    This driver assumes that uMPL is running on MPS4305528
 *                  with a sub master clock set to higher frequency ~
 *                  12MHz.
 *
 * */

#include "msp430.h"
#include "msp430_spi.h"
unsigned char interrupt_detect_flag;
void msp430_init_spi_usb1(void){
	/* Will initialize spi to peripheral mode and enable SPI mode
	 * P4.0 : CS
	 * P4.1 : SIMO
	 * P4.2 : SOMI
	 * P4.3 : SCLK
	 * */
	P4SEL    |= (BIT1+BIT2+BIT3);              // Peripheral function instead of I/O
	UCB1CTL1 |= UCSWRST;                       // **Put state machine in reset**
	UCB1CTL0 |= UCMST+UCSYNC+UCCKPL+UCMSB;     // 3-pin, 8-bit SPI master
	UCB1CTL1 |= UCSSEL_3;                      // SMCLK = 12MHz speed.
	UCB1BR0 = 0x01;                            //
	UCB1BR1 = 0;                               //
	UCB1CTL1  &=~UCSWRST;                      // **Initialize USCI state machine**
	UCB1IE |= UCRXIE;                          // Enable USCI_A0 RX interrupt
}

void msp430_spi_send(unsigned char data){
	  while (!(UCB1IFG&UCTXIFG));              // USCI_A0 TX buffer ready?
	  UCB1TXBUF = data;                        // Transmit first character
}

void set_spi_rx_interrupt_flag(unsigned char value){
	interrupt_detect_flag = value;
}

unsigned char  get_spi_rx_interrupt_flag(void){
	return(interrupt_detect_flag);
}

#pragma vector=USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void)
{
  volatile unsigned int i;

  switch(__even_in_range(UCB1IV,4))
  {
    case 0: break;                          // Vector 0 - no interrupt
    case 2:                                 // Vector 2 - RXIFG
      while (!(UCB1IFG&UCTXIFG));           // USCI_B1 TX buffer ready?
     // for(i = 10; i>0; i--);                // Add time between transmissions to
                                            // make sure slave can process information
      interrupt_detect_flag = 1;            // Interrupt detected.
      break;
    case 4: break;                          // Vector 4 - TXIFG
    default: break;
  }
}




