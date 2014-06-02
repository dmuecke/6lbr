/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Id: platform-conf.h,v 1.1 2010/06/23 10:25:54 joxe Exp $
 */

/**
 * \file
 *         Platform configuration for Arduino
 * \author
 * 	   Ilya Dmitrichenko <errordeveloper@gmail.com>
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 */

#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__

/*
 * Definitions below are dictated by the hardware and not really
 * changeable!
 */
#define PLATFORM PLATFORM_AVR
//#define PLATFORM_HAS_LEDS    1
//#define PLATFORM_HAS_RADIO   1



/* Clock ticks per second */
#define CLOCK_CONF_SECOND 128
#define USB_SERIAL 0

/* LEDs ports. */
#define LEDS_PxDIR 			DDRA // port direction register
#define LEDS_PxOUT 			PORTA // port register

#define LEDS_CONF_RED    	(1 << PA1) //red led
#define LEDS_CONF_GREEN  	(1 << PA0) //green led


#include "dev/rs232.h"

/* USART port configuration for SLIP */

#ifndef SLIP_PORT
 #ifndef RS232_PORT_0
	#error  "RS232 header was not included ?"
 #else
  #define SLIP_PORT (RS232_PORT_0)
 #endif
#endif

#ifndef SLIP_BAUD
 #ifndef USART_BAUD_38400
	#error  "RS232 header was not included ?"
 #else
  #define SLIP_BAUD (USART_BAUD_38400)
 #endif
#endif

/* USART port configuration for serial I/O */

#ifndef USART_PORT
 #ifndef RS232_PORT_0
	#error  "RS232 header was not included ?"
 #else
  #define USART_PORT (RS232_PORT_0)
 #endif
#endif

#ifndef USART_BAUD
 #ifndef USART_BAUD_38400 
	#error  "RS232 header was not included ?"
 #else
  #define USART_BAUD (USART_BAUD_38400)
 #endif
#endif

/* Pre-allocated memory for loadable modules heap space (in bytes)*/
#define MMEM_CONF_SIZE 1024

/* Use the following address for code received via the codeprop
 * facility
 */
#define EEPROMFS_ADDR_CODEPROP 0x8000

#define EEPROM_NODE_ID_START 0x00



/*
 * SPI bus configuration.
 */
// Port B, cs PB4, mosi 5, miso 6, clk 7
/* SPI input/output registers. */
#define SPI_TXBUF SPDR
#define SPI_RXBUF SPDR

#define BV(bitno) _BV(bitno)

#define SPI_WAITFOREOTx() do { while (!(SPSR & BV(SPIF))); } while (0)
#define SPI_WAITFOREORx() do { while (!(SPSR & BV(SPIF))); } while (0)

#define CSN            4  /* Chip Select */
#define SCK            7  /* - Output: SPI Serial Clock (SCLK) - ATMEGA128 PORTB, PIN1 */
#define MOSI           5  /* - Output: SPI Master out  */
#define MISO           6  /* - Input:  SPI Master in  */

#define GDO2           2
#define GDO0           1
#define GDOINT         INT0
#define GDOEDGE        ((1<<ISC00) |(1<<ISC01))
#define GDOINTVECTOR   INT0_vect

#define IN_DDR         DDRD
#define IN_PORT        PORTD
#define IN_PIN         PIND





#endif /* __PLATFORM_CONF_H__ */
