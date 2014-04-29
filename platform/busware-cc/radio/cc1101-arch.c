/*
 * Copyright (c) 2012, Thingsquare, http://www.thingsquare.com/.
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
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include "contiki.h"
#include "contiki-net.h"

#include "cc1101.h"
#include "cc1101-arch.h"
#include "dev/spi.h"
#include "dev/leds.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#define SPI_PRESCALER_DIV128_gc (0x03<<0)
#define SPI_PRESCALER_DIV64_gc (0x02<<0)
#define SPI_PRESCALER_DIV16_gc (0x01<<0)

/*---------------------------------------------------------------------------*/
void
cc1101_arch_enable(void)
{
  /* Set CSn to low (0) */
  PORTB &= ~BV(CSN);

  /* The MISO pin should go high before chip is fully enabled. */
  while((PINB & BV(MISO)) != 0);
}
/*---------------------------------------------------------------------------*/
void
cc1101_arch_disable(void)
{
  /* Set CSn to high (1) */
  PORTB |= BV(CSN);
}
/*---------------------------------------------------------------------------*/
int
spi_rw_byte(unsigned char c)
{
  SPI_WAITFORTx_BEFORE();
  SPI_TXBUF = c;
  SPI_WAITFOREOTx();
  SPI_WAITFOREORx();
  c = SPI_RXBUF;

  return c;
}
/*---------------------------------------------------------------------------*/
void
cc1101_arch_init(void) 
{
	/* SPI init */
	/* Initalize ports for communication with SPI units. */
  /* CSN=SS and must be output when master! */
  DDRB  |= BV(MOSI) | BV(SCK) | BV(CSN);
  PORTB |= BV(MOSI) | BV(SCK);

  /* Enables SPI, selects "master", clock rate FCK / 2, and SPI mode 0 */
  SPCR = SPI_PRESCALER_DIV64_gc | BV(SPE) | BV(MSTR);
  SPSR = BV(SPI2X);
  
	EIMSK &= ~BV(GDOINT);
  /* Unselect radio. */
	cc1101_arch_disable();

  /*
    The following reset procedure is recommended by Section 19.1.2 in
    the CC1101 data sheet.

    * Set SCLK = 1 and SI = 0, to avoid
    potential problems with pin control mode
    (see Section 11.3).
    * Strobe CSn low / high.
    * Hold CSn low and then high for at least 40
    μs relative to pulling CSn low
    * Pull CSn low and wait for SO to go low
    (CHIP_RDYn).
    * Issue the SRES strobe on the SI line.
    * When SO goes low again, reset is
    complete and the chip is in the IDLE state.
  */

  PORTB |= BV(SCK);
  PORTB |= BV(MOSI);

  DDRB  |= BV(CSN);

  PORTB &= ~BV(CSN);
  PORTB |= BV(CSN);
  clock_delay_usec(400);
  PORTB &= ~BV(CSN);

  while((PINB & BV(MISO)) != 0);
}
/*---------------------------------------------------------------------------*/
void
cc1101_arch_interrupt_enable(void)
{
	/* Enable interrupt on the GDO2 pin */

	IN_DDR  &= ~BV(GDO2);		// Set PD2 as input (Using for interupt INT0)
	IN_PORT |= BV(GDO2);		// Enable PD2 pull-up resistor

	EICRA |=  GDOEDGE;              // Rising edge generates an interrupt
	EIMSK |=  (1<<GDOINT);
}

/*---------------------------------------------------------------------------
void
clock_delay_usec(uint16_t usec)
{
  clock_delay(usec / 100);
}
*/
/*---------------------------------------------------------------------------*/
ISR(GDOINTVECTOR)
{
	ENERGEST_ON(ENERGEST_TYPE_IRQ);

  	cc1101_rx_interrupt();

	ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
/*---------------------------------------------------------------------------*/
int
cc1101_arch_write_command(uint8_t c)
{
  return spi_rw_byte(c);
}
/*---------------------------------------------------------------------------*/
int
cc1101_arch_write_data(uint8_t d)
{
  return spi_rw_byte(d);
}
/*---------------------------------------------------------------------------*/
uint8_t
cc1101_arch_read_data(void)
{
  return spi_rw_byte(0);
}
/*---------------------------------------------------------------------------*/
int
cc1101_arch_write_databuf(const uint8_t *buf, int len)
{
  int i;
  for(i = 0; i < len; i++) {
    SPI_WRITE(buf[i]);
  }
  return len;
}
/*---------------------------------------------------------------------------*/
int
cc1101_arch_read_databuf(uint8_t *buf, int len)
{
  int i;
  for(i = 0; i < len; i++) {
    SPI_READ(buf[i]);
  }
  return len;
}
/*---------------------------------------------------------------------------*/

