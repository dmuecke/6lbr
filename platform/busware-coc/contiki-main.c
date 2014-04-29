/*
 * Copyright (c) 2006, Technical University of Munich
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
 * This file is part of the Contiki operating system.
 *
 * @(#)$$
 */

#include <string.h>
#include <stdbool.h>

#include <avr/pgmspace.h>
#include <avr/power.h>

#include "lib/mmem.h"
#include "loader/symbols-def.h"
#include "loader/symtab.h"


#include "contiki.h"
#include "contiki-net.h"
#include "contiki-lib.h"

#include "radio/cc1101.h"
#include "net/mac/frame802154.h"
#include "net/mac/framer-802154.h"

#if UIP_CONF_IPV6
#include "sicslowpan.h"
#endif

#include "dev/eeprom.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include "dev/rs232.h"

#include "dev/serial-line.h"
#include <dev/watchdog.h>

#include "platform-conf.h"

#include "net/netstack.h"
#include "net/linkaddr.h"

#include "node-id.h"

#define DEBUG DEBUG_FULL
#include "net/ip/uip-debug.h"


/*---------------------------------------------------------------------------*/

/* Put default MAC address in EEPROM */
uint8_t mac_address[8] = { 3, 0, 0, 0, 0, 0, 0xaa, 0xaa };

PROCINIT(&etimer_process);

void uip_log(char *m) {
  printf("uip: '%s'\n", m);
}

void init_lowlevel(void) {
	watchdog_init();
	clock_prescale_set(clock_div_1);

	leds_init();
	leds_on(LEDS_ALL);

	rs232_init(USART_PORT, USART_BAUD,
	             USART_PARITY_NONE | USART_STOP_BITS_1 | USART_DATA_BITS_8);

	/* Redirect stdout and stdin to USART port */
	rs232_redirect_stdout(USART_PORT);
	rs232_set_input(USART_PORT, serial_line_input_byte);

}

int main(void) {

/* Initialize hardware */
    init_lowlevel();
    clock_init();
/* Process subsystem */
    process_init();

/* Register initial processes */
    procinit_init();
    rtimer_init();
    ctimer_init();

    serial_line_init();

    queuebuf_init();

    NETSTACK_RADIO.init();
#ifdef CHANNEL_802_15_4
    cc1101_channel_set(CHANNEL_802_15_4);
#else /* CHANNEL_802_15_4 */
    cc1101_channel_set(RF_CHANNEL);
#endif /* CHANNEL_802_15_4 */

    /* Set addresses BEFORE starting tcpip process */
    linkaddr_t addr;

    memset(&addr, 0, sizeof(linkaddr_t));
    memcpy((void *)&addr.u8, &mac_address, 8);

#if UIP_CONF_IPV6
    memcpy(&uip_lladdr.addr, &addr.u8, 8);
#endif /* UIP_CONF_IPV6 */

    linkaddr_set_node_addr(&addr);
    PRINTF("MAC address %x:%x:%x:%x:%x:%x:%x:%x\n", addr.u8[0], addr.u8[1],
        addr.u8[2], addr.u8[3], addr.u8[4], addr.u8[5], addr.u8[6],
        addr.u8[7]);

    /* Initialize hardware */

    NETSTACK_RDC.init();
    NETSTACK_MAC.init();
    NETSTACK_NETWORK.init();

    ANNOTATE("Netstack: %s %s %s\n", NETSTACK_MAC.name, NETSTACK_RDC.name, NETSTACK_NETWORK.name);

#if UIP_CONF_IPV6
    process_start(&tcpip_process, NULL);
#endif /* UIP_CONF_IPV6 */

    /* Autostart processes */
    autostart_start(autostart_processes);

    ANNOTATE("\r\n********BOOTING CONTIKI*********\r\n");
    leds_off(LEDS_ALL);

    watchdog_start();
    sei();				//Enable Global Interrupt

    do {
	    process_run();

    } while (1);

    return 0;
}
