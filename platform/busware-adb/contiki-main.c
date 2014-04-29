/*
 * This file is part of the Contiki operating system.
 *
 */

#include <string.h>


#include <avr/pgmspace.h>
#include <avr/power.h>

#include "lib/mmem.h"
#include "loader/symbols-def.h"
#include "loader/symtab.h"


#include "contiki.h"
#include "contiki-net.h"
#include "contiki-lib.h"


#include "dev/eeprom.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"

#include "dev/rs232.h"
#include "dev/serial-line.h"
#include <dev/watchdog.h>

#include "platform-conf.h"

#include "net/netstack.h"
#include "net/rime.h"

#include "node-id.h"

#define DEBUG 1

#if DEBUG
#include <stdio.h>
#define PRINTF(FORMAT,args...) printf_P(PSTR(FORMAT),##args)
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/

SENSORS(&button_sensor);
PROCINIT(&etimer_process, &sensors_process);


static void set_rime_addr(void) {
	rimeaddr_t addr;

	memset(&addr, 0, sizeof(rimeaddr_t));

	addr = get_rime_addr_from_eeprom();
	rimeaddr_set_node_addr(&addr);

#if DEBUG
	uint8_t i;
	PRINTF("Rime started with address ");
	for(i = 0; i < sizeof(addr.u8) - 1; i++) {
		PRINTF("%d.", addr.u8[i]);
	}
	PRINTF("%d\n", addr.u8[i]);
#endif
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
    set_rime_addr();
    queuebuf_init();

    netstack_init();

    PRINTF("%s %s, channel check rate %d Hz, radio channel %d\n",
	   NETSTACK_MAC.name, NETSTACK_RDC.name,
	   CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0? 1:
			   NETSTACK_RDC.channel_check_interval()),
	   RF_CHANNEL);

#ifdef NETSTACK_AES_KEY
  {
    const uint8_t key[] = NETSTACK_AES_KEY;
    netstack_aes_set_key(key);
  }
  /*printf("AES encryption is enabled: '%s'\n", NETSTACK_AES_KEY);*/
  PRINTF("AES encryption is enabled\n");
#else /* NETSTACK_AES_H */
  PRINTF("Warning: AES encryption is disabled\n");
#endif /* NETSTACK_AES_H */


/* Autostart processes */
    autostart_start(autostart_processes);

    PRINTF("\r\n********BOOTING CONTIKI*********\r\n");
    leds_off(LEDS_ALL);

    watchdog_start();
    sei();				//Enable Global Interrupt
	
    do {
	process_run();

    } while (1);
    
    return 0;
}
