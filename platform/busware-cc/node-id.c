/*
 *
 * This file is part of the Contiki operating system.
 */

/**
 * \file
 *         Utility to store a node id in the external flash
 * \author
 *         busware
 */


#include "contiki-conf.h"

#include "net/linkaddr.h"
#include <avr/pgmspace.h>




#define CONTIKI_CONF_SETTINGS_MANAGER  0 //adds 1696 bytes

#if CONTIKI_CONF_SETTINGS_MANAGER
//#define PARAMETER_STORAGE 2
#define PARAMETER_STORAGE 1
#else
#define PARAMETER_STORAGE 0
#endif

#ifdef NODE_CONF_ID
#define NODE_ID NODE_CONF_ID
#else
#define NODE_ID 0x16
#endif

/* Save the default settings into program flash memory */
static const uint8_t default_mac_address[8] PROGMEM = {0xa4, 0x50, 0x55, 0xff, 0xfe, 0x14, 0x15, NODE_ID};


/*---------------------------------------------------------------------------*/
uint8_t get_eui64_from_eeprom(uint8_t macptr[8])
{
    return 0;

}

linkaddr_t get_rime_addr_from_eeprom() {
    linkaddr_t addr;
    uint8_t i;

    for(i=1; i <= LINKADDR_SIZE; i++){
	addr.u8[LINKADDR_SIZE-i] = pgm_read_byte_near(default_mac_address+8-i);
    }
    return addr;
}

/*---------------------------------------------------------------------------*/
void write_euid64_to_eprom(void)
{


}
