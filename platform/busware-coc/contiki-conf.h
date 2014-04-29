/*
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Kernel configuration for busware
 *
 * \author
 * 	   Dieter Muecke <dieter@busware.de>
 */

#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

#define HAVE_STDINT_H
#include "avrdef.h"

#include "platform-conf.h"

#define RIMEADDR_CONF_SIZE       8
#define PACKETBUF_CONF_HDR_SIZE    0

//define UIP_CONF_IPV6            1 //Let the makefile do this, allows hello-world to compile
#if UIP_CONF_IPV6
#define UIP_CONF_ICMP6           1
#define UIP_CONF_UDP             1
#define UIP_CONF_TCP             1
#endif

#define RF_CHANNEL              0

#define SICSLOWPAN_CONF_COMPRESSION_IPV6        0
#define SICSLOWPAN_CONF_COMPRESSION_HC1         1
#define SICSLOWPAN_CONF_COMPRESSION_HC01        2
#define SICSLOWPAN_CONF_COMPRESSION       SICSLOWPAN_COMPRESSION_HC06
#define SICSLOWPAN_CONF_CONVENTIONAL_MAC  1     //for barebones driver, sicslowpan calls radio->read function


/* No radio cycling */
#if UIP_CONF_IPV6
#define NETSTACK_CONF_NETWORK     sicslowpan_driver
#else
#define NETSTACK_CONF_NETWORK     rime_driver
#endif
#define NETSTACK_CONF_MAC         csma_driver
#define NETSTACK_CONF_RDC         nullrdc_driver        //sicslowmac_driver
#define NETSTACK_CONF_FRAMER      framer_nullmac
#define NETSTACK_CONF_RADIO       cc1101_driver
#define CHANNEL_802_15_4          26
#define NULLRDC_CONF_SEND_802154_ACK    1


//Most browsers reissue GETs after 3 seconds which stops frag reassembly, longer MAXAGE does no good
#define SICSLOWPAN_CONF_MAXAGE    3
#define QUEUEBUF_CONF_NUM         1
#define QUEUEBUF_CONF_REF_NUM     1
/* Default uip_aligned_buf and sicslowpan_aligned_buf sizes of 1280 overflows RAM */
#define UIP_CONF_BUFFER_SIZE	240

#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS 2
#define SICSLOWPAN_CONF_FRAG              1

#define UIP_CONF_LLH_LEN                0
#define UIP_CONF_LL_802154              1

#define UIP_CONF_MAX_CONNECTIONS 2
#define UIP_CONF_MAX_LISTENPORTS 2
#define UIP_CONF_UDP_CONNS       2

#define UIP_CONF_IP_FORWARD      0
#define UIP_CONF_FWCACHE_SIZE    0
#define UIP_CONF_BROADCAST       1

#define UIP_CONF_IPV6_CHECKS     1
#define UIP_CONF_IPV6_QUEUE_PKT  0
#define UIP_CONF_IPV6_REASSEMBLY 0
#define UIP_CONF_NETIF_MAX_ADDRESSES  3
#define UIP_CONF_ND6_MAX_PREFIXES     3
#define UIP_CONF_ND6_MAX_NEIGHBORS    4
#define UIP_CONF_ND6_MAX_DEFROUTERS   2
#define UIP_CONF_UDP_CHECKSUMS   1
#define UIP_CONF_TCP_SPLIT       1


#ifndef TIMESYNCH_CONF_ENABLED
#define TIMESYNCH_CONF_ENABLED           0
#endif /* TIMESYNCH_CONF_ENABLED */


#define PACKETBUF_CONF_ATTRS_INLINE 1


#define WITH_ASCII 1

#define PROCESS_CONF_NUMEVENTS 8
#define PROCESS_CONF_STATS 0

#define CCIF
#define CLIF

/* The process names are not used to save RAM */
#define PROCESS_CONF_NO_PROCESS_NAMES 0

//#define CC_CONF_REGISTER_ARGS		1
//#define CC_CONF_FUNCTION_POINTER_ARGS	1
//#define CC_CONF_INLINE			inline
//#define CC_CONF_VA_ARGS			1



/* Skip the last four bytes of the EEPROM, to leave room for things
 * like the avrdude erase count and bootloader signaling. */



typedef unsigned short clock_time_t;
typedef unsigned short uip_stats_t;
typedef unsigned long off_t;

#ifdef PROJECT_CONF_H
#include PROJECT_CONF_H
#endif /* PROJECT_CONF_H */

#endif /* __CONTIKI_CONF_H__ */
