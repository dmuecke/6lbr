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

#define RF_CHANNEL              0

#define NETSTACK_CONF_RADIO   cc1101_driver

#define NETSTACK_CONF_MAC     nullmac_driver
#define NETSTACK_CONF_RDC     nullrdc_driver
#define NETSTACK_CONF_FRAMER  framer_nullmac

//#define PACKETBUF_CONF_SIZE 30
#define QUEUEBUF_CONF_NUM 8

#define QUEUEBUF_CONF_REF_NUM 2
#define ROUTE_CONF_ENTRIES 4


/* Network setup for non-IPv6 (rime). */

#define NETSTACK_CONF_NETWORK rime_driver


#ifndef TIMESYNCH_CONF_ENABLED
#define TIMESYNCH_CONF_ENABLED           0
#endif /* TIMESYNCH_CONF_ENABLED */


#define PACKETBUF_CONF_ATTRS_INLINE 1
//#define SERIAL_LINE_CONF_BUFSIZE 32


#define WITH_ASCII 1

#define PROCESS_CONF_NUMEVENTS 8
#define PROCESS_CONF_STATS 0

#define CCIF
#define CLIF

/* The process names are not used to save RAM */
#define PROCESS_CONF_NO_PROCESS_NAMES 0

#define NETSTACK_ENCRYPTION_INIT netstack_aes_init
#define NETSTACK_ENCRYPT         netstack_aes_encrypt
#define NETSTACK_DECRYPT         netstack_aes_decrypt
#define NETSTACK_VERIFY          netstack_aes_verify

#define NETSTACK_AES_MICLEN 4

#define NETSTACK_AES_KEY {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}

typedef unsigned short clock_time_t;
typedef unsigned short uip_stats_t;
typedef unsigned long off_t;

#endif /* __CONTIKI_CONF_H__ */
