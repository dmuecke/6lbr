/**
 * \file
 *         TI CC1101 driver for Contiki
 * \author
 *         Dirk Tostmann <tostmann@busware.de>
 */

#include "contiki.h"

#include "cc1101.h"
#include "cc1101-arch.h"

#include "util/delay.h"

#include "dev/leds.h"

#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"

#include <avr/pgmspace.h>

#include <string.h>
#define CC1101_MAX_PAYLOAD 162

#define RESET_RADIO()                   \
    do {                                \
        on();                           \
    } while(0)

#define DEBUG DEBUG_NONE
#include "uip-debug.h"

#define RSSI_OFFSET 74
#define AUX_LEN 2
#define ACK_LEN 3

#define BUSYWAIT_UNTIL(cond, max_time)                                    \
    do {                                                                  \
      rtimer_clock_t t0;                                                  \
      t0 = RTIMER_NOW();                                                  \
      while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time))) {}  \
    } while(0)


static uint8_t packet_tx[CC1101_MAX_PAYLOAD];
static uint8_t packet_rx[1 + CC1101_MAX_PAYLOAD + AUX_LEN];
static volatile uint8_t packet_rx_len = 0;
static volatile uint8_t packet_rx_idx = 0;
static volatile uint8_t is_receiving = 0;
static uint8_t fifo_status;

// CC1101 CHIP configuration - REG -> VALUE
// to be reviewed and moved platform independed (PROGMEM)...
const uint8_t PROGMEM CC1101_CFG[] = {
     0x00, 0x01, //IOCFG2: Asserts when RX FIFO is filled at or above the RX FIFO threshold or the end of packet is reached. De-asserts when the RX FIFO is empty.
     0x02, 0x09, //IOCFG0: CCA indicator
     0x03, 0x0A, //fifo threshold tx /rx
     0x04, 0x05, //SYNC1
     0x05, 0x26, //SYNC0
     0x0B, 0x06, //FSCTRL1
     0x10, 0xC8, //MDMCFG4 DRATE_E=8,
     0x11, 0x93, //MDMCFG3 DRATE_M=147, data rate = (256+DRATE_M)*2^DRATE_E/2^28*f_xosc = (9992.599) 1kbit/s (at f_xosc=26 Mhz)
     0x12, 0x03, //MDMCFG2 see above
     0x13, 0x22,  //MDMCFG1 CHANSPC_E=2, NUM_PREAMBLE=2 (4 bytes), FEC_EN = 0 (disabled)
     0x15, 0x34, //DEVIATN
     0x17, 0x3C, //MCSM1: TXOFF=IDLE, RXOFF=Stay RX, CCA_MODE=3:If RSSI below threshold unless currently receiving a packet
     0x18, 0x28, //MCSM0: PO_TIMEOUT=64, FS_AUTOCAL=2: When going from idle to RX or TX automatically
     0x19, 0x16, //FOCCFG
     0x1B, 0x43, //AGCTRL2
     0x21, 0x56, //FREND1
     0x25, 0x00, //FSCAL1
     0x26, 0x11, //FSCAL0
     0x0D, 0x21, //FREQ2
     0x0E, 0x65, //FREQ1
     0x0F, 0x6A, //FREQ0
     0x07, 0x0C, //PKTCTRL1
     0x16, 0x07, //MCSM2 RX_TIME = 7 (Timeout for sync word search in RX for both WOR mode and normal RX operation = Until end of packet) RX_TIME_QUAL=0 (check if sync word is found)
     0x20, 0xF8, //WORCTRL, WOR_RES=00 (1.8-1.9 sec) EVENT1=7 (48, i.e. 1.333 – 1.385 ms)
     0x1E, 0x87, //WOREVT1 EVENT0[high]
     0x1F, 0x6B, //WOREVT0 EVENT0[low]
     0x29, 0x59, //FSTEST
     0x2C, 0x81, //TEST2
     0x2D, 0x35, //TEST1
     0x3E, 0xC3, //?? Readonly PATABLE?
     0xff
};


PROCESS(cc1101_process, "CC1101 driver");


int on(void);
int pending_packet(void);

//-------- basic SPI communication  & helpers --------------------------

static uint8_t cc1100_readReg(uint8_t addr) {
  cc1101_arch_enable();
  spi_rw_byte( addr|CC1100_READ_BURST );
  uint8_t ret = spi_rw_byte( 0 );
  cc1101_arch_disable();
  return ret;
}

static void cc1100_writeReg(uint8_t addr, uint8_t data) {
  cc1101_arch_enable();
  spi_rw_byte( addr|CC1100_WRITE_BURST );
  spi_rw_byte( data );
  cc1101_arch_disable();
}

static uint8_t ccStrobe(uint8_t strobe) {
  cc1101_arch_enable();
  uint8_t ret = spi_rw_byte( strobe );
  cc1101_arch_disable();
  return ret;
}

static signed char rssi_dbm(unsigned char raw_rssi) {
  int16_t dbm = 0;

  if(raw_rssi >= 128) {
    dbm = (int16_t)((int16_t)(raw_rssi - 256) / 2) - RSSI_OFFSET;
  } else {
    dbm = (raw_rssi / 2) - RSSI_OFFSET;
  }
  return dbm;
}


//--------------------------------------------------------------------

/** init the radio */
int init(void) {
  uint8_t i, reg;
  cc1101_arch_init();

  ccStrobe(CC1100_SRES);
  _delay_us(200);

  ccStrobe(CC1100_SIDLE);

  // load configuration
  for (i = 0; i<200; i += 2) {
    reg = pgm_read_byte( &CC1101_CFG[i] );
    if (reg>0x40)
      break;

    cc1100_writeReg( reg, pgm_read_byte(&CC1101_CFG[i+1]) );
  }

  process_start(&cc1101_process, NULL);

  // just for fun, perform manual calibration (SRX would also do it automatically ...)
  ccStrobe( CC1100_SCAL );
  BUSYWAIT_UNTIL((cc1100_readReg( CC1100_MARCSTATE ) == MARCSTATE_IDLE), RTIMER_SECOND / 5);

  cc1101_arch_interrupt_enable();

  PRINTF("CC1101: init complete!\n");

  return 1;
};

uint8_t read_txbytes(void) {
    uint8_t txbytes1, txbytes2;
    do {
        txbytes1 = cc1100_readReg(CC1100_TXBYTES);
        txbytes2 = cc1100_readReg(CC1100_TXBYTES);
    }while(txbytes1 != txbytes2);
    return txbytes1;
}


void write_txfifo(uint8_t payload_len) {
    // clear the TX fifo
    ccStrobe(CC1100_SFTX);

    // fill FIFO
    cc1101_arch_enable();

    spi_rw_byte(CC1100_WRITE_BURST | CC1100_TXFIFO);

    // length
    spi_rw_byte(payload_len);

#define FIRST_TX 60 /* Must not be 62 or above! */
#define TXFIFO_UNDERFLOW 0x70
#ifndef MIN
#define MIN(a, b) ((a) < (b)? (a) : (b))
#endif /* MIN */
    PRINTF(".");
    uint8_t i   = 0;
    uint8_t len = MIN(payload_len, FIRST_TX);
    cc1101_arch_write_databuf(&packet_tx[i],len);

    cc1101_arch_disable();

    ccStrobe(CC1100_STX);

    if (payload_len > len ) { // fill fifo with remaining data
        PRINTF(".");
        BUSYWAIT_UNTIL(read_txbytes() < 30 , RTIMER_SECOND / 10 );

        payload_len -= len;
        i += len;

	    // fill FIFO
	    cc1101_arch_enable();
        uint8_t status = spi_rw_byte(CC1100_TXFIFO | CC1100_WRITE_BURST);
        len = status & 0x07; // available bytes in fifo
        while(payload_len > 0) {
            status = cc1101_arch_write_databuf(&packet_tx[i], len);
            payload_len -= len;
            i += len;
            if((status & 0x70) == TXFIFO_UNDERFLOW ) {
                cc1101_arch_disable();
		    /* Acknowledge it with an SFTX (else the
		       CC1101 becomes completely unresponsive) followed by an SRX,
		       and break the transmission. */
               ccStrobe(CC1100_SFTX);
               ccStrobe(CC1100_SRX);
               PRINTF("tx underflow\n");
               break;
           } else if( (status & 0x0f) < 2) {
	        /* fifo full (< 2 bytes available) */
               cc1101_arch_disable();
               BUSYWAIT_UNTIL(read_txbytes() < 30 , RTIMER_SECOND / 10 );

               cc1101_arch_enable();
               status = spi_rw_byte(CC1100_TXFIFO | CC1100_WRITE_BURST);
           }
           len = status & 0x07;

        }
        cc1101_arch_disable();
    }
    PRINTF("\n");
}

/** Prepare the radio with a packet to be sent. */
int prepare(const void *payload, unsigned short payload_len){

    if(payload_len > CC1101_MAX_PAYLOAD) {
	    PRINTF("Too big packet, aborting send %d\n", payload_len);
	    return RADIO_TX_ERR;
    }

    PRINTF("Prep %d!\n", payload_len);

    // this is temporary - chip should remain in RX for CCA checking in "transmit" later ...
    ccStrobe(CC1100_SIDLE);
    BUSYWAIT_UNTIL((cc1100_readReg( CC1100_MARCSTATE ) == MARCSTATE_IDLE), RTIMER_SECOND / 5);


    memcpy(packet_tx, payload, payload_len);

#if 0
  {
    uint8_t i;
    PRINTF("0000");       //Start a new wireshark packet
    for (i=0;i<payload_len;i++) PRINTF(" %02x",packet_tx[i]);
    PRINTF("\n");
  }
#endif

    return RADIO_TX_OK;
};


int transmit(unsigned short payload_len){
    PRINTF("T\n");


    write_txfifo(payload_len);
    BUSYWAIT_UNTIL((cc1100_readReg( CC1100_MARCSTATE )  == CC1100_STATE_TX), RTIMER_SECOND / 10);

    ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
    ENERGEST_OFF(ENERGEST_TYPE_LISTEN);

    BUSYWAIT_UNTIL((cc1100_readReg( CC1100_MARCSTATE )  != CC1100_STATE_TX), RTIMER_SECOND / 10);

    ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
    ENERGEST_ON(ENERGEST_TYPE_LISTEN);

    // we are not IDLE after timeout ...
    if (cc1100_readReg( CC1100_MARCSTATE ) != MARCSTATE_IDLE) {
	    PRINTF("f\n");

        RESET_RADIO();
	    return RADIO_TX_ERR;
    }

    ccStrobe(CC1100_SRX);
    BUSYWAIT_UNTIL((cc1100_readReg(CC1100_MARCSTATE) == MARCSTATE_RX), RTIMER_SECOND / 10);

    return RADIO_TX_OK;
};

/** Prepare & transmit a packet. */
int send_packet(const void *payload, unsigned short payload_len){
    int ret;
    PRINTF("S\n");

    ret = prepare(payload,payload_len);
    if (ret == RADIO_TX_OK)
        ret = transmit(payload_len);

    return ret;
};

/** Read a received packet into a buffer. */
int read_packet(void *buf, unsigned short buf_len){
    uint8_t len;
    signed char rssi;
    signed char lqi;

    PRINTF("R\n");
    if (!pending_packet())
	   return 0;

    rssi = rssi_dbm(packet_rx[packet_rx_len - 2]);
    lqi = (packet_rx[packet_rx_len - 1] & 0x7F);

    len = packet_rx_len - AUX_LEN;
    memcpy(buf, (void *)packet_rx, MIN(len, buf_len));

#if 0
  {
    uint8_t i;
    PRINTF("0000");       //Start a new wireshark packet
    for (i=0;i<MIN(len, buf_len);i++) PRINTF(" %02x",packet_rx[i]);
    PRINTF("\n");
  }
#endif

    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rssi);
    packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, lqi);

    RIMESTATS_ADD(llrx);
    packet_rx_len = 0;
    return MIN(len,buf_len);
};

/** Perform a Clear-Channel Assessment (CCA) to find out if there is
    a packet in the air or not. */
int channel_clear(void){
  PRINTF("c\n");
  return 0; // actually read GDO0
};



/*---------------------------------------------------------------------------*/
void cc1101_channel_set(uint8_t c) {
  cc1100_writeReg(CC1100_CHANNR, c);

}

/*---------------------------------------------------------------------------*/
uint8_t cc1101_channel_get(void) {
  return cc1100_readReg(CC1100_CHANNR);
}

/** Check if the radio driver is currently receiving a packet */
int receiving_packet(void){
  // read SFD
  PRINTF("r\n");
  return (cc1100_readReg( CC1100_PKTSTATUS ) & BV(3)) ? 1 : 0;
};

/** Check if the radio driver has just received a packet */
int pending_packet(void){
  return is_receiving == 0 && packet_rx_len > 0;
};

/** Turn the radio on. */
int on(void){
  ccStrobe( CC1100_SIDLE );
  ccStrobe( CC1100_SFRX  );
  ccStrobe( CC1100_SFTX  );
  ccStrobe( CC1100_SRX   );
  BUSYWAIT_UNTIL((cc1100_readReg( CC1100_MARCSTATE ) == MARCSTATE_RX), RTIMER_SECOND / 10);
  is_receiving = 0;
  PRINTF("rx!\n");

  return 1;
};

/** Turn the radio off. */
int off(void){
  PRINTF("CC1101 DEBUG: Off!\n");
  ccStrobe(CC1100_SIDLE);
  return 1;
};

/** Interrupt/RX handler */
int cc1101_rx_interrupt(void){
  PRINTF("I\n");

  process_post(&cc1101_process, PROCESS_EVENT_MSG, NULL);

  return 0;
};

uint8_t read_rxbytes(void) {
    uint8_t rxbytes1, rxbytes2;
    do {
        rxbytes1 = cc1100_readReg(CC1100_RXBYTES);
        rxbytes2 = cc1100_readReg(CC1100_RXBYTES);
    }while(rxbytes1 != rxbytes2);
    return rxbytes1;
}


const struct radio_driver cc1101_driver = {
  init,
  prepare,
  transmit,
  send_packet,
  read_packet,
  channel_clear,
  receiving_packet,
  pending_packet,
  on,
  off,
};



PROCESS_THREAD(cc1101_process, ev, data)
{
    static struct etimer et;
    uint8_t i,len;

    PROCESS_BEGIN();

    etimer_set(&et, 2*CLOCK_SECOND);

process:    while(1) {
        etimer_reset(&et);

	    PROCESS_WAIT_EVENT();
	    if(ev == PROCESS_EVENT_MSG) {
            do {
                fifo_status = cc1100_readReg(CC1100_MARCSTATE);
                len = read_rxbytes();

                if (fifo_status == MARCSTATE_RXFIFO_OVERFLOW || (len & 0x80)) { // fifo overflow
                    len=0;
                    PRINTF("RX OV\n");
                    RESET_RADIO();
                } else if (len > 0){
                    /* read all data in FIFO */
                    if (! is_receiving) {
                        cc1101_arch_enable();
                        spi_rw_byte(CC1100_READ_BURST | CC1100_RXFIFO);
                        packet_rx_len = spi_rw_byte(0);
                        cc1101_arch_disable();

                        packet_rx_idx = 0;
                        if (packet_rx_len > 1 + CC1101_MAX_PAYLOAD + AUX_LEN) {
                            PRINTF("packet too large: %d\n",packet_rx_len);

                            RESET_RADIO();
                            goto process;
                        }
                        len -=1;
                        is_receiving = 1;
                        etimer_reset(&et);
                    }
                    if(len < packet_rx_len + 2 - packet_rx_idx) {
                        /* keep 1 byte in rx fifo if packet bigger then fifo*/
                        len -=1;
                    }
                    if (packet_rx_idx + len > 1 + CC1101_MAX_PAYLOAD + AUX_LEN) {
                        PRINTF("packet too large: %d\n",packet_rx_idx + len);

                        RESET_RADIO();
                        goto process;

                    }
                    cc1101_arch_enable();
                    spi_rw_byte(CC1100_READ_BURST | CC1100_RXFIFO);
                    /* read more data from fifo */
                    for (i=0 ;i < len; i++) {
                        packet_rx[packet_rx_idx++] = spi_rw_byte(0);
                    }
                    cc1101_arch_disable();

                    if (packet_rx_len + 2 <= packet_rx_idx ) {
                        packet_rx_len = packet_rx_idx;
                        is_receiving = 0;
                        process_post(&cc1101_process, PROCESS_EVENT_CONTINUE, NULL);
                    // } else {
                    //     PRINTF("idx:%d\n",packet_rx_idx);
                    }
                    len = read_rxbytes();
                }
            } while(len>0);
    	} else if(ev == PROCESS_EVENT_CONTINUE) {
    	    packetbuf_clear();
    	    len = read_packet(packetbuf_dataptr(), PACKETBUF_SIZE);

    	    if(len > 0) {
        		packetbuf_set_datalen(len);
        		PRINTF("RDC input %d\n", len);
        		NETSTACK_RDC.input();
    	    }
    	} else if(etimer_expired(&et)) {
    	    is_receiving = 0;

	    // verify cc1101 is in RX state
            switch(cc1100_readReg( CC1100_MARCSTATE )) {
                case MARCSTATE_RXFIFO_OVERFLOW:
                case MARCSTATE_IDLE:
                case MARCSTATE_TXFIFO_UNDERFLOW:

                    PRINTF("Lost RX!\n");
                    RESET_RADIO();
                default:
                    break;
            }
    	}
    }

    PROCESS_END();
}
