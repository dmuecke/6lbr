#include "contiki-conf.h"
#include "dev/leds.h"
#include <avr/io.h>


void leds_arch_init(void)
{
  LEDS_PxDIR |= (LEDS_CONF_RED | LEDS_CONF_GREEN);
  LEDS_PxOUT |= (LEDS_CONF_RED | LEDS_CONF_GREEN);
}

unsigned char leds_arch_get(void)
{
  return ((LEDS_PxOUT & LEDS_CONF_RED) ? LEDS_RED : 0)
    | ((LEDS_PxOUT & LEDS_CONF_GREEN) ? LEDS_GREEN : 0);

}

void leds_arch_set(unsigned char leds)
{
  LEDS_PxOUT = (LEDS_PxOUT & ~(LEDS_CONF_RED|LEDS_CONF_GREEN))
    | ((leds & LEDS_RED) ?  LEDS_CONF_RED: 0)
    | ((leds & LEDS_GREEN) ? LEDS_CONF_GREEN : 0);
}
