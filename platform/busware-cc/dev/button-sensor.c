#include "lib/sensors.h"
#include "dev/button-sensor.h"

#include <avr/io.h>
#include <avr/interrupt.h>

const struct sensors_sensor button_sensor;
struct sensors_sensor *sensors[1];
unsigned char sensors_flags[1];

#define DEBOUNCE 1

#if DEBOUNCE
static struct timer debouncetimer;
#endif

#define FALSE 0
#define TRUE  1

static void init(void) {
#if DEBOUNCE
  timer_set(&debouncetimer, 0);
#endif
  
	BTN_PORT  &= ~BV(BTN);		// Set PD3 as input (Using for interupt INT1)
	BTN_PORT |= BV(BTN);		// Enable PD3 pull-up resistor
}

static void activate(void) {
	EICRA |= (1<<ISC10) |(1<<ISC11); // Rising edge generates an interrupt
	EIMSK |=  BV(INT1);
}

/*---------------------------------------------------------------------------*/
static void deactivate(void) {
	EIMSK &= ~BV(INT1);
}
/*---------------------------------------------------------------------------*/
static int active(void) {
  return (EIMSK & BV(INT1)) ? TRUE : FALSE ;
}

static int value(int type) {
#if DEBOUNCE
  return (BTN_PORT & BV(BTN)) || !timer_expired(&debouncetimer);
#else
  return BTN_PORT & BV(BTN);
#endif
}

static int configure(int type, int value) {
  switch(type){
    case SENSORS_HW_INIT:
      init();
      return 1;
    case SENSORS_ACTIVE:
      if(value)        
        activate();
      else
        deactivate();
      return 1;
  }
       
  return 0;
}

static int status(int type) {
  switch(type) {
	case SENSORS_ACTIVE:
    case SENSORS_READY:
      return active();
  }
  
  return 0;
}

ISR(INT1_vect) {
	ENERGEST_ON(ENERGEST_TYPE_IRQ);
#if DEBOUNCE
    if(timer_expired(&debouncetimer)) {
      timer_set(&debouncetimer, CLOCK_SECOND / 5);
      sensors_changed(&button_sensor);
    }
#else
    sensors_changed(&button_sensor);
#endif

	ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

SENSORS_SENSOR(button_sensor, BUTTON_SENSOR, value, configure, status);
