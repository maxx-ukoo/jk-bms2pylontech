
#ifndef _PORTTIMER_H
#define _PORTTIMER_H

#include "port.h"

//#include <assert.h>
//#include <inttypes.h>

extern volatile uint16_t modbus_counter;
extern uint16_t modbus_timeout;
void prvvTIMERExpiredISR( void );


#endif
