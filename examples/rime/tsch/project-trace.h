/*
 * project-trace.h
 *
 *  Created on: 24 апр. 2020 г.
 *      Author: netuser
 */

#ifndef PROJECT_TRACE_H_
#define PROJECT_TRACE_H_


// provide it to disable WD during debug
//#define CONTIKI_WATCHDOG_CONF       CONTIKI_WATCHDOG_STALL
#define CONTIKI_WATCHDOG_CONF       CONTIKI_WATCHDOG_OFF
#ifndef LPM_MODE_MAX_SUPPORTED_CONF
#define LPM_MODE_MAX_SUPPORTED_CONF LPM_MODE_SLEEP
#endif


#include "trace_probes.h"


#endif /* PROJECT_TRACE_H_ */
