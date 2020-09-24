/*
 * trace-port.h
 *
 *  Created on: 6 июл. 2020 г.
 *      Author: netuser
 */

#ifndef EXAMPLES_6TISCH_SIMPLE_NODE_TRACE_PORT_H_
// this is default header for project, prefer use board-specific one
#include_next "trace-port.h"
#ifndef EXAMPLES_6TISCH_SIMPLE_NODE_TRACE_PORT_H_
#define EXAMPLES_6TISCH_SIMPLE_NODE_TRACE_PORT_H_


#include "trace_probes.h"
//****************   probes   **************************
//*пробам назначаются пины, которыми они дергают
#define trace_idle          probe_none

#define trace_tsch_slot                     probe_dio28
#define trace_tsch_rx_poll                  probe_dio21
#define trace_tsch_tx_before_send           probe_none
#define trace_tsch_tx_send                  probe_dio22
#define trace_tsch_rx_slot                  probe_dio23
#define trace_tsch_ack_slot                 probe_dio24

#define trace_rf_read                       probe_dio25
#define trace_rf_tx                         probe_dio27
#define trace_rf_pollrecv                   probe_dio26
#define trace_rf_isrecv                     probe_dio21
#define trace_rf_isrecved                   probe_none


//* для упрощенного дергания каждому пробу создаются манипуляторы
//* манипуляторы вида trace_<name>_on/off()
trace_probe(idle);
trace_probe(tsch_slot);
trace_probe(tsch_rx_poll);
trace_probe(tsch_tx_before_send);
trace_probe(tsch_tx_send);
trace_probe(tsch_rx_slot);
trace_probe(tsch_ack_slot);

trace_probe(rf_tx);
trace_probe(rf_read);
trace_probe(rf_pollrecv);
trace_probe(rf_isrecv);
trace_probe(rf_isrecved);



#endif /* EXAMPLES_6TISCH_SIMPLE_NODE_TRACE_PORT_H_ */
#endif /* EXAMPLES_6TISCH_SIMPLE_NODE_TRACE_PORT_H_ */
