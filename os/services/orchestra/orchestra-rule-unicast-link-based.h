/*
 * orchestra-rule-unicast-link-based.h
 *
 *  Created on: 23/04/2020
 *      Author: alexrayne <alexraynepe196@gmail.com>
 */

#ifndef CONTIKI_OS_SERVICES_ORCHESTRA_ORCHESTRA_RULE_UNICAST_LINK_BASED_H_
#define CONTIKI_OS_SERVICES_ORCHESTRA_ORCHESTRA_RULE_UNICAST_LINK_BASED_H_

#include "net/mac/tsch/tsch.h"

// slotframe for orchestra rule;
extern
struct tsch_slotframe* orchestra_unb_link_based_sf;
extern
uint16_t orchestra_unb_link_based_choffs;

#ifdef ORCHESTRA_UNB_LINKBASE_ADD_UC_PAIR
void ORCHESTRA_UNB_LINKBASE_ADD_UC_PAIR(const linkaddr_t *linkaddr
                , struct tsch_link* tx_link, struct tsch_link* rx_link);
#endif


#endif /* CONTIKI_OS_SERVICES_ORCHESTRA_ORCHESTRA_RULE_UNICAST_LINK_BASED_H_ */
