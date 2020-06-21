/*
 * Copyright (c) 2019, Inria.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 *         MSF Autonomous Cell APIs
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#include <stddef.h>

#include "contiki.h"
#include "lib/assert.h"

#include "net/linkaddr.h"

#include "msf.h"
#include "msf-autonomous-cell.h"
#include "msf-avoid-cell.h"
#include "msf-reserved-cell.h"
#include "msf-negotiated-cell.h"
#include "msf-housekeeping.h"
#include "sax.h"

#include "sys/log.h"
#define LOG_MODULE "MSF"
#define LOG_LEVEL LOG_LEVEL_MSF

/**
 * \brief Autonomous Cell Types
 */
typedef enum {
  MSF_AUTONOMOUS_RX_CELL,
  MSF_AUTONOMOUS_TX_CELL,
} msf_autonomous_cell_type_t;

/* variables */
extern struct tsch_asn_divisor_t tsch_hopping_sequence_length;
tsch_slotframe_t *msf_autonomous_slotframe = NULL;
#define slotframe msf_autonomous_slotframe
tsch_link_t* msf_our_autonomous_rx_cell = NULL;
#define our_autonomous_rx_cell msf_our_autonomous_rx_cell

// true if other nbrs open cells at RX one
bool msf_autonomous_rx_cell_is_conflicts = false;

/* static functions */
static tsch_link_t *add_cell(msf_autonomous_cell_type_t type,
                             const linkaddr_t *mac_addr);
static void delete_cell(tsch_link_t *autonomous_cell);

/*---------------------------------------------------------------------------*/
bool msf_is_autonomous_cell(tsch_link_t *cell){
    return list_contains( slotframe->links_list, cell);
}

/*---------------------------------------------------------------------------*/
static tsch_link_t *
add_cell(msf_autonomous_cell_type_t type, const linkaddr_t *mac_addr)
{
  const char *type_str;
  (void)type_str;
  uint8_t link_options;
  uint16_t slot_offset;
  uint16_t channel_offset;
  uint16_t num_channels;
  tsch_link_t *cell;

  if(type == MSF_AUTONOMOUS_TX_CELL) {
    type_str = "TX";
    link_options = LINK_OPTION_TX | LINK_OPTION_SHARED;
  } else {
    assert(type == MSF_AUTONOMOUS_RX_CELL);
    type_str = "RX";
    link_options = LINK_OPTION_RX;
  }

  /*
   * o  slotOffset(MAC)    = 1 + hash(EUI64, length(Slotframe_1) - 1)
   * o  channelOffset(MAC) = hash(EUI64, NUM_CH_OFFSET)
   */
  assert(slotframe != NULL);
  assert(mac_addr != NULL);
  num_channels = tsch_hopping_sequence_length.val;

  slot_offset = 1 + sax(slotframe->size.val - 1,
                        mac_addr->u8, sizeof(linkaddr_t),
                        MSF_SAX_H0, MSF_SAX_L_BIT, MSF_SAX_R_BIT);
  channel_offset = sax(num_channels, mac_addr->u8, sizeof(linkaddr_t),
                       MSF_SAX_H0, MSF_SAX_L_BIT, MSF_SAX_R_BIT);

  cell = tsch_schedule_get_link_by_timeslot(slotframe, slot_offset, channel_offset);
  // overlap existing link
  bool is_conflict = (cell != NULL);

  cell = tsch_schedule_add_link(slotframe,
                                      link_options, LINK_TYPE_NORMAL, mac_addr,
                                      slot_offset, channel_offset ,
                                      0 //do not replace overlaped link, allow concurence
                                      );
  if(cell == NULL) {
    LOG_ERR("failed to add the autonomous %s cell for ", type_str);
    LOG_ERR_LLADDR(mac_addr);
    LOG_ERR_("\n");
  } else {
    LOG_DBG("added an autonomous %s cell for ", type_str);
    LOG_DBG_LLADDR(mac_addr);
    LOG_DBG_(" at slot_offset:%u, channel_offset:%u\n",
             slot_offset, channel_offset);

    //inspect link in housekeep process
    if (!is_conflict)
        msf_housekeeping_inspect_link_consintensy(cell);
    else{
        //less job on such direct conflict
        msf_autonomous_inspect_vs_cell( msf_cell_of_link(cell) );
    }

    MSF_AFTER_CELL_USE(NULL, cell);
    msf_avoid_fixed_link_cell(cell);
  }

  return cell;
}
/*---------------------------------------------------------------------------*/
static void
delete_cell(tsch_link_t *cell)
{
  const char *cell_type_str;
  (void)cell_type_str;

  assert(cell != NULL);
  if(cell->link_options & LINK_OPTION_TX) {
    cell_type_str = "TX";
  } else if(cell->link_options & LINK_OPTION_RX){
    cell_type_str = "RX";
  } else {
    assert(cell->link_options == LINK_OPTION_LINK_TO_DELETE);
    /* skip this one */
  }

  // we not unvoid cells right after use, but keep them until nbr is removed
  msf_housekeeping_delete_cell_later(cell);
  LOG_DBG("removed an autonomous %s cell for ", cell_type_str);
  LOG_DBG_LLADDR(&cell->addr);
  LOG_DBG_(" at slot_offset:%u, channel_offset:%u\n",
           cell->timeslot, cell->channel_offset);
}
/*---------------------------------------------------------------------------*/
int
msf_autonomous_cell_activate(void)
{
  slotframe = tsch_schedule_get_slotframe_by_handle(
    MSF_SLOTFRAME_HANDLE_AUTONOMOUS_CELLS);
  if(slotframe == NULL) {
    our_autonomous_rx_cell = NULL;
  } else {
    msf_autonomous_rx_cell_is_conflicts = false;
    our_autonomous_rx_cell = add_cell(MSF_AUTONOMOUS_RX_CELL,
                                      &linkaddr_node_addr);
    assert(our_autonomous_rx_cell != NULL);
    msf_avoid_link_cell_default(our_autonomous_rx_cell);

  }
  return our_autonomous_rx_cell != NULL ? 0 : -1;
}
/*---------------------------------------------------------------------------*/
void
msf_autonomous_cell_deactivate(void)
{
  msf_autonomous_cell_delete_tx(NULL);
  delete_cell(our_autonomous_rx_cell);
  our_autonomous_rx_cell = NULL;
  slotframe = NULL;
}
/*---------------------------------------------------------------------------*/
void
msf_autonomous_cell_add_tx(const linkaddr_t *peer_addr)
{
  if(peer_addr == NULL) {
    /* shouldn't happen */
    LOG_ERR("msf_autonomous_cell_add_tx: invalid peer_addr\n");
  } else {
    (void)add_cell(MSF_AUTONOMOUS_TX_CELL, peer_addr);
  }
}
/*---------------------------------------------------------------------------*/
typedef enum DeleteMode {
    dmTEMPORARY,        //< delete cell, that later willingly use for same peer
    dmFORGET            //< delete cell, with no later intentions
} DeleteMode;

static
void msf_autonomous_cell_delete_tx_how(const linkaddr_t *peer_addr, DeleteMode how)
{
  if(slotframe == NULL) {
    /* do nothing */
  } else {
    /* remove all the autonomous TX cells found in the slotframe */
    tsch_link_t *cell, *next_cell;
    for(cell = list_head(slotframe->links_list);
        cell != NULL;
        cell = next_cell) {
      next_cell = list_item_next(cell);
      if((cell->link_options & LINK_OPTION_TX) &&
         (peer_addr == NULL ||
          linkaddr_cmp(peer_addr, &cell->addr))) {
        delete_cell(cell);
        if (how == dmFORGET){
            msf_unvoid_link_cell(cell);
        }
        MSF_AFTER_CELL_RELEASE(NULL, cell);
      }
    }
  }
}

void msf_autonomous_cell_delete_tx(const linkaddr_t *peer_addr){
    msf_autonomous_cell_delete_tx_how(peer_addr, dmTEMPORARY);
}

void msf_autonomous_cell_forget(const linkaddr_t *peer_addr){
    msf_autonomous_cell_delete_tx_how(peer_addr, dmFORGET);
}

/*---------------------------------------------------------------------------*/
bool
msf_autonomous_cell_is_scheduled_tx(const linkaddr_t *peer_addr)
{
  bool ret;
  if(slotframe == NULL || peer_addr == NULL) {
    ret = false;
  } else {
    tsch_link_t *cell, *next_cell;
    ret = false;
    for(cell = list_head(slotframe->links_list);
        cell != NULL;
        cell = next_cell) {
      next_cell = list_item_next(cell);
      if((cell->link_options & LINK_OPTION_TX) &&
         linkaddr_cmp(peer_addr, &cell->addr)) {
        ret = true;
        break;
      }
    }
  }
  return ret;
}
/*---------------------------------------------------------------------------*/
int msf_autonomous_inspect_vs_cell(msf_cell_t cell){
    if (msf_autonomous_rx_cell_is_conflicts)
        return 0;

    if (our_autonomous_rx_cell != NULL)
    if (cell.raw == msf_cell_of_link(our_autonomous_rx_cell).raw ){
        msf_autonomous_rx_cell_is_conflicts = true;
        msf_housekeeping_negotiate_for_parent_rx();
        LOG_DBG("autoRX conflicts!\n");
        return 1;
    }
    return 0;
}
/*---------------------------------------------------------------------------*/
