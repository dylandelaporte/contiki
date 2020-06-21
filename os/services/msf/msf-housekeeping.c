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
 *         MSF Housekeeping
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#include <stdbool.h>

#include "contiki.h"
#include "lib/assert.h"

#include "net/linkaddr.h"
#include "net/mac/tsch/sixtop/sixtop.h"
#include "net/mac/tsch/sixtop/sixp-trans.h"
#include "services/shell/shell.h"

#include "msf.h"
#include "msf-autonomous-cell.h"
#include "msf-negotiated-cell.h"
#include "msf-num-cells.h"
#include "msf-avoid-cell.h"
#include "msf-sixp.h"
#include "msf-housekeeping.h"

#include "sys/log.h"
#define LOG_MODULE "MSF"
#define LOG_LEVEL LOG_LEVEL_MSF

/* variables */
static linkaddr_t parent_addr_storage;
const linkaddr_t* msf_housekeeping_parent_addr;
tsch_neighbor_t * msf_housekeeping_parent_nbr;
#define parent_addr msf_housekeeping_parent_addr

static tsch_link_t *cell_to_relocate;
static process_event_t PROCESS_EVENT_MSF_RELOCATE           = PROCESS_EVENT_POLL;
static process_event_t PROCESS_EVENT_MSF_RESOLVE            = PROCESS_EVENT_POLL;
static process_event_t PROCESS_EVENT_MSF_RESOLVE_REMOTE     = PROCESS_EVENT_POLL;


PROCESS(msf_housekeeping_process, "MSF housekeeping");

/* static functions */
static void exec_postponed_cell_deletion(tsch_slotframe_t *slotframe);

/*---------------------------------------------------------------------------*/
static void
exec_postponed_cell_deletion(tsch_slotframe_t *slotframe)
{
  if(slotframe == NULL) {
    /* do nothing */
  } else {
    tsch_link_t *cell, *next_cell;
    for(cell = list_head(slotframe->links_list);
        cell != NULL;
        cell = next_cell) {
      next_cell = list_item_next(cell);
      if(cell->link_options == LINK_OPTION_LINK_TO_DELETE) {
        LOG_DBG("delete a cell at slot_offset: %u, channel_offset: %u\n",
                cell->timeslot, cell->channel_offset);
        (void)tsch_schedule_remove_link(slotframe, cell);
      } else {
        /* need to keep this cell */
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(msf_housekeeping_process, ev, data)
{
  static struct etimer et;
  static struct timer t_col, t_gc;
  const clock_time_t slotframe_interval = (CLOCK_SECOND /
                                           TSCH_SLOTS_PER_SECOND *
                                           MSF_SLOTFRAME_LENGTH);
  PROCESS_BEGIN();
  etimer_set(&et, slotframe_interval);
  timer_set(&t_col, MSF_HOUSEKEEPING_COLLISION_PERIOD_MIN * 60 * CLOCK_SECOND);
  timer_set(&t_gc, MSF_HOUSEKEEPING_GC_PERIOD_MIN * 60 * CLOCK_SECOND);
  if (PROCESS_EVENT_MSF_RELOCATE == PROCESS_EVENT_POLL) {
      PROCESS_EVENT_MSF_RELOCATE        = process_alloc_event();
      PROCESS_EVENT_MSF_RESOLVE         = process_alloc_event();
      PROCESS_EVENT_MSF_RESOLVE_REMOTE  = process_alloc_event();
  }

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_POLL
                            || ev == PROCESS_EVENT_MSF_RELOCATE
                            || ev == PROCESS_EVENT_MSF_RESOLVE
                            || ev == PROCESS_EVENT_MSF_RESOLVE_REMOTE
                            || etimer_expired(&et)
                            );
    etimer_reset(&et);

    if(ev == PROCESS_EVENT_POLL) {
      exec_postponed_cell_deletion(msf_autonomous_cell_get_slotframe());
      exec_postponed_cell_deletion(msf_negotiated_cell_get_slotframe());
      if(msf_is_activated()) {
        continue;
      } else {
        /* stopping this process */
        break;
      }

    } else if(ev == PROCESS_EVENT_MSF_RELOCATE){
        LOG_DBG("housekeep: on relocate\n");

    } else if(ev == PROCESS_EVENT_MSF_RESOLVE_REMOTE){
        MSFCellID x;
        x.as_void = data;
        LOG_DBG("housekeep: inspects remote %u+%u:%x\n"
                , x.field.slot, x.field.chanel, x.field.link_options
                );
        msf_negotiated_inspect_vs_cellid(x);

    } else if(ev == PROCESS_EVENT_MSF_RESOLVE){
        LOG_DBG("housekeep: inspects\n");
        tsch_link_t* cell = (tsch_link_t *)data;
        bool valid_link = msf_is_negotiated_cell(cell)
                       || msf_is_autonomous_cell(cell)
                        ;
        if (valid_link){
            // is negotiation handles cell
            if ( msf_negotiated_inspect_vs_link(cell) <= irNOCELL )
            // do not check vs mine cells, only nbrs are conflicts
            if ( !linkaddr_cmp(&cell->addr, &linkaddr_node_addr) )
                msf_autonomous_inspect_vs_cell( msf_cell_of_link(cell) );
        }

    } else {
      /* etimer_expired(&et); go through */

    /* update the counters */
    msf_num_cells_update();
    }

    if(timer_expired(&t_gc)) {
      /* handle unused negotiated cells */
      msf_negotiated_cell_delete_unused_cells();
      timer_restart(&t_gc);
    }

    if(timer_expired(&t_col)) {
      /* decide to relocate a cell or not */
      if(cell_to_relocate == NULL && parent_addr) {
        cell_to_relocate = msf_negotiated_get_cell_to_relocate();
        if (cell_to_relocate == NULL)
            cell_to_relocate = msf_negotiated_propose_cell_to_relocate();
      }
      timer_restart(&t_col);
    }

    /* start an ADD or a DELETE transaction if necessary and possible */
    bool need6p= (parent_addr != NULL) && msf_sixp_is_request_wait_timer_expired();
    if (need6p)
        need6p = (sixp_trans_find_for_sfid(parent_addr, MSF_SFID) == NULL);
    if(need6p) {
      if(cell_to_relocate != NULL) {
        // check that cell still exists
        if (msf_is_negotiated_cell(cell_to_relocate))
        msf_sixp_relocate_send_request(cell_to_relocate);
        else
            msf_housekeeping_delete_cell_to_relocate();

      } else {
        msf_num_cells_trigger_6p_transaction();
      }
    } else {
      /*
       * We cannot send a request since we don't have the parent or
       * we're busy on an on-going transaction with the parent. try it
       * later.
       */
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
msf_housekeeping_start(void)
{
  parent_addr = NULL;
  msf_num_cells_reset(true);
  cell_to_relocate = NULL;
  process_start(&msf_housekeeping_process, NULL);
}
/*---------------------------------------------------------------------------*/
void
msf_housekeeping_stop(void)
{
  if(process_is_running(&msf_housekeeping_process)) {
    process_poll(&msf_housekeeping_process);
  } else {
    /* the houskeeping process is not running; do nothing */
  }
}
/*---------------------------------------------------------------------------*/
void
msf_housekeeping_set_parent_addr(const linkaddr_t *new_parent)
{
  assert(msf_is_activated() == true);

  LOG_INFO("switch parent from ");
  LOG_INFO_LLADDR(parent_addr);
  LOG_INFO_(" to ");
  LOG_INFO_LLADDR(new_parent);
  LOG_INFO_("\n");

  if(parent_addr != NULL) {
    /* CLEAR all the cells scheduled with the old parent */
    sixp_trans_t *trans = sixp_trans_find_for_sfid(parent_addr, MSF_SFID);
    if(trans != NULL) {
      /* abort the ongoing transaction */
      sixp_trans_abort(trans);
    }

    if(msf_negotiated_cell_get_num_cells(MSF_NEGOTIATED_CELL_TYPE_TX,
                                         parent_addr) > 0 ||
       msf_negotiated_cell_get_num_cells(MSF_NEGOTIATED_CELL_TYPE_RX,
                                         parent_addr) > 0) {
      msf_sixp_clear_send_request(parent_addr);
    }
  }

  /* start allocating negotiated cells with new_parent */
  /* reset the timer so as to send a request immediately */
  cell_to_relocate = NULL;
  msf_sixp_stop_request_wait_timer();
  assert(msf_sixp_is_request_wait_timer_expired());

  if(new_parent == NULL) {
    parent_addr = NULL;
    msf_num_cells_reset(true);
  } else {
    parent_addr = &parent_addr_storage;
    linkaddr_copy(&parent_addr_storage, new_parent);
    /*
     * keep the numbers of cells required and use them for the new
     * parent
     */
    msf_num_cells_reset(false);
    if(sixp_trans_find_for_sfid(parent_addr, MSF_SFID) == NULL) {
      msf_sixp_add_send_request(MSF_NEGOTIATED_CELL_TYPE_TX);
    } else {
      /* we may have a transaction with the new parent */
      msf_sixp_start_request_wait_timer();
    }
  }
}
/*---------------------------------------------------------------------------*/
void
msf_housekeeping_delete_cell_to_relocate(void)
{
  if(cell_to_relocate != NULL) {
    //this will mark taht relocate starts, so not take it for relocation
    //msf_avoid_mark_link_cell(cell_to_relocate);

    msf_negotiated_cell_delete(cell_to_relocate);
    cell_to_relocate = NULL;
  }
  // process next requested relocation
  msf_housekeeping_request_cell_to_relocate( msf_negotiated_get_cell_to_relocate() );
}

void msf_housekeeping_request_cell_to_relocate(tsch_link_t *cell){
    if (cell == NULL)
        return;
    LOG_DBG("housekeep: relocate cell %u+%u\n", cell->timeslot, cell->channel_offset);
    msf_avoid_relocate_link_cell(cell);
    if (cell_to_relocate == NULL){
        cell_to_relocate = cell;
        process_post(&msf_housekeeping_process, PROCESS_EVENT_MSF_RELOCATE, cell);
    }
}

/*---------------------------------------------------------------------------*/
/**
 * \brief ckecks that cell not conflicts with any, and rquest relocations if need
 */
void msf_housekeeping_inspect_link_consintensy(tsch_link_t *cell){
    process_post(&msf_housekeeping_process, PROCESS_EVENT_MSF_RESOLVE, cell);
}

void msf_housekeeping_inspect_cell_consintensy(
                                msf_cell_t cell,  tsch_neighbor_t *n,
                                sixp_pkt_cell_options_t cell_opts)
{
    // check auto here, since it lightweight
    if (msf_autonomous_inspect_vs_cell(cell))
        return;
    process_post(&msf_housekeeping_process, PROCESS_EVENT_MSF_RESOLVE_REMOTE
            , msf_cellid_of_sixp(cell, cell_opts).as_void
            );
}
/*---------------------------------------------------------------------------*/
void msf_housekeeping_negotiate_for_parent_rx(void){
    if (msf_num_cells_request_rx_link())
        process_poll(&msf_housekeeping_process);
}
/*---------------------------------------------------------------------------*/
void
msf_housekeeping_resolve_inconsistency(const linkaddr_t *peer_addr)
{
  sixp_trans_abort(sixp_trans_find_for_sfid(peer_addr, MSF_SFID));
  msf_sixp_clear_send_request(peer_addr);
}
/*---------------------------------------------------------------------------*/
void
msf_housekeeping_delete_cell_later(tsch_link_t *cell)
{
  if(cell == NULL) {
    /* do nothing */
  } else {
    cell->link_options = LINK_OPTION_LINK_TO_DELETE;
    process_poll(&msf_housekeeping_process);
  }
}
/*---------------------------------------------------------------------------*/
