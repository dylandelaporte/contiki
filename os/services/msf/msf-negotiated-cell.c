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
 *         MSF Negotiated Cell APIs
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#include "contiki.h"
#include "lib/assert.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "lib/random.h"

#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/sixtop/sixp-nbr.h"
#if ROUTING_CONF_RPL_LITE
#include "net/routing/rpl-lite/rpl-timers.h"
#endif

#include "msf.h"
#include "msf-autonomous-cell.h"
#include "msf-housekeeping.h"
#include "msf-negotiated-cell.h"
#include "msf-reserved-cell.h"
#include "msf-avoid-cell.h"
#include "msf-num-cells.h"
#include "msf-sixp-relocate.h"

#include "sys/log.h"
#define LOG_MODULE "MSF"
#define LOG_LEVEL LOG_LEVEL_MSF

// this is list elements as:
//  [tsch_link_t:]
//  [       data ]  -> [msf_negotiated_cell_data_t:]
//  [tsch_link_t:] <-  [next:                      ]
//  [       data ]  -> [msf_negotiated_cell_data_t:]
typedef struct {
  tsch_link_t *next;
  uint16_t num_tx;
  uint16_t num_tx_ack;
} msf_negotiated_cell_data_t;

/* variables */
tsch_slotframe_t * msf_negotiate_slotframe;
#define slotframe msf_negotiate_slotframe
static const bool is_used = true;
static const bool is_unused = true;
static const bool is_kept = true;
static const bool is_used_relocate = true;
static const bool is_unused_relocate = true;

MEMB(msf_negotiated_cell_data_memb,
     msf_negotiated_cell_data_t,
     MSF_MAX_NUM_NEGOTIATED_TX_CELLS);

/* static functions */
static void keep_rx_cells(const linkaddr_t *peer_addr);
static void mark_as_used(tsch_link_t *cell);
static void mark_as_unused(tsch_link_t *cell);
static void mark_as_kept(tsch_link_t *cell);
static bool is_marked_as_used(const tsch_link_t *cell);
static bool is_marked_as_unused(const tsch_link_t *cell);
static bool is_marked_as_kept(const tsch_link_t *cell);



//==============================================================================
enum {
    LINK_OPTION_XX = (LINK_OPTION_TX|LINK_OPTION_RX) ,
};

static inline
bool is_link_rx(unsigned opt){
    return (opt & LINK_OPTION_XX) == LINK_OPTION_RX;
}

static inline
bool is_link_tx(unsigned opt){
    return (opt & LINK_OPTION_XX) == LINK_OPTION_TX;
}

// just short handy acronims here
typedef msf_negotiated_cell_data_t neg_data_t;
static inline
msf_negotiated_cell_data_t * neglink_data(tsch_link_t *cell){
    return ((neg_data_t*)cell->data);
}

/*---------------------------------------------------------------------------*/
static void
keep_rx_cells(const linkaddr_t *peer_addr)
{
  if(slotframe == NULL || peer_addr == NULL) {
    /* do nothing */
  } else {
    for(tsch_link_t *cell = list_head(slotframe->links_list);
        cell != NULL;
        cell = list_item_next(cell))
    {
      if ( is_link_rx(cell->link_options) )
      if ( linkaddr_cmp(&cell->addr, peer_addr) ) {
        mark_as_kept(cell);
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
mark_rx_cell(tsch_link_t *cell, const bool *flag)
{
  if(cell == NULL) {
    /* do nothing */
  } else {
    assert( is_link_rx(cell->link_options) );
    cell->data = (void *)flag;
  }
}
/*---------------------------------------------------------------------------*/
static void
mark_as_used(tsch_link_t *cell)
{
  mark_rx_cell(cell, &is_used);
}
/*---------------------------------------------------------------------------*/
static void
mark_as_unused(tsch_link_t *cell)
{
  mark_rx_cell(cell, &is_unused);
}
/*---------------------------------------------------------------------------*/
static void
mark_as_kept(tsch_link_t *cell)
{
  mark_rx_cell(cell, &is_kept);
}
/*---------------------------------------------------------------------------*/
static bool
is_marked_as_used(const tsch_link_t *cell)
{
    if (cell != NULL){
        if (cell->data == (void *)&is_used)
            return true;
        if (cell->data == (void *)&is_used_relocate)
            return true;
    }
    return false;
}
/*---------------------------------------------------------------------------*/
static bool
is_marked_as_unused(const tsch_link_t *cell)
{
    if (cell != NULL){
        if (cell->data == (void *)&is_unused)
            return true;
        if (cell->data == (void *)&is_unused_relocate)
            return true;
    }
    return false;
}
/*---------------------------------------------------------------------------*/
static bool
is_marked_as_kept(const tsch_link_t *cell)
{
  return cell != NULL && cell->data == (void *)&is_kept;
}

bool msf_is_marked_as_relocate(const tsch_link_t *cell){
    if (cell != NULL){
        AvoidOptionsResult ops = msf_is_avoid_link_cell(cell);
        if (ops >= 0){
            return (ops & aoRELOCATE) != 0;
        }
    }
    return false;
}

tsch_link_t* msf_negotiated_get_cell_to_relocate(void){
    sixp_cell_t ops[2];
    SIXPCellsPkt* op = (SIXPCellsPkt*)ops;

    tsch_neighbor_t *n = tsch_queue_get_nbr( msf_housekeeping_get_parent_addr() );

    int ok= msf_avoid_append_nbr_cell_to_relocate(op, n);
    if ( ok <= 0)
        return NULL;

    tsch_link_t *cell;
    cell = tsch_schedule_get_link_by_timeslot(slotframe, ops[1].field.slot, ops[1].field.chanel);
    if (cell != NULL){
        LOG_DBG("found relocate cell [%u+%u]\n"
                        , cell->timeslot, cell->channel_offset);
        return cell;
    }
    //mark it to set it processed
    msf_avoid_expose_nbr_cell(ops[1], n);
    return NULL;
}

/*---------------------------------------------------------------------------*/
void
msf_negotiated_cell_activate(void)
{
  memb_init(&msf_negotiated_cell_data_memb);
  slotframe = tsch_schedule_get_slotframe_by_handle(
    MSF_SLOTFRAME_HANDLE_NEGOTIATED_CELLS);
  assert(slotframe != NULL);
}
/*---------------------------------------------------------------------------*/
void
msf_negotiated_cell_deactivate(void)
{
  msf_negotiated_cell_delete_all(NULL);
  msf_reserved_cell_delete_all(NULL);
  slotframe = NULL;
}

/*---------------------------------------------------------------------------*/
bool msf_is_negotiated_cell(tsch_link_t *cell){
    return list_contains( slotframe->links_list, cell);
}

/*---------------------------------------------------------------------------*/
const char* msf_negotiated_cell_type_str(msf_negotiated_cell_type_t type){
    if(type == MSF_NEGOTIATED_CELL_TYPE_TX)
        return "TX";
    else if (type == MSF_NEGOTIATED_CELL_TYPE_RX)
        return "RX";
    else
        return "?X";
}

int
msf_negotiated_cell_add(const linkaddr_t *peer_addr,
                        msf_negotiated_cell_type_t type,
                        uint16_t slot_offset, uint16_t channel_offset)
{
  tsch_neighbor_t *nbr;
  uint8_t cell_options;
  tsch_link_t *new_cell;

  assert(slotframe != NULL);
  assert(peer_addr != NULL);

  if(type == MSF_NEGOTIATED_CELL_TYPE_TX) {
    cell_options = LINK_OPTION_TX;
  } else {
    assert(type == MSF_NEGOTIATED_CELL_TYPE_RX);
    cell_options = LINK_OPTION_RX;
  }

  // WHAT IT? try add nbr twice?
  if((nbr = tsch_queue_add_nbr(peer_addr)) == NULL)
  if((nbr = tsch_queue_add_nbr(peer_addr)) == NULL) {
    LOG_ERR("failed to add a negotiated %s cell because nbr is not available\n",
            msf_negotiated_cell_type_str(type) );
    return irNOCELL;
  }

  new_cell = tsch_schedule_add_link(slotframe, cell_options,
                                    LINK_TYPE_NORMAL, peer_addr,
                                    slot_offset, channel_offset
                                    , 1 // WHAT to do here?
                                    );

  if (new_cell != NULL)
  if (type == MSF_NEGOTIATED_CELL_TYPE_TX) {
    new_cell->data = memb_alloc(&msf_negotiated_cell_data_memb);
    if(new_cell->data != NULL) {
        memset(new_cell->data, 0, sizeof(msf_negotiated_cell_data_t));
        LOG_DBG("add_tx: new data(%p)[%p]\n", new_cell, new_cell->data);
    } else {
      LOG_ERR("memb_alloc failed for a new negotiated cell\n");
      (void)tsch_schedule_remove_link(slotframe, new_cell);
      new_cell = NULL;
    }
  }

  if(new_cell == NULL) {
    LOG_ERR("failed to add a negotiated %s cell for ", msf_negotiated_cell_type_str(type) );
    LOG_ERR_LLADDR(peer_addr);
    LOG_ERR_(" at slot_offset:%u, channel_offset:%u\n",
             slot_offset, channel_offset);
  } else {
    LOG_INFO("added a negotiated %s cell for ", msf_negotiated_cell_type_str(type) );
    LOG_INFO_LLADDR(peer_addr);
    LOG_INFO_(" at slot_offset:%u, channel_offset:%u\n",
              slot_offset, channel_offset);
    if(type == MSF_NEGOTIATED_CELL_TYPE_RX) {
      /* the initial state is set to "used" */
      mark_as_used(new_cell);
    } else {
      /* MSF_NEGOTIATED_CELL_TYPE_TX */
      /* sort the list in reverse order of slot_offset */
      tsch_link_t *curr = nbr->negotiated_tx_cell;
      tsch_link_t *prev = NULL;
      while(curr != NULL) {
        if(new_cell->timeslot > curr->timeslot) {
          neglink_data(new_cell)->next = curr;
          if(prev == NULL) {
            /* new_cell has the largest slot_offset in the list */
            nbr->negotiated_tx_cell = new_cell;
          } else {
            neglink_data(prev)->next = new_cell;
          }
          break;
        } else {
          prev = curr;
          curr = neglink_data(curr)->next;
        }
      }
      if(curr == NULL) {
        if(nbr->negotiated_tx_cell == NULL) {
          /*
           * this is the first negotiated TX cell to the parent; now the
           * schedule is ready (msf_is_ready() returns true).
           */
          nbr->negotiated_tx_cell = new_cell;
        } else {
          /* put the new cell to the end */
          neglink_data(prev)->next = new_cell;
        }
      }
    }
    MSF_AFTER_CELL_USE(nbr, new_cell);
    msf_avoid_nbr_use_cell(msf_cell_of_link(new_cell), nbr, aoUSE_LOCAL);
    msf_num_cells_update_peers(1, type, peer_addr);
  }

  return new_cell == NULL ? -1 : 0;
}
/*---------------------------------------------------------------------------*/
static
void msf_negotiated_drop_tx_cell(tsch_neighbor_t *nbr, tsch_link_t *cell){
    if (nbr->negotiated_tx_cell == NULL)
        return;

    /* update the chain of the negotiated cells */
    tsch_link_t *curr_cell, *prev_cell;
    for(curr_cell = nbr->negotiated_tx_cell, prev_cell = NULL;
        (curr_cell != NULL) && (curr_cell->data != NULL);
        curr_cell = neglink_data(curr_cell)->next )
    {

      // we can hook in infinite cycle, if access deleted call
      assert(curr_cell != prev_cell);

      neg_data_t* cell_data = neglink_data(curr_cell);
      assert(cell_data != NULL);

      if(curr_cell == cell) {

        if(prev_cell == NULL) {
          nbr->negotiated_tx_cell = cell_data->next;
        } else {
            neglink_data(prev_cell)->next = cell_data->next;
        }
        // just for sure against lost pointer access, invalidate next
        cell_data->next = NULL; //cell;//
        memb_free(&msf_negotiated_cell_data_memb, cell_data);
        cell->data = NULL;
        return;
      } else {
        prev_cell = curr_cell;
      }
    }
}

static
void msf_negotiated_drop_all_tx(tsch_neighbor_t *nbr){
    if (nbr->negotiated_tx_cell == NULL)
        return;

    /* update the chain of the negotiated cells */
    tsch_link_t *curr_cell, *next_cell;
    for(curr_cell = nbr->negotiated_tx_cell;
        curr_cell != NULL && (curr_cell->data != NULL);
        curr_cell = next_cell)
    {
        next_cell = neglink_data(curr_cell)->next;
        neglink_data(curr_cell)->next = NULL;
        memb_free(&msf_negotiated_cell_data_memb, curr_cell->data);
        curr_cell->data = NULL;
    }

    nbr->negotiated_tx_cell = NULL;
}

// if nbr need send, provide it with autonomous cell, if no negotiated have
static
void msf_sending_nbr_ensure_tx(tsch_neighbor_t *nbr, const linkaddr_t* peer_addr){
    assert(nbr != NULL);
    //check that nbr need tx cell
    if ( !msf_negotiated_nbr_is_scheduled_tx(nbr) )
    if ( tsch_queue_nbr_packet_count(nbr) > 0){
        if ( !msf_autonomous_cell_is_scheduled_tx(peer_addr) )
            // here leave sending nbr without cells, so provide autonomous for it
            msf_autonomous_cell_add_tx(peer_addr);
    }
}

typedef enum DeleteOption{ doCAREFUL, doBAREDROP, doRESERVED } DeleteOption;
static
void msf_negotiated_cell_delete_as(tsch_link_t *cell, DeleteOption how)
{
  // this is for LOG_
  linkaddr_t peer_addr;
  (void)peer_addr;

  msf_negotiated_cell_type_t cell_type;

  uint16_t slot_offset, channel_offset;

  assert(slotframe != NULL);
  assert(cell != NULL);
  linkaddr_copy(&peer_addr, &cell->addr);

  if (cell->link_options & LINK_OPTION_RESERVED_LINK)
      how = doRESERVED;

  tsch_neighbor_t *nbr = NULL;
  if(cell->link_options & LINK_OPTION_TX) {
    cell_type = MSF_NEGOTIATED_CELL_TYPE_TX;

    if (how == doCAREFUL) {
        // manage links and  send queues
    nbr = tsch_queue_get_nbr(&cell->addr);
    assert(nbr != NULL);
    msf_negotiated_drop_tx_cell(nbr, cell);
        msf_sending_nbr_ensure_tx(nbr, &cell->addr);
    }

  } else if(cell->link_options & LINK_OPTION_RX){

    cell_type = MSF_NEGOTIATED_CELL_TYPE_RX;

  }
  else {
      cell_type = MSF_NEGOTIATED_CELL_TYPE_NO;
  }

  slot_offset = cell->timeslot;
  channel_offset = cell->channel_offset;
  msf_housekeeping_delete_cell_later(cell);
  LOG_INFO("removed a negotiated %s cell for ", msf_negotiated_cell_type_str(cell_type));
  LOG_INFO_LLADDR(&peer_addr);
  LOG_INFO_(" at slot_offset:%u, channel_offset:%u\n",
            slot_offset, channel_offset);

  // reserved links are not avoids, not active. So drop it bare and silent
  if (how >= doRESERVED)
      return;

  msf_unvoid_link_cell(cell);
  MSF_AFTER_CELL_RELEASE(nbr, cell);
  msf_num_cells_update_peers(-1, cell_type, &cell->addr);
}

void msf_negotiated_cell_delete(tsch_link_t *cell){
    msf_negotiated_cell_delete_as(cell, doCAREFUL);
}

/*---------------------------------------------------------------------------*/
void
msf_negotiated_cell_delete_all(const linkaddr_t *peer_addr)
{
  tsch_link_t *cell;
  assert(slotframe);

  {
    tsch_neighbor_t *nbr = NULL;
    tsch_link_t *next_cell;
    if(peer_addr == NULL) {
        for(cell = list_head(slotframe->links_list);
            cell != NULL;
            cell = next_cell)
        {
          next_cell = list_item_next(cell);

          if ((cell->link_options & LINK_OPTION_LINK_TO_DELETE) != 0)
              continue;

          nbr = tsch_queue_get_nbr(&cell->addr);
          assert(nbr != NULL);
          if (nbr->negotiated_tx_cell == NULL){
              // looks that nbr witout tx_links alredy managed for sending
              msf_negotiated_cell_delete_as(cell, doBAREDROP);
          }
          else {
          msf_negotiated_drop_all_tx(nbr);
              msf_negotiated_cell_delete_as(cell, doCAREFUL);
          }
        }
        LOG_INFO("removed all negotiated cells\n");
        MSF_AFTER_CELL_CLEAN(NULL);
    } else {
      nbr = tsch_queue_get_nbr(peer_addr);
      assert(nbr != NULL);
      if (nbr == NULL)
            //strange this
            return;
      msf_negotiated_drop_all_tx(nbr);

      for(cell = list_head(slotframe->links_list);
          cell != NULL;
          cell = next_cell) {
        next_cell = list_item_next(cell);
        if(linkaddr_cmp(&cell->addr, peer_addr) &&
           (cell->link_options & LINK_OPTION_LINK_TO_DELETE) == 0) {
            msf_negotiated_cell_delete_as(cell, doBAREDROP);
        } else {
          /* this cell is not scheduled with peer_addr; ignore it */
          continue;
        }
      }

      msf_sending_nbr_ensure_tx(nbr, peer_addr);
      MSF_AFTER_CELL_CLEAN(nbr);
    }
  }
}
/*---------------------------------------------------------------------------*/
bool
msf_negotiated_nbr_is_scheduled_tx(tsch_neighbor_t *nbr)
{
  assert(nbr != NULL);
  return nbr->negotiated_tx_cell != NULL;
}
/*---------------------------------------------------------------------------*/
bool msf_negotiated_is_scheduled_nbr(tsch_neighbor_t *nbr){
    if (msf_negotiated_nbr_is_scheduled_tx(nbr))
        return true;

    tsch_link_t *cell;
    for(cell = list_head(slotframe->links_list);
        cell != NULL;
        cell = list_item_next(cell))
    {
        if ((cell->link_options & (LINK_OPTION_RESERVED_LINK|LINK_OPTION_LINK_TO_DELETE))!= 0)
            continue;
        if ( nbr == tsch_queue_get_nbr(&cell->addr))
            return true;
    }
    return false;
}

/*---------------------------------------------------------------------------*/
bool msf_negotiated_is_scheduled_peer(const linkaddr_t *peer_addr){
    tsch_link_t *cell;
    for(cell = list_head(slotframe->links_list);
        cell != NULL;
        cell = list_item_next(cell))
    {
      if ((cell->link_options & (LINK_OPTION_RESERVED_LINK|LINK_OPTION_LINK_TO_DELETE))!= 0)
          continue;
      if (linkaddr_cmp(peer_addr, &cell->addr))
          return true;
    }
    return false;
}

/*---------------------------------------------------------------------------*/
tsch_link_t *
msf_negotiated_cell_get_cell_to_delete(const linkaddr_t *peer_addr,
                                       msf_negotiated_cell_type_t cell_type)
{
  tsch_link_t *ret;

  if(cell_type == MSF_NEGOTIATED_CELL_TYPE_TX) {
    tsch_neighbor_t *nbr;
    if((nbr = tsch_queue_get_nbr(peer_addr)) == NULL ||
       nbr->negotiated_tx_cell == NULL) {
      ret = NULL;
    } else {
      /* propose to delete the cell found first */
      ret = nbr->negotiated_tx_cell;
    }
  } else if(cell_type == MSF_NEGOTIATED_CELL_TYPE_RX) {
    if(slotframe == NULL) {
      ret = NULL;
    } else {
      ret = NULL;
      for(tsch_link_t *cell = list_head(slotframe->links_list);
          cell != NULL;
          cell = list_item_next(cell))
      {
        if(linkaddr_cmp(&cell->addr, peer_addr) && is_link_rx(cell->link_options) ) {
          ret = cell;
          break;
        }
      }
    }
  } else {
    /* unsupported type */
    ret = NULL;
  }
  return ret;
}
/*---------------------------------------------------------------------------*/
uint16_t
msf_negotiated_cell_get_num_cells(msf_negotiated_cell_type_t cell_type,
                                  const linkaddr_t *peer_addr)
{
  int ret;

  if(peer_addr == NULL) {
    ret = 0;
  } else {
    if(cell_type == MSF_NEGOTIATED_CELL_TYPE_TX) {
      tsch_neighbor_t *nbr;
      if((nbr = tsch_queue_get_nbr(peer_addr)) == NULL) {
        ret = 0;
      } else {
        ret = 0;
        for(tsch_link_t *cell = nbr->negotiated_tx_cell;
            cell != NULL;
            cell = neglink_data(cell)->next)
        {
          ret++;
        }
      }
    } else if(cell_type == MSF_NEGOTIATED_CELL_TYPE_RX) {
      tsch_slotframe_t *slotframe = msf_negotiated_cell_get_slotframe();
      if(slotframe == NULL) {
        ret = 0;
      } else {
        ret = 0;
        for(tsch_link_t *cell = (tsch_link_t *)list_head(slotframe->links_list);
            cell != NULL;
            cell = list_item_next(cell))
        {
          // take into account only active links
          if ((cell->link_options & (LINK_OPTION_RESERVED_LINK|LINK_OPTION_LINK_TO_DELETE))!= 0)
                continue;
          if(linkaddr_cmp(&cell->addr, peer_addr) && is_link_rx(cell->link_options)) {
            ret++;
          } else {
            /* skip TX or reserved cells */
          }
        }
      }
    } else {
      LOG_WARN("invalid cell_type: %u\n", cell_type);
      ret = 0;
    }
  }

  return ret;
}
/*---------------------------------------------------------------------------*/
void
msf_negotiated_cell_update_num_tx(uint16_t slot_offset,
                                  uint16_t num_tx, uint8_t mac_tx_status)
{
  /* update NumTx/NumTxAck of TX cells scheduled with the parent */
  const linkaddr_t *parent_addr;
  tsch_neighbor_t *nbr;

  if( (parent_addr = msf_housekeeping_get_parent_addr()) != NULL )
  if( (nbr = tsch_queue_get_nbr(parent_addr)) != NULL)
  if( nbr->negotiated_tx_cell != NULL ) {
    tsch_link_t *cell;
    tsch_link_t *last_used = NULL;
    /* identify the cell that is used for the last transmission */
    for(cell = nbr->negotiated_tx_cell;
        cell != NULL;
        cell = neglink_data(cell)->next )
    {
      if(cell->timeslot <= slot_offset) {
        last_used = cell;
        break;
      }
    }

    if(last_used == NULL) {
      if (num_tx<=1)
      /*
       * The last used cell is not scheduled as a negotiated cell for
       * the neighbor; give up updating NumTX/NumTxAck
       */
      return;

      // spread other TX attempts between other cells, since resoning link looks deleted
      --num_tx;
      last_used = nbr->negotiated_tx_cell;
    }

      cell = last_used;
      assert(cell->data != NULL);

      /* update NumTxAck */
      if(mac_tx_status == MAC_TX_OK) {
          neglink_data(cell)->num_tx_ack++;
      }

      /* update the counters.
       * Assume that every tx attempt steps through all links assigned to nbr.
       * So traverse through links sequently incrementing it's num_tx
       *
       * */

      uint16_t i;
      for(i = 0; i < num_tx; i++) {
        assert(cell->data != NULL);
        neg_data_t* cell_data = neglink_data(cell);

        if(cell_data->num_tx == 255) {
          cell_data->num_tx = 128;
          cell_data->num_tx_ack /= 2;
        } else {
          cell_data->num_tx++;
        }

        if( cell_data->next == NULL) {
          cell = nbr->negotiated_tx_cell;
        } else {
          cell = cell_data->next;
        }
      }//for(i = 0;
  }//if( nbr->negotiated_tx_cell != NULL )
}
/*---------------------------------------------------------------------------*/
tsch_link_t *
msf_negotiated_propose_cell_to_relocate(void)
{
  const linkaddr_t *parent_addr = msf_housekeeping_get_parent_addr();
  tsch_neighbor_t *nbr;
  tsch_link_t *cell;
  tsch_link_t *cell_to_relocate = NULL;
  msf_negotiated_cell_data_t *cell_data;
  tsch_link_t *worst_pdr_cell = NULL;
  int16_t best_pdr = -1; /* initialized with an invalid value for PDR */
  uint16_t worst_pdr = 100;
  int16_t pdr;

  if(parent_addr == NULL ||
     (nbr = tsch_queue_get_nbr(parent_addr)) == NULL) {
    return NULL;
  }

  for(cell = nbr->negotiated_tx_cell; cell != NULL; cell = cell_data->next) {
    cell_data = neglink_data(cell);
    assert(cell_data != NULL);
    if(cell_data->num_tx < MSF_MIN_NUM_TX_FOR_RELOCATION) {
      /* we don't evaluate this cell since it's not used much enough yet */
      pdr = -1;
    } else {
      assert(cell_data->num_tx > 0);
      pdr = cell_data->num_tx_ack * 100 / cell_data->num_tx;

      if(best_pdr < 0 || pdr > best_pdr) {
        best_pdr = pdr;
      }

      if(worst_pdr_cell == NULL || pdr < worst_pdr) {
        worst_pdr_cell = cell;
        worst_pdr = pdr;
        assert(best_pdr >= worst_pdr);
      }
    }
    LOG_DBG("cell[slot_offset: %3u, channel_offset: %3u] -- ",
            cell->timeslot, cell->channel_offset);
    LOG_DBG_("NumTx: %u, NumTxAck: %u ",
             cell_data->num_tx, cell_data->num_tx_ack);
    if(pdr < 0) {
      LOG_DBG_("PDR: N/A\n");
    } else {
      LOG_DBG_("PDR: %d\%%n", pdr);
    }
  }

  if(best_pdr < 0) {
    cell_to_relocate = NULL;
  } else {
    assert(worst_pdr >= 0);
    LOG_INFO("best PDR is %d%%, worst PDR is %u%%", best_pdr, worst_pdr);
    if((best_pdr - worst_pdr) <= MSF_RELOCATE_PDR_THRESHOLD) {
      /* worst_pdr_cell is not so bad to relocate */
      LOG_INFO_("\n");
      cell_to_relocate = NULL;
    } else {
      cell_to_relocate = worst_pdr_cell;
      LOG_INFO_("; going to relocate a TX cell"
                " [slot_offset: %u, channel_offset: %u]\n",
                cell_to_relocate->timeslot, cell_to_relocate->channel_offset);
    }
  }

  return cell_to_relocate;
}
/*---------------------------------------------------------------------------*/
uint16_t
msf_negotiated_cell_get_num_tx(tsch_link_t *cell)
{
  if(cell == NULL) return 0;
  if ( (cell->link_options & LINK_OPTION_TX) == 0 || cell->data == NULL) {
    return 0;
  } else {
    return neglink_data(cell)->num_tx;
  }
}
/*---------------------------------------------------------------------------*/
uint16_t
msf_negotiated_cell_get_num_tx_ack(tsch_link_t *cell)
{
  if(cell == NULL) return 0;
  if ( (cell->link_options & LINK_OPTION_TX) == 0 || cell->data == NULL) {
    return 0;
  } else {
    return neglink_data(cell)->num_tx_ack;
  }
}
/*---------------------------------------------------------------------------*/
void
msf_negotiated_cell_rx_mark_used(const linkaddr_t *src_addr,
                               uint16_t slot_offset)
{
  if(slotframe == NULL || src_addr == NULL || slot_offset == 0) {
    /* nothing to do */
  } else {
    for(tsch_link_t *cell = list_head(slotframe->links_list);
        cell != NULL;
        cell = list_item_next(cell))
    {
      if( is_link_rx(cell->link_options) && (cell->timeslot == slot_offset) )
      if ( linkaddr_cmp(src_addr, &cell->addr) ) {
        mark_as_used(cell);
        break;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
void
msf_negotiated_cell_delete_unused_cells(void)
{
  if(slotframe == NULL) {
      /* nothing to do; shouldn't happen, though */
      return;
  }

  LOG_DBG("unused_cells GC\n");
  /*
   * MSF assumes there is constant upward traffic, which is handled by
   * negotiated RX cells if we have descendants. Therefore, if we
   * don't use any of negotiated RX cells scheduled with a neighbor
   * for a while, we can consider either the neighbor is gone
   * somewhere or it loses the cells scheduled with us.
   *
   * In the first loop in this function, we will identify negotiated
   * RX cells we should keep. Then, in the second loop, we will delete
   * all the negotiated cells associated with MAC addresses found in
   * the rest of the negotiated RX cells (non-kept negotiated RX
   * cells).
   */
  /* This function involves many loops, which could be improved */
  const linkaddr_t *parent_addr = msf_housekeeping_get_parent_addr();

    tsch_link_t *cell, *next_cell;
    /* mark cells to keep with "is_kept" */
    for(cell = list_head(slotframe->links_list);
        cell != NULL;
        cell = list_item_next(cell))
    {
      if( is_link_rx(cell->link_options) && is_marked_as_used(cell)) {
        keep_rx_cells(&cell->addr);
      }
    }
    for(cell = list_head(slotframe->links_list), next_cell = NULL;
        cell != NULL;
        cell = next_cell)
    {
      next_cell = list_item_next(cell);

      if ( !is_link_rx(cell->link_options) )
          continue;

      // do not garbage cells that are listen parent. why?
      if (parent_addr != NULL)
      if (linkaddr_cmp(&cell->addr, parent_addr))
          continue;

        /*
         * reset the state of a cell if it has "is_kept", delete it
         * otherwise
         */
        if(is_marked_as_kept(cell)) {
          mark_as_unused(cell);
        } else if(is_marked_as_unused(cell)) {
          sixp_nbr_t *nbr = sixp_nbr_find(&cell->addr);
          if(nbr != NULL) {
            /*
             * remove "nbr", the same as resetting seqno, in order to
             * avoid schedule inconsistency in a first 6P transaction
             * with the same peer in future
             */
            sixp_nbr_free(nbr);
          }
          LOG_INFO("Detected inactivity with ");
          LOG_INFO_LLADDR(&cell->addr);
          LOG_INFO_("\n");
          msf_negotiated_cell_delete_all(&cell->addr);
        } else {
          LOG_WARN("Unexpected states found in a negotiated cell for ");
          LOG_WARN_LLADDR(&cell->addr);
          LOG_WARN_("\n");
        }
    }//for(cell
}
/*---------------------------------------------------------------------------*/
MSFInspectResult msf_negotiated_inspect_vs_link(tsch_link_t* x){
    return msf_negotiated_inspect_vs_cellid( msf_cellid_of_link(x) );
}

MSFInspectResult msf_negotiated_inspect_vs_cellid(MSFCellID x)
{
    /* mark cells to keep with "is_kept" */
    tsch_link_t *cell;
    for(cell = list_head(slotframe->links_list);
        cell != NULL;
        cell = list_item_next(cell))
    {
      if (cell->timeslot != x.field.slot) continue;

      if (cell->link_options & LINK_OPTION_LINK_TO_DELETE)
          continue;

      //inspection alredy managed;
      if (msf_is_marked_as_relocate(cell))
          return irRELOCATE;

      //if( cell->link_options == LINK_OPTION_RX && is_marked_as_used(cell))
      //local links links can mix in one slot only TX
      //bool can_overlap = true;
      bool can_overlap = msf_cellid_is_remote(x)
                      || ( is_link_tx(cell->link_options) && msf_cellid_is_tx(x) )
                      ;

      if (can_overlap)
      if (cell->channel_offset != x.field.chanel) continue;

      //try to eval that have any slot to relocate conflicting link
      long new_slot = msf_find_unused_slot_offset(slotframe);
      msf_chanel_mask_t busych =  msf_avoided_slot_chanels(new_slot);

      if (busych != 0){
          // have no unconflicting slots!
          if (!can_overlap){
              // nothing can do with it
              LOG_ERR("!new link conflicts with ->");
              LOG_ERR_LLADDR(&cell->addr);
              LOG_ERR_("for slot %d\n", cell->timeslot);
              return irFAIL;
          }
          //check that have unconflict chanels
          if (cell->channel_offset != x.field.chanel)
              return irOK;
      }
      else {
          // since able allocate unconflicting slots, drop preserved.
          //    This prevents from alloc link that later reallocate
          if (cell->link_options & LINK_OPTION_RESERVED_LINK){
              msf_reserved_release_link(cell);
              return irOK;
          }
      }

      LOG_INFO("new link relocate slot %d ->", cell->timeslot);
      LOG_INFO_LLADDR(&cell->addr);
      LOG_INFO_("\n");
      msf_housekeeping_request_cell_to_relocate(cell);
      return irRELOCATE;
    }
    return irNOCELL;
}
