/*
 * Copyright (c) 2014, SICS Swedish ICT.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         IEEE 802.15.4 TSCH MAC schedule manager.
 * \author
 *         Simon Duquennoy <simonduq@sics.se>
 *         Beshr Al Nahas <beshr@sics.se>
 *         Atis Elsts <atis.elsts@edi.lv>
 */

/**
 * \addtogroup tsch
 * @{
*/

#include "contiki.h"
#include "dev/leds.h"
#include "lib/memb.h"
#include "net/nbr-table.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/framer/frame802154.h"
#include "sys/process.h"
#include "sys/rtimer.h"
#include <string.h>
#include <assert.h>

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "TSCH Sched"
#define LOG_LEVEL LOG_LEVEL_MAC

// turn on TSCH_PRINT if have some LOG_LEVEL_MAC
#if LOG_CONF_LEVEL_MAC > LOG_LEVEL_NONE
#define DEBUG   (DEBUG_PRINT)
#else
#define DEBUG   0
#endif
#include "net/net-debug.h"



/* Pre-allocated space for links */
MEMB(link_memb, struct tsch_link, TSCH_SCHEDULE_MAX_LINKS);
/* Pre-allocated space for slotframes */
MEMB(slotframe_memb, struct tsch_slotframe, TSCH_SCHEDULE_MAX_SLOTFRAMES);
/* List of slotframes (each slotframe holds its own list of links) */
LIST(slotframe_list);

/* Adds and returns a slotframe (NULL if failure) */
struct tsch_slotframe *
tsch_schedule_add_slotframe(tsch_sf_h handle, uint16_t size)
{
  if(size == 0) {
    return NULL;
  }

  if(tsch_schedule_get_slotframe_by_handle(handle)) {
    /* A slotframe with this handle already exists */
    return NULL;
  }

  if(tsch_get_lock()) {
    struct tsch_slotframe *sf = memb_alloc(&slotframe_memb);
    if(sf != NULL) {
      /* Initialize the slotframe */
      sf->handle = handle;
      TSCH_ASN_DIVISOR_INIT(sf->size, size);
      LIST_STRUCT_INIT(sf, links_list);
      /* Add the slotframe to the global list */
      list_add(slotframe_list, sf);
    }
    LOG_INFO("TSCH-schedule: add_slotframe %u %u\n", handle, size);
    tsch_release_lock();
    return sf;
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
/* Removes all slotframes, resulting in an empty schedule */
int
tsch_schedule_remove_all_slotframes(void)
{
  struct tsch_slotframe *sf;
  while((sf = list_head(slotframe_list))) {
    if(tsch_schedule_remove_slotframe(sf) == 0) {
      return 0;
    }
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
/* Removes a slotframe Return 1 if success, 0 if failure */
int
tsch_schedule_remove_slotframe(struct tsch_slotframe *slotframe)
{
  if(slotframe != NULL) {
    /* Remove all links belonging to this slotframe */
    struct tsch_link *l;
    while((l = list_head(slotframe->links_list))) {
      tsch_schedule_remove_link(slotframe, l);
    }

    /* Now that the slotframe has no links, remove it. */
    if(tsch_get_lock()) {
      LOG_INFO("remove slotframe %u %u\n", slotframe->handle, slotframe->size.val);
      memb_free(&slotframe_memb, slotframe);
      list_remove(slotframe_list, slotframe);
      tsch_release_lock();
      return 1;
    }
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Looks for a slotframe from a handle */
struct tsch_slotframe *
tsch_schedule_get_slotframe_by_handle(tsch_sf_h handle)
{
  if(!tsch_is_locked()) {
    struct tsch_slotframe *sf = list_head(slotframe_list);
    while(sf != NULL) {
      if(sf->handle == handle) {
        return sf;
      }
      sf = list_item_next(sf);
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
/* Looks for a link from a handle */
struct tsch_link *
tsch_schedule_get_link_by_handle(uint16_t handle)
{
  if(!tsch_is_locked()) {
    struct tsch_slotframe *sf = list_head(slotframe_list);
    while(sf != NULL) {
      struct tsch_link *l = list_head(sf->links_list);
      /* Loop over all items. Assume there is max one link per timeslot */
      while(l != NULL) {
        if(l->handle == handle) {
          return l;
        }
        l = list_item_next(l);
      }
      sf = list_item_next(sf);
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
static const char *
print_link_options(uint16_t link_options)
{
  static char buffer[20];
  unsigned length;

  buffer[0] = '\0';
  if(link_options & LINK_OPTION_TX) {
    strcat(buffer, "Tx|");
  }
  if(link_options & LINK_OPTION_RX) {
    strcat(buffer, "Rx|");
  }
  if(link_options & LINK_OPTION_SHARED) {
    strcat(buffer, "Sh|");
  }
  length = strlen(buffer);
  if(length > 0) {
    buffer[length - 1] = '\0';
  }

  return buffer;
}
/*---------------------------------------------------------------------------*/
static const char *
print_link_type(uint16_t link_type)
{
  switch(link_type) {
  case LINK_TYPE_NORMAL:
    return "NORMAL";
  case LINK_TYPE_ADVERTISING:
    return "ADV";
  case LINK_TYPE_ADVERTISING_ONLY:
    return "ADV_ONLY";
  default:
    return "?";
  }
}
/*---------------------------------------------------------------------------*/
void tsch_schedule_link_addr_aqure(struct tsch_link *l);
void tsch_schedule_link_addr_release(uint8_t link_options, const linkaddr_t* addr);

/* Adds a link to a slotframe, return a pointer to it (NULL if failure) */
struct tsch_link *
tsch_schedule_add_link(struct tsch_slotframe *slotframe,
                       uint8_t link_options, enum link_type link_type, const linkaddr_t *address,
                       uint16_t timeslot, uint16_t channel_offset, uint8_t do_remove)
{
  struct tsch_link *l = NULL;
  if(slotframe != NULL) {
    /* We currently support only one link per timeslot in a given slotframe. */

    /* Validation of specified timeslot and channel_offset */
    if(timeslot > (slotframe->size.val - 1)) {
      LOG_ERR("! add_link invalid timeslot: %u\n", timeslot);
      return NULL;
    }

    if(do_remove) {
      /* Start with removing the link currently installed at this timeslot (needed
       * to keep neighbor state in sync with link options etc.) */
      tsch_schedule_remove_link_by_timeslot(slotframe, timeslot, channel_offset);
    }
    if(!tsch_get_lock()) {
      LOG_ERR("! add_link memb_alloc couldn't take lock\n");
    } else {
      l = memb_alloc(&link_memb);
      if(l == NULL) {
        //TSCH_PUTS("TSCH-schedule:! add_link memb_alloc failed\n");
        LOG_ERR("! add_link memb_alloc failed\n");
        tsch_release_lock();
      } else {
        static int current_link_handle = 0;
        /* Add the link to the slotframe */
        list_add(slotframe->links_list, l);
        /* Initialize link */
        l->handle = current_link_handle++;
        l->link_options = link_options;
        l->link_type = link_type;
        l->slotframe_handle = slotframe->handle;
        l->timeslot = timeslot;
        l->channel_offset = channel_offset;
        l->data = NULL;
        if(address == NULL) {
          address = &linkaddr_null;
        }
        linkaddr_copy(&l->addr, address);

        /* Release the lock before we update the neighbor (will take the lock) */
        tsch_release_lock();
        tsch_schedule_link_addr_aqure(l);

        TSCH_PRINTF8("TSCH-schedule: add_link sf%u %x/%u [%u+%u] %x\n",
               slotframe->handle, link_options, link_type
               , timeslot, channel_offset
               , TSCH_LOG_ID_FROM_LINKADDR(address));

        LOG_INFO("add_link sf=%u opt=%s type=%s [%u+%u] addr=",
                 slotframe->handle,
                 print_link_options(link_options),
                 print_link_type(link_type), timeslot, channel_offset);
        LOG_INFO_LLADDR(address);
        LOG_INFO_("\n");

      }
    }
  }
  return l;
}

/*---------------------------------------------------------------------------*/
/* Changes adress on a link*/
void tsch_schedule_link_change_addr(struct tsch_link *l, const linkaddr_t *address)
{
    if(address == NULL) {
      address = &linkaddr_null;
    }
    if (linkaddr_cmp(address, &l->addr))
        return;
    linkaddr_copy(&l->addr, address);
    tsch_schedule_link_addr_release(l->link_options, &l->addr);
    tsch_schedule_link_addr_aqure(l);
}

/*---------------------------------------------------------------------------*/
/* Changes adress on a link*/
void tsch_schedule_link_change_option(struct tsch_link *l, uint8_t link_options)
{
    if (l->link_options == link_options)
        return;
    tsch_schedule_link_addr_release(l->link_options, &l->addr);
    l->link_options = link_options;
    tsch_schedule_link_addr_aqure(l);
}

/*---------------------------------------------------------------------------*/
/* Removes a link from slotframe. Return 1 if success, 0 if failure */
int
tsch_schedule_remove_link(struct tsch_slotframe *slotframe, struct tsch_link *l)
{
  if(slotframe != NULL && l != NULL && l->slotframe_handle == slotframe->handle) {
    if(tsch_get_lock()) {
      uint8_t link_options;
      linkaddr_t addr;

      /* Save link option and addr in local variables as we need them
       * after freeing the link */
      link_options = l->link_options;
      linkaddr_copy(&addr, &l->addr);

      /* The link to be removed is scheduled as next, set it to NULL
       * to abort the next link operation */
      if(l == current_link) {
        current_link = NULL;
      }
      TSCH_PRINTF8("TSCH-schedule: remove_link %u %u %u %u %x\n",
             slotframe->handle, l->link_options, l->timeslot, l->channel_offset,
             TSCH_LOG_ID_FROM_LINKADDR(&l->addr));

      list_remove(slotframe->links_list, l);
      memb_free(&link_memb, l);

      /* Release the lock before we update the neighbor (will take the lock) */
      tsch_release_lock();

      tsch_schedule_link_addr_release(link_options, &addr);

      return 1;
    } else {
        LOG_ERR("! remove_link memb_alloc couldn't take lock\n");
    }
  }
  return 0;
}

/*---------------------------------------------------------------------------*/
void tsch_schedule_link_addr_aqure(struct tsch_link *l){
    struct tsch_neighbor *n;
    if(l->link_options & LINK_OPTION_TX) {
      n = tsch_queue_add_nbr(&l->addr);
      /* We have a tx link to this neighbor, update counters */
      if(n != NULL) {
        n->tx_links_count++;
        if(!(l->link_options & LINK_OPTION_SHARED)) {
          n->dedicated_tx_links_count++;
        }
      }
    }
}

void tsch_schedule_link_addr_release(uint8_t link_options, const linkaddr_t* addr){
    /* This was a tx link to this neighbor, update counters */
    if(link_options & LINK_OPTION_TX) {
      struct tsch_neighbor *n = tsch_queue_add_nbr(addr);
      if(n != NULL) {
        n->tx_links_count--;
        if(!(link_options & LINK_OPTION_SHARED)) {
          n->dedicated_tx_links_count--;
        }
      }
    }
}


/*---------------------------------------------------------------------------*/
/* Removes a link from slotframe and timeslot. Return a 1 if success, 0 if failure */
int
tsch_schedule_remove_link_by_timeslot(struct tsch_slotframe *slotframe,
                                    tsch_slot_offset_t timeslot, uint16_t channel_offset)
{
  int ret = 0;
  if(!tsch_is_locked()) {
    if(slotframe != NULL) {
      struct tsch_link *l = list_head(slotframe->links_list);
      /* Loop over all items and remove all matching links */
      while(l != NULL) {
        struct tsch_link *next = list_item_next(l);
        if(l->timeslot == timeslot && l->channel_offset == channel_offset) {
          if(tsch_schedule_remove_link(slotframe, l)) {
            ret = 1;
          }
        }
        l = next;
      }
    }
  }
  return ret;
}
/*---------------------------------------------------------------------------*/
/* Looks within a slotframe for a link with a given timeslot */
struct tsch_link *
tsch_schedule_get_link_by_timeslot(struct tsch_slotframe *slotframe,
                                    tsch_slot_offset_t timeslot, uint16_t channel_offset)
{
  if(!tsch_is_locked()) {
    if(slotframe != NULL) {
      struct tsch_link *l = list_head(slotframe->links_list);
      /* Loop over all items. Assume there is max one link per timeslot and channel_offset */
      while(l != NULL) {
        if(l->timeslot == timeslot && l->channel_offset == channel_offset) {
          return l;
        }
        l = list_item_next(l);
      }
      return l;
    }
  }
  return NULL;
}

tsch_link_t *tsch_schedule_get_any_link_by_timeslot(tsch_slotframe_t *slotframe,
                                        tsch_slot_offset_t timeslot)
{
  if(!tsch_is_locked()) {
    if(slotframe != NULL) {
      struct tsch_link *l = list_head(slotframe->links_list);
      /* Loop over all items. Assume there is max one link per timeslot and channel_offset */
      while(l != NULL) {
        if(l->timeslot == timeslot) {
          return l;
        }
        l = list_item_next(l);
      }
      return l;
    }
  }
  return NULL;
}


/*---------------------------------------------------------------------------*/
static struct tsch_link *
default_tsch_link_comparator(struct tsch_link *a, struct tsch_link *b)
{
  if(!(a->link_options & LINK_OPTION_TX)) {
    /* None of the links are Tx: simply return the first link */
    return a;
  }

  /* Two Tx links at the same slotframe; return the one with most packets to send */
  if(!linkaddr_cmp(&a->addr, &b->addr)) {
    struct tsch_neighbor *an = tsch_queue_get_nbr(&a->addr);
    struct tsch_neighbor *bn = tsch_queue_get_nbr(&b->addr);
    int a_packet_count = an ? ringbufindex_elements(&an->tx_ringbuf) : 0;
    int b_packet_count = bn ? ringbufindex_elements(&bn->tx_ringbuf) : 0;
    /* Compare the number of packets in the queue */
    return a_packet_count >= b_packet_count ? a : b;
  }

  /* Same neighbor address; simply return the first link */
  return a;
}

/*---------------------------------------------------------------------------*/
/* Returns the next active link after a given ASN, and a backup link (for the same ASN, with Rx flag) */
//  \arg time_offset - gives TSCH_DESYNC_THRESHOLD_SLOTS value, used to escape
//                  timesource EB
// used by tsch_slot_operation
struct tsch_link *signaling_link = NULL;

struct tsch_link *
tsch_schedule_get_next_active_link(struct tsch_asn_t *asn
    , tsch_slot_offset_t *time_offset,
    struct tsch_link **backup_link)
{
  tsch_slot_offset_t time_to_curr_best = 0;
  struct tsch_slotframe * best_frame = NULL;
    // signaling link has seen at time_to_curr_best
    signaling_link                = NULL;

  struct tsch_link *curr_best = NULL;
  struct tsch_link *curr_backup = NULL; /* Keep a back link in case the current link
  turns out useless when the time comes. For instance, for a Tx-only link, if there is
  no outgoing packet in queue. In that case, run the backup link instead. The backup link
  must have Rx flag set. */
  if(!tsch_is_locked()) {
    struct tsch_slotframe *sf = list_head(slotframe_list);
    /* For each slotframe, look for the earliest occurring link */
    for(;sf != NULL; sf = list_item_next(sf)) {
      if (sf->size.val == 0)
          continue;
      /* Get timeslot from ASN, given the slotframe length */
      tsch_slot_offset_t timeslot = TSCH_ASN_MOD(*asn, sf->size);
      struct tsch_link *l = list_head(sf->links_list);
      for(; l != NULL; l = list_item_next(l)) {
          tsch_slot_offset_t linktime = l->timeslot;

          // plan point is not avoidable
          if ((l->link_options & (LINK_OPTION_DISABLE)) != 0)
              continue;
          if (TSCH_SCHEDULE_POLICY & TSCH_SCHEDULE_OMMIT_NOXFER){
              if ((l->link_options & (LINK_OPTION_RX|LINK_OPTION_TX|LINK_OPTION_PLANPOINT)) == 0)
              // when link ton transfers, skip it
                continue;
              if ((l->link_options & LINK_OPTION_TIME_EB_ESCAPE) != 0){
                  // TODO use cource calculation to escape use division
                  linktime += *time_offset; //(*time_offset/sf->size.val)*sf->size.val;
                  tsch_slot_offset_t loosetime = sf->size.val*TSCH_TIMESYNC_EB_LOOSES;
                  if (*time_offset > loosetime)
                      linktime -= loosetime;
              }
          }//if (TSCH_SCHEDULE_POLICY & TSCH_SCHEDULE_OMMIT_NOXFER)
        tsch_slot_offset_t time_to_timeslot =
                  linktime > timeslot ?
                  linktime - timeslot :
                  sf->size.val + linktime - timeslot;
        if(curr_best == NULL || time_to_timeslot < time_to_curr_best) {
          time_to_curr_best = time_to_timeslot;
          best_frame        = sf;
          curr_best = l;
#ifdef TSCH_CALLBACK_LINK_SIGNAL
          if ( (l->link_options & (LINK_OPTION_SIGNAL|LINK_OPTION_SIGNAL_ONCE)) != 0) {
              signaling_link = l;
          }
          else
              signaling_link    = NULL;
#endif
          curr_backup = NULL;
        } else if(time_to_timeslot == time_to_curr_best) {
          struct tsch_link *new_best = NULL;
          /* Two links are overlapping, we need to select one of them.
           * By standard: prioritize Tx links first, second by lowest handle */
          if((curr_best->link_options & LINK_OPTION_TX) == (l->link_options & LINK_OPTION_TX)) {
            /* Both or neither links have Tx, select the one with lowest handle */
            if(l->slotframe_handle != curr_best->slotframe_handle) {
              if(l->slotframe_handle < curr_best->slotframe_handle) {
                new_best = l;
              }
            } else {
              /* compare the link against the current best link and return the newly selected one */
              new_best = TSCH_LINK_COMPARATOR(curr_best, l);
            }
          } else {
            /* Select the link that has the Tx option */
            if(l->link_options & LINK_OPTION_TX) {
              new_best = l;
            }
          }

          /* Maintain backup_link */
          if(curr_backup == NULL) {
            /* Check if 'l' best can be used as backup */
            if(new_best != l && (l->link_options & LINK_OPTION_RX)) { /* Does 'l' have Rx flag? */
              curr_backup = l;
            }
            /* Check if curr_best can be used as backup */
            if(new_best != curr_best && (curr_best->link_options & LINK_OPTION_RX)) { /* Does curr_best have Rx flag? */
              curr_backup = curr_best;
            }
          }

          /* Maintain curr_best */
          if(new_best != NULL) {
            curr_best = new_best;
            best_frame  = sf;
          }

#ifdef TSCH_CALLBACK_LINK_SIGNAL
          if ( (l->link_options & (LINK_OPTION_SIGNAL|LINK_OPTION_SIGNAL_ONCE)) != 0) {
              signaling_link = l;
          }
#endif

        }
      }//for(; l != NULL
    }//for(;sf != NULL
    if (curr_best != NULL)
    if(time_offset != NULL) {
      if (TSCH_SCHEDULE_POLICY & TSCH_SCHEDULE_OMMIT_NOXFER)
      if (*time_offset > 0)
      if ((curr_best->link_options & LINK_OPTION_TIME_EB_ESCAPE) != 0)
      {
          tsch_slot_offset_t timeslot = 0;
          timeslot = TSCH_ASN_MOD(*asn, best_frame->size);
          // make fine calculation here using div
          tsch_slot_offset_t sf_size = best_frame->size.val;
          tsch_slot_offset_t linktime = (*time_offset/sf_size);
          if (linktime >= TSCH_TIMESYNC_EB_LOOSES)
              linktime -= TSCH_TIMESYNC_EB_LOOSES;
          linktime = linktime * sf_size;
          linktime += curr_best->timeslot;
          if (timeslot > linktime)
              linktime += sf_size;
          time_to_curr_best = linktime - timeslot;
      }
      assert(time_to_curr_best >= 0);
      *time_offset = time_to_curr_best;
    }
  }
  if(backup_link != NULL) {
    *backup_link = curr_backup;
  }

#ifdef TSCH_CALLBACK_LINK_SIGNAL
  // since need to signal
  if (signaling_link != NULL)
      curr_best->link_options |= LINK_OPTION_SIGNAL_ONCE;
#endif

  return curr_best;
}
/*---------------------------------------------------------------------------*/
/* Module initialization, call only once at startup. Returns 1 is success, 0 if failure. */
int
tsch_schedule_init(void)
{
  if(tsch_get_lock()) {
    memb_init(&link_memb);
    memb_init(&slotframe_memb);
    list_init(slotframe_list);
    tsch_release_lock();
    return 1;
  } else {
    return 0;
  }
}
/*---------------------------------------------------------------------------*/
/* Create a 6TiSCH minimal schedule */
void
tsch_schedule_create_minimal(void)
{
  struct tsch_slotframe *sf_min;
  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();
  /* Build 6TiSCH minimal schedule.
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_min = tsch_schedule_add_slotframe(0, TSCH_SCHEDULE_DEFAULT_LENGTH);
  /* Add a single Tx|Rx|Shared slot using broadcast address (i.e. usable for unicast and broadcast).
   * We set the link type to advertising, which is not compliant with 6TiSCH minimal schedule
   * but is required according to 802.15.4e if also used for EB transmission.
   * Timeslot: 0, channel offset: 0. */
  tsch_schedule_add_link(sf_min,
      (LINK_OPTION_RX | LINK_OPTION_TX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING),
      LINK_TYPE_ADVERTISING, &tsch_broadcast_address,
      0, 0, 1);
}
/*---------------------------------------------------------------------------*/
struct tsch_slotframe *
tsch_schedule_slotframe_head(void)
{
  return list_head(slotframe_list);
}
/*---------------------------------------------------------------------------*/
struct tsch_slotframe *
tsch_schedule_slotframe_next(struct tsch_slotframe *sf)
{
  return list_item_next(sf);
}
/*---------------------------------------------------------------------------*/
/* Prints out the current schedule (all slotframes and links) */
void
tsch_schedule_print(void)
{
  if(!tsch_is_locked()) {
    struct tsch_slotframe *sf = list_head(slotframe_list);

    LOG_PRINT("----- start slotframe list -----\n");

    while(sf != NULL) {
      struct tsch_link *l = list_head(sf->links_list);

      LOG_PRINT("Slotframe Handle %u, size %u\n", sf->handle, sf->size.val);

      while(l != NULL) {
        LOG_PRINT("* Link Options %02x, type %u, timeslot %u, channel offset %u, address %u\n",
               l->link_options, l->link_type, l->timeslot, l->channel_offset, l->addr.u8[7]);
        l = list_item_next(l);
      }

      sf = list_item_next(sf);
    }

    LOG_PRINT("----- end slotframe list -----\n");
  }
}
/*---------------------------------------------------------------------------*/
/** @} */
