/*
 * Copyright (c) 2015, SICS Swedish ICT.
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
 *         TSCH slot operation implementation, running from interrupt.
 * \author
 *         Simon Duquennoy <simonduq@sics.se>
 *         Beshr Al Nahas <beshr@sics.se>
 *         Atis Elsts <atis.elsts@bristol.ac.uk>
 *
 */

/**
 * \addtogroup tsch
 * @{
*/

#include <stdbool.h>
#include "contiki.h"
#include "dev/radio.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/mac/framer/framer-802154.h"
#include "net/mac/tsch/tsch.h"
#include "sys/critical.h"
#include "net/mac/tsch/tsch-private.h"
#include "net/mac/tsch/tsch-slot-operation.h"
#include <stdlib.h>

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "TSCH Slop"
#define LOG_LEVEL LOG_LEVEL_MAC

// turn on TSCH_PRINT if have some LOG_LEVEL_MAC
#if LOG_CONF_LEVEL_MAC > LOG_LEVEL_NONE
#define DEBUG   (DEBUG_PRINT)
#else
#define DEBUG   0
#endif
#include "net/net-debug.h"



/* TSCH debug macros, i.e. to set LEDs or GPIOs on various TSCH
 * timeslot events */
#ifndef TSCH_DEBUG_INIT
#define TSCH_DEBUG_INIT()
#endif
#ifndef TSCH_DEBUG_INTERRUPT
#define TSCH_DEBUG_INTERRUPT()
#endif
#ifndef TSCH_DEBUG_RX_EVENT
#define TSCH_DEBUG_RX_EVENT()
#endif
#ifndef TSCH_DEBUG_TX_EVENT
#define TSCH_DEBUG_TX_EVENT()
#endif
#ifndef TSCH_DEBUG_SLOT_START
#define TSCH_DEBUG_SLOT_START()
#endif
#ifndef TSCH_DEBUG_SLOT_END
#define TSCH_DEBUG_SLOT_END()
#endif

/* Check if TSCH_MAX_INCOMING_PACKETS is power of two */
#if (TSCH_MAX_INCOMING_PACKETS & (TSCH_MAX_INCOMING_PACKETS - 1)) != 0
#error TSCH_MAX_INCOMING_PACKETS must be power of two
#endif

/* Check if TSCH_DEQUEUED_ARRAY_SIZE is power of two and greater or equal to QUEUEBUF_NUM */
#if TSCH_DEQUEUED_ARRAY_SIZE < QUEUEBUF_NUM
#error TSCH_DEQUEUED_ARRAY_SIZE must be greater or equal to QUEUEBUF_NUM
#endif
#if (TSCH_DEQUEUED_ARRAY_SIZE & (TSCH_DEQUEUED_ARRAY_SIZE - 1)) != 0
#error TSCH_DEQUEUED_ARRAY_SIZE must be power of two
#endif

/* Truncate received drift correction information to maximum half
 * of the guard time (one fourth of TSCH_DEFAULT_TS_RX_WAIT) */
#define SYNC_IE_BOUND ((int32_t)US_TO_RTIMERTICKS(tsch_timing_us[tsch_ts_rx_wait] / 4))

#ifdef TSCH_CONF_RTIMER_GUARD
#define RTIMER_GUARD TSCH_CONF_RTIMER_GUARD
#elif  defined(RTIMER_GUARD_TIME)
#define RTIMER_GUARD RTIMER_GUARD_TIME
#else
/* By default: check that rtimer runs at >=32kHz and use a guard time of 10us */
#if RTIMER_SECOND < (8 * 1024)
#error "TSCH: RTIMER_SECOND < (8 * 1024)"
#endif
#if CONTIKI_TARGET_COOJA || CONTIKI_TARGET_COOJA_IP64
/* Use 0 usec guard time for Cooja Mote with a 1 MHz Rtimer*/
#define RTIMER_GUARD 0u
#elif RTIMER_SECOND >= 200000
#define RTIMER_GUARD (RTIMER_SECOND / 100000)
#else
#define RTIMER_GUARD 2u
#endif
#endif //#define RTIMER_GUARD

#if RTIMER_GUARD > 0
#if !defined(RTIMER_ARCH_SECOND)
#warning "TSCH need RTimer resolution better 8khz, when RTIMER_GUARD > 0 , try setup RTIMER_CONF_ARCH_SECOND"
#elif RTIMER_ARCH_SECOND <= 2000
#warning "TSCH need RTimer resolution better 8khz, when RTIMER_GUARD > 0 , try setup RTIMER_CONF_ARCH_SECOND"
#endif
#endif

enum tsch_radio_state_on_cmd {
  TSCH_RADIO_CMD_ON_START_OF_TIMESLOT,
  TSCH_RADIO_CMD_ON_WITHIN_TIMESLOT,
  TSCH_RADIO_CMD_ON_FORCE,
};

enum tsch_radio_state_off_cmd {
  TSCH_RADIO_CMD_OFF_END_OF_TIMESLOT,
  TSCH_RADIO_CMD_OFF_WITHIN_TIMESLOT,
  TSCH_RADIO_CMD_BREAK_NOISE_TIMESLOT,
  TSCH_RADIO_CMD_OFF_FORCE,
};

/* A ringbuf storing outgoing packets after they were dequeued.
 * Will be processed layer by tsch_tx_process_pending */
struct ringbufindex dequeued_ringbuf;
struct tsch_packet *dequeued_array[TSCH_DEQUEUED_ARRAY_SIZE];
/* A ringbuf storing incoming packets.
 * Will be processed layer by tsch_rx_process_pending */
struct ringbufindex input_ringbuf;
struct input_packet input_array[TSCH_MAX_INCOMING_PACKETS];

/* Updates and reads of the next two variables must be atomic (i.e. both together) */
/* Last time we received Sync-IE (ACK or data packet from a time source) */
struct tsch_asn_t tsch_last_sync_asn;
clock_time_t tsch_last_sync_time; /* Same info, in clock_time_t units */

/* A global lock for manipulating data structures safely from outside of interrupt */
volatile bool tsch_locked = 0;
/* As long as this is set, skip all slot operation */
static volatile bool tsch_lock_requested = 0;

/* Last estimated drift in RTIMER ticks
 * (Sky: 1 tick = 30.517578125 usec exactly) */
static int32_t drift_correction = 0;
/* Is drift correction used? (Can be true even if drift_correction == 0) */
static uint8_t is_drift_correction_used;

/* The neighbor last used as our time source */
struct tsch_neighbor *last_timesource_neighbor = NULL;

/* Used from tsch_slot_operation and sub-protothreads */
static rtimer_clock_t volatile current_slot_start;

/* Are we currently inside a slot? */
static volatile int tsch_in_slot_operation = 0;

/* If we are inside a slot, these tell the current channel and channel offset */
uint8_t tsch_current_channel;
tsch_ch_offset_t tsch_current_channel_offset;

/* Info about the link, packet and neighbor of
 * the current (or next) slot */
struct tsch_link *current_link = NULL;
/* A backup link with Rx flag, overlapping with current_link.
 * If the current link is Tx-only and the Tx queue
 * is empty while executing the link, fallback to the backup link. */
static struct tsch_link *backup_link = NULL;
static struct tsch_packet *current_packet = NULL;
static struct tsch_neighbor *current_neighbor = NULL;

/* Indicates whether an extra link is needed to handle the current burst */
static int burst_link_scheduled = 0;
/* Counts the length of the current burst */
int tsch_current_burst_count = 0;

/* Protothread for association */
PT_THREAD(tsch_scan(struct pt *pt));
/* Protothread for slot operation, called from rtimer interrupt
 * and scheduled from tsch_schedule_slot_operation */
static PT_THREAD(tsch_slot_operation(struct rtimer *t, void *ptr));
static struct pt slot_operation_pt;
/* Sub-protothreads of tsch_slot_operation */
static PT_THREAD(tsch_tx_slot(struct pt *pt, struct rtimer *t));
static PT_THREAD(tsch_rx_slot(struct pt *pt, struct rtimer *t));

/*---------------------------------------------------------------------------*/
/* TSCH locking system. TSCH is locked during slot operations */

#if !LIB_INLINES
/* Is TSCH locked? */
bool tsch_is_locked(void)
{
  return tsch_locked;
}

/* Release TSCH lock */
void
tsch_release_lock(void)
{
  tsch_locked = 0;
}
#endif

/* Lock TSCH (no slot operation) */
bool
tsch_get_lock(void)
{
  if(!tsch_locked) {
    rtimer_clock_t busy_wait_time;
    int busy_wait = 0; /* Flag used for logging purposes */
    /* Make sure no new slot operation will start */
    tsch_lock_requested = 1;
    /* Wait for the end of current slot operation. */
    if(tsch_in_slot_operation) {
      busy_wait = 1;
      busy_wait_time = RTIMER_NOW();
      while(tsch_in_slot_operation) {
        watchdog_periodic();
      }
      busy_wait_time = RTIMER_NOW() - busy_wait_time;
    }
    if(!tsch_locked) {
      /* Take the lock if it is free */
      tsch_locked = 1;
      tsch_lock_requested = 0;
      if(busy_wait) {
        /* Issue a log whenever we had to busy wait until getting the lock */
        TSCH_LOG_ADD(tsch_log_message,
            snprintf(log->message, sizeof(log->message),
                "!get lock delay %u", (unsigned)busy_wait_time);
        );
      }
      return 1;
    }
  }
  TSCH_LOG_ADD(tsch_log_message,
      snprintf(log->message, sizeof(log->message),
                      "!failed to lock");
          );
  return 0;
}

/*---------------------------------------------------------------------------*/
/* Channel hopping utility functions */

/* Return the channel offset to use for the current slot */
static tsch_ch_offset_t
tsch_get_channel_offset(struct tsch_link *link, struct tsch_packet *p)
{
#if TSCH_WITH_LINK_SELECTOR
  if(p != NULL) {
    uint16_t packet_channel_offset = queuebuf_attr(p->qb, PACKETBUF_ATTR_TSCH_CHANNEL_OFFSET);
    if(packet_channel_offset != 0xffff) {
      /* The schedule specifies a channel offset for this one; use it */
      return packet_channel_offset;
    }
  }
#endif
  return link->channel_offset;
}

/**
 * Returns a 802.15.4 channel from an ASN and channel offset. Basically adds
 * The offset to the ASN and performs a hopping sequence lookup.
 *
 * \param asn A given ASN
 * \param channel_offset Given channel offset
 * \return The resulting channel
 */
uint8_t
tsch_calculate_channel(struct tsch_asn_t *asn, uint16_t channel_offset)
{
  uint16_t index_of_0, index_of_offset;
  index_of_0 = TSCH_ASN_MOD(*asn, tsch_hopping_sequence_length);
  index_of_offset = (index_of_0 + channel_offset) % tsch_hopping_sequence_length.val;
  return tsch_hopping_sequence[index_of_offset];
}

/*---------------------------------------------------------------------------*/
/* Timing utility functions */

/* Checks if the current time has passed a ref time + offset. Assumes
 * a single overflow and ref time prior to now. */
static uint8_t
check_timer_miss(rtimer_clock_t ref_time, rtimer_clock_t offset, rtimer_clock_t now)
{
  rtimer_clock_t target = ref_time + offset;
  int now_has_overflowed = now < ref_time;
  int target_has_overflowed = target < ref_time;

  if(now_has_overflowed == target_has_overflowed) {
    /* Both or none have overflowed, just compare now to the target */
    return target <= now;
  } else {
    /* Either now or target of overflowed.
     * If it is now, then it has passed the target.
     * If it is target, then we haven't reached it yet.
     *  */
    return now_has_overflowed;
  }
}
/*---------------------------------------------------------------------------*/
/* Schedule a wakeup at a specified offset from a reference time.
 * Provides basic protection against missed deadlines and timer overflows
 * A return value of zero signals a missed deadline: no rtimer was scheduled. */
static uint8_t
tsch_schedule_slot_operation(struct rtimer *tm, rtimer_clock_t ref_time, rtimer_clock_t offset, const char *str)
{
  rtimer_clock_t now = RTIMER_NOW();
  int r;
  /* Subtract RTIMER_GUARD before checking for deadline miss
   * because we can not schedule rtimer less than RTIMER_GUARD in the future */
  int missed = 0;
  if (RTIMER_CLOCK_LT(ref_time + RTIMER_GUARD, now))
      missed = check_timer_miss(ref_time, offset - RTIMER_GUARD, now);

  if(missed) {
    TSCH_LOG_ADD(tsch_log_message,
                snprintf(log->message, sizeof(log->message),
                    "!dl-miss %s %d %d",
                        str, (int)(now-ref_time), (int)offset);
    );

    return 0;
  }
  ref_time += offset - RTIMER_GUARD;
  // FIX: this is when rtimer_set cant overide current timer. When, for some
  //    cause, last timer was expired - in this case expired timer will counts for
  //    rtimer clock override. thus, during override wait it will block new rtimer
  //    operation start.
  rtimer_cancel(tm);
  r = rtimer_set(tm, ref_time, 1, (void (*)(struct rtimer *, void *))tsch_slot_operation, NULL);
  if(r != RTIMER_OK) {
    return 0;
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
/* Schedule slot operation conditionally, and YIELD if success only.
 * Always attempt to schedule RTIMER_GUARD before the target to make sure to wake up
 * ahead of time and then busy wait to exactly hit the target. */
#define TSCH_SCHEDULE_AND_YIELD(pt, tm, ref_time, offset, str) \
  do { \
    if(tsch_schedule_slot_operation(tm, ref_time, offset, str)) { \
      PT_YIELD(pt); \
    } \
    RTIMER_BUSYWAIT_UNTIL_ABS(0, ref_time, offset); \
  } while(0);
/*---------------------------------------------------------------------------*/
/* Get EB, broadcast or unicast packet to be sent, and target neighbor. */
static struct tsch_packet *
get_packet_and_neighbor_for_link(struct tsch_link *link, struct tsch_neighbor **target_neighbor)
{
  struct tsch_packet *p = NULL;
  struct tsch_neighbor *n = NULL;

  /* Is this a Tx link? */
  if(link->link_options & LINK_OPTION_TX) {
    /* is it for advertisement of EB? */
    if(link->link_type == LINK_TYPE_ADVERTISING || link->link_type == LINK_TYPE_ADVERTISING_ONLY) {
      /* fetch EB packets */
      n = n_eb;
      p = tsch_queue_get_packet_for_nbr(n, link);
    }
    if(link->link_type != LINK_TYPE_ADVERTISING_ONLY) {
      /* NORMAL link or no EB to send, pick a data packet */
      if(p == NULL) {
        /* Get neighbor queue associated to the link and get packet from it */
        n = tsch_queue_get_nbr(&link->addr);
        p = tsch_queue_get_packet_for_nbr(n, link);
        /* if it is a broadcast slot and there were no broadcast packets, pick any unicast packet */
        if(p == NULL && n == n_broadcast) {
          p = tsch_queue_get_unicast_packet_for_any(&n, link);
        }
      }
    }
  }
  /* return nbr (by reference) */
  if(target_neighbor != NULL) {
    *target_neighbor = n;
  }

  return p;
}

static
void tsch_slot_operation_update_current_bacokff(void){
    if(current_link != NULL
        && (current_link->link_options & LINK_OPTION_TX)
        && (current_link->link_options & LINK_OPTION_SHARED) )
    {
      /* Decrement the backoff window for all neighbors able to transmit over
       * this Tx, Shared link. */
      tsch_queue_update_all_backoff_windows(&current_link->addr);
    }
}
/*---------------------------------------------------------------------------*/
uint64_t
tsch_get_network_uptime_ticks(void)
{
  uint64_t uptime_asn;
  uint64_t uptime_ticks;
  int_master_status_t status;

  if(!tsch_is_associated) {
    /* not associated, network uptime is not known */
    return (uint64_t)-1;
  }

  status = critical_enter();

  uptime_asn = tsch_last_sync_asn.ls4b + ((uint64_t)tsch_last_sync_asn.ms1b << 32);
  /* first calculate the at the uptime at the last sync in rtimer ticks */
  uptime_ticks = uptime_asn * tsch_timing[tsch_ts_timeslot_length];
  /* then convert to clock ticks (assume that CLOCK_SECOND divides RTIMER_ARCH_SECOND) */
  uptime_ticks /= (RTIMER_ARCH_SECOND / CLOCK_SECOND);
  /* then add the ticks passed since the last timesync */
  uptime_ticks += (clock_time() - tsch_last_sync_time);

  critical_exit(status);

  return uptime_ticks;
}

/*---------------------------------------------------------------------------*/
//* TSCH use state of RF to plan next timeslot operation
enum tsch_rf_states{
    tsch_rfOFF, tsch_rfON
};
typedef enum tsch_rf_states tsch_rf_states;
static
tsch_rf_states tsch_rf_state = tsch_rfOFF;

/**
 * This function turns on the radio. Its semantics is dependent on
 * the value of TSCH_RADIO_ON_DURING_TIMESLOT constant:
 * - if enabled, the radio is turned on at the start of the slot
 * - if disabled, the radio is turned on within the slot,
 *   directly before the packet Rx guard time and ACK Rx guard time.
 */
static void
tsch_radio_on(enum tsch_radio_state_on_cmd command)
{
  int do_it = 0;
  switch(command) {
  case TSCH_RADIO_CMD_ON_START_OF_TIMESLOT:
    if(TSCH_RADIO_ON_DURING_TIMESLOT) {
      do_it = 1;
    }
    break;
  case TSCH_RADIO_CMD_ON_WITHIN_TIMESLOT:
    if(!TSCH_RADIO_ON_DURING_TIMESLOT) {
      do_it = 1;
    }
    break;
  case TSCH_RADIO_CMD_ON_FORCE:
    do_it = 1;
    break;
  }
  if(do_it) {
    NETSTACK_RADIO.on();
    tsch_rf_state = tsch_rfON;
  }
}
/*---------------------------------------------------------------------------*/
//* prognose next active timeslot. for heavy turn on/off RF, this prognose
//* helps to avoid useless radio-off, and save timeslot time for work
static
bool tsch_next_timeslot_far(rtimer_clock_t slot_start);


/**
 * This function turns off the radio. In the same way as for tsch_radio_on(),
 * it depends on the value of TSCH_RADIO_ON_DURING_TIMESLOT constant:
 * - if enabled, the radio is turned off at the end of the slot
 * - if disabled, the radio is turned off within the slot,
 *   directly after Tx'ing or Rx'ing a packet or Tx'ing an ACK.
 */
static void
tsch_radio_off(enum tsch_radio_state_off_cmd command)
{
  int do_it = 0;
  switch(command) {
  case TSCH_RADIO_CMD_OFF_END_OF_TIMESLOT:
    if(TSCH_RADIO_ON_DURING_TIMESLOT) {
      do_it = 1;
    }
    // provide power off between slot, when gaurding time defined
    if (tsch_rf_state > tsch_rfOFF)
    if (tsch_timing[tsch_ts_rfon_prepslot_guard] > 0)
        do_it = 1;
    break;

  case TSCH_RADIO_CMD_OFF_WITHIN_TIMESLOT:
    // provide power off in slot, only when gaurding time not defined
    if (tsch_timing[tsch_ts_rfon_prepslot_guard] <= 0)
    if(!TSCH_RADIO_ON_DURING_TIMESLOT) {
      do_it = 1;
      break;
    }
#if (TSCH_HW_FEATURE & TSCH_HW_FEATURE_BREAK_BY_POWER)
    // process it same as frame break
    // no break
#else
    break;
#endif

  case TSCH_RADIO_CMD_BREAK_NOISE_TIMESLOT:
      // this is need to break current receiving op. do it by invalidate
      //    current power
#if TSCH_HW_FEATURE & TSCH_HW_FEATURE_BREAK_BY_POWER
  {
      radio_value_t pwrlevel;
      radio_result_t ok;
      ok = NETSTACK_RADIO.get_value(RADIO_PARAM_TXPOWER, &pwrlevel);
      if (ok == RADIO_RESULT_OK){
          ok = NETSTACK_RADIO.set_value(RADIO_PARAM_TXPOWER, pwrlevel-10);
          ok = NETSTACK_RADIO.set_value(RADIO_PARAM_TXPOWER, pwrlevel);
      }
      if (ok != RADIO_RESULT_OK)
          do_it = 1;
  }
#else
      do_it = 1;
#endif
      break;

  case TSCH_RADIO_CMD_OFF_FORCE:
    do_it = 1;
    break;
  }
  if(do_it) {
    NETSTACK_RADIO.off();
    tsch_rf_state = tsch_rfOFF;
  }
}

static
bool tsch_next_timeslot_far(rtimer_clock_t slot_start){
    if (tsch_timing[tsch_ts_rfon_prepslot_guard] <= 0)
        return true;

    rtimer_clock_t next_slot_start = slot_start - RTIMER_GUARD;
    rtimer_clock_t now = RTIMER_NOW();
    long timeout = RTIMER_CLOCK_DIFF(next_slot_start, now);
    // use tsch_ts_rfon_prepslot_guard to predict rf off+on time
    const rtimer_clock_t time_gap = tsch_timing[tsch_ts_rfon_prepslot_guard]*2;
    return (timeout > time_gap);
}

unsigned tsch_next_slot_prefetched_time(unsigned timeout){
    if (tsch_rf_state == tsch_rfOFF){
        if (timeout > tsch_timing[tsch_ts_rfon_prepslot_guard])
            timeout -= tsch_timing[tsch_ts_rfon_prepslot_guard];
        else
            return 0;
    }
    return timeout;
}
/*---------------------------------------------------------------------------*/
#if (TSCH_HW_FEATURE & TSCH_HW_FEATURE_SPUROUS_RX) != 0
#define TSCH_HW_SPUROUS_RX 1
#else
#define TSCH_HW_SPUROUS_RX 0
#endif

static
PT_THREAD(tsch_tx_slot(struct pt *pt, struct rtimer *t))
{
  /**
   * TX slot:
   * 1. Copy packet to radio buffer
   * 2. Perform CCA if enabled
   * 3. Sleep until it is time to transmit
   * 4. Wait for ACK if it is a unicast packet
   * 5. Extract drift if we received an E-ACK from a time source neighbor
   * 6. Update CSMA parameters according to TX status
   * 7. Schedule mac_call_sent_callback
   **/

  /* tx status */
  static uint8_t mac_tx_status;
  /* is the packet in its neighbor's queue? */
  uint8_t in_queue;
  static int dequeued_index;
  static bool packet_ready = 1;

  PT_BEGIN(pt);

  TSCH_DEBUG_TX_EVENT();

  /* First check if we have space to store a newly dequeued packet (in case of
   * successful Tx or Drop) */
  dequeued_index = ringbufindex_peek_put(&dequeued_ringbuf);
  mac_tx_status = MAC_TX_ERR_FATAL;
  if(dequeued_index != -1) {
    if(current_packet != NULL && current_packet->qb != NULL) {
      /* packet payload */
      static void *packet;
#if LLSEC802154_ENABLED
      /* encrypted payload */
      static uint8_t encrypted_packet[TSCH_PACKET_MAX_LEN];
#endif /* LLSEC802154_ENABLED */
      /* packet payload length */
      static uint8_t packet_len;
      /* packet seqno */
      static uint8_t seqno;
      /* wait for ack? */
      static uint8_t do_wait_for_ack;
      /* Did we set the frame pending bit to request an extra burst link? */
      static int burst_link_requested;

#if TSCH_CCA_ENABLED
      static uint8_t cca_status;
#endif /* TSCH_CCA_ENABLED */

      /* get payload */
      packet = queuebuf_dataptr(current_packet->qb);
      packet_len = queuebuf_datalen(current_packet->qb);
      /* if is this a broadcast packet, don't wait for ack */
      do_wait_for_ack = !current_neighbor->is_broadcast;
      /* Unicast. More packets in queue for the neighbor? */
      burst_link_requested = 0;
      if(do_wait_for_ack
             && tsch_current_burst_count + 1 < TSCH_BURST_MAX_LEN
             && tsch_queue_nbr_packet_count(current_neighbor) > 1) {
        burst_link_requested = 1;
        tsch_packet_set_frame_pending(packet, packet_len);
      }
      /* read seqno from payload */
      seqno = ((uint8_t *)(packet))[2];
      /* if this is an EB, then update its Sync-IE */
      if(current_neighbor == n_eb) {
        packet_ready = tsch_packet_update_eb(packet, packet_len, current_packet->tsch_sync_ie_offset);
      } else {
        packet_ready = 1;
      }

#if LLSEC802154_ENABLED
      {
        int seclvl = (char)queuebuf_attr(current_packet->qb, PACKETBUF_ATTR_SECURITY_LEVEL);
        if (seclvl > 0){
        /* If we are going to encrypt, we need to generate the output in a separate buffer and keep
         * the original untouched. This is to allow for future retransmissions. */
        char with_encryption = seclvl & 0x4;
        int len =  tsch_security_secure_packet(packet, with_encryption ? encrypted_packet : packet
                      , current_packet->header_len
                      , packet_len - current_packet->header_len
                      , queuebuf_attr(current_packet->qb, PACKETBUF_ATTR_KEY_INDEX), seclvl
                      , queuebuf_addr(current_packet->qb, PACKETBUF_ADDR_RECEIVER)
                      , &tsch_current_asn);
        if (len >= 0) {
            packet_len += len;
        if(with_encryption) {
          packet = encrypted_packet;
        }
        }
        else{
            packet_ready = 0;
            mac_tx_status = MAC_TX_ERR_SEC;
            //abort packet
            current_packet->transmissions = TSCH_MAC_MAX_FRAME_RETRIES;
        }
        }//if (seclvl > 0)
      }
#endif /* LLSEC802154_ENABLED */

      /* prepare packet to send: copy to radio buffer */
      if(packet_ready && NETSTACK_RADIO.prepare(packet, packet_len) == 0) { /* 0 means success */

#if TSCH_CCA_ENABLED
        cca_status = 1;
        /* delay before CCA */
        TSCH_SCHEDULE_AND_YIELD(pt, t, current_slot_start, tsch_timing[tsch_ts_cca_offset], "cca");
        TSCH_DEBUG_TX_EVENT();
        tsch_radio_on(TSCH_RADIO_CMD_ON_WITHIN_TIMESLOT);
        /* CCA */
        RTIMER_BUSYWAIT_UNTIL_ABS(!(cca_status &= NETSTACK_RADIO.channel_clear()),
                           current_slot_start, tsch_timing[tsch_ts_cca_offset] + tsch_timing[tsch_ts_cca]);
        TSCH_DEBUG_TX_EVENT();
        /* there is not enough time to turn radio off */
        /*  NETSTACK_RADIO.off(); */
        if(cca_status == 0) {
          mac_tx_status = MAC_TX_COLLISION;
        } else
#endif /* TSCH_CCA_ENABLED */
        {
          /* delay before TX */
          TSCH_SCHEDULE_AND_YIELD(pt, t, current_slot_start, tsch_timing[tsch_ts_tx_offset] - RADIO_DELAY_BEFORE_TX, "TxBeforeTx");
          TSCH_DEBUG_TX_EVENT();
          /* send packet already in radio tx buffer */
          mac_tx_status = NETSTACK_RADIO.transmit(packet_len);
          /* turn tadio off -- will turn on again to wait for ACK if needed */
          tsch_radio_off(TSCH_RADIO_CMD_OFF_WITHIN_TIMESLOT);

          if(mac_tx_status == RADIO_TX_OK) {
            if(do_wait_for_ack) {
              uint8_t ackbuf[TSCH_PACKET_MAX_LEN];
              int ack_len;
              static rtimer_clock_t ack_start_time;
              int is_time_source;
              struct ieee802154_ies ack_ies;
              uint8_t ack_hdrlen;
              frame802154_t frame;

#if TSCH_RADIO_ON_DURING_TIMESLOT
              // clenup receiving buffer from packets that have ocasionaly received not
              //  in this time-slot
              while (NETSTACK_RADIO.pending_packet()){
                  NETSTACK_RADIO.read(NULL, 0);
              }
#endif

#if TSCH_HW_FRAME_FILTERING
              radio_value_t radio_rx_mode;
              /* Entering promiscuous mode so that the radio accepts the enhanced ACK */
              NETSTACK_RADIO.get_value(RADIO_PARAM_RX_MODE, &radio_rx_mode);
              NETSTACK_RADIO.set_value(RADIO_PARAM_RX_MODE, radio_rx_mode & (~RADIO_RX_MODE_ADDRESS_FILTER));
#endif /* TSCH_HW_FRAME_FILTERING */

#if TSCH_ACK_TIMING_STYLE == TSCH_ACK_TIMING_IMMEDIATE
              ack_start_time = RTIMER_NOW();
#else // TSCH_ACK_TIMING_STYLE == TSCH_ACK_TIMING_OLD
              /* Save tx timestamp */
              rtimer_clock_t tx_start_time = current_slot_start + tsch_timing[tsch_ts_tx_offset];
              rtimer_clock_t tx_duration;
              /* calculate TX duration based on sent packet len */
              tx_duration = TSCH_PACKET_DURATION(packet_len);
              /* limit tx_time to its max value */
              tx_duration = MIN(tx_duration, tsch_timing[tsch_ts_max_tx]);
              ack_start_time = tx_start_time + tx_duration;
#endif
              /* Unicast: wait for ack after tx: sleep until ack time */
              RTIMER_BUSYWAIT_UNTIL_ABS(0, ack_start_time
                  , tsch_timing[tsch_ts_rx_ack_delay] - RADIO_DELAY_BEFORE_RX
                  ); //"TxBeforeAck"
              TSCH_DEBUG_TX_EVENT();
              ack_start_time +=  tsch_timing[tsch_ts_rx_ack_delay];
              tsch_radio_on(TSCH_RADIO_CMD_ON_WITHIN_TIMESLOT);
              /* Wait for ACK to come */
#if 1 //TSCH_RESYNC_WITH_SFD_TIMESTAMPS
              // rely that ACK time more then tsch_ts_ack_wait, so
              //    for moment Tack_start+ack_delay+ack_wait ACK is receiving
              //    or there is no ACK
              TSCH_SCHEDULE_AND_YIELD(pt, t
                  , ack_start_time
                  , tsch_timing[tsch_ts_ack_wait] + RADIO_DELAY_BEFORE_RX
                  , "TxWaitAck");
              TSCH_DEBUG_TX_EVENT();
              ack_start_time += tsch_timing[tsch_ts_ack_wait];
#else
              BUSYWAIT_UNTIL_ABS(NETSTACK_RADIO.receiving_packet()
                   , ack_start_time
                   , tsch_timing[tsch_ts_ack_wait]
                     + RADIO_DELAY_BEFORE_DETECT + RADIO_RSSI_DETECT_DELAY);
              TSCH_DEBUG_TX_EVENT();

              ack_start_time = RTIMER_NOW() - RADIO_DELAY_BEFORE_DETECT;

#endif
              /* Wait for ACK to finish */
              RTIMER_BUSYWAIT_UNTIL_ABS(( !NETSTACK_RADIO.receiving_packet()
#if TSCH_HW_FEATURE & TSCH_HW_FEATURE_RECV_BY_PENDING
                                  || NETSTACK_RADIO.pending_packet()
#endif
                                  ),
                                 ack_start_time, tsch_timing[tsch_ts_max_ack]+ RADIO_DELAY_BEFORE_RX);
              ack_len = 0;
              if (NETSTACK_RADIO.receiving_packet()){
                  TSCH_LOGF("tx ack: tooo long\n");
                  ack_len = -1;
                  // looks like some noise as on air. need to reser RF receiver to prepare
                  //    for next frame
                  tsch_radio_off(TSCH_RADIO_CMD_BREAK_NOISE_TIMESLOT);
              }
              TSCH_DEBUG_TX_EVENT();
              tsch_radio_off(TSCH_RADIO_CMD_OFF_WITHIN_TIMESLOT);

#if TSCH_HW_FRAME_FILTERING
              /* Leaving promiscuous mode */
              NETSTACK_RADIO.get_value(RADIO_PARAM_RX_MODE, &radio_rx_mode);
              NETSTACK_RADIO.set_value(RADIO_PARAM_RX_MODE, radio_rx_mode | RADIO_RX_MODE_ADDRESS_FILTER);
#endif /* TSCH_HW_FRAME_FILTERING */

              /* Read ack frame */
              ack_len = 0;
              do {
              ack_len = NETSTACK_RADIO.read((void *)ackbuf, sizeof(ackbuf));
              //  take only last packet as ACK
              //  this is protection vs spurous packets, generated by some bad radio
              //    on jumming air, or hardware issue.
              } while ( TSCH_HW_SPUROUS_RX && (NETSTACK_RADIO.pending_packet() > 0));

              is_time_source = 0;
              /* The radio driver should return 0 if no valid packets are in the rx buffer */
              if(ack_len > 0) {
                is_time_source = current_neighbor != NULL && current_neighbor->is_time_source;
                if(tsch_packet_parse_eack(ackbuf, ack_len, seqno,
                    &frame, &ack_ies, &ack_hdrlen) == 0) {
                  ack_len = 0;
                }

#if LLSEC802154_ENABLED
                if(ack_len != 0) {
                  if(!tsch_security_parse_frame(ackbuf, ack_hdrlen, ack_len - ack_hdrlen - tsch_security_mic_len(&frame),
                      &frame
                      , queuebuf_addr(current_packet->qb, PACKETBUF_ADDR_RECEIVER)
                      , &tsch_current_asn))
                  {
                    TSCH_LOG_ADD(tsch_log_message,
                        snprintf(log->message, sizeof(log->message),
                        "!failed to authenticate ACK"));
                    ack_len = 0;
                  }
                } else {
                  TSCH_LOG_ADD(tsch_log_message,
                      snprintf(log->message, sizeof(log->message),
                      "!failed to parse ACK"));
                }
#endif /* LLSEC802154_ENABLED */
              }

              if(ack_len != 0) {
                if(is_time_source) {
                  int32_t eack_time_correction = us_to_rtimerticks(ack_ies.ie_time_correction);
                  int32_t since_last_timesync = TSCH_ASN_DIFF(tsch_current_asn, tsch_last_sync_asn);
                  if(eack_time_correction > SYNC_IE_BOUND) {
                    drift_correction = SYNC_IE_BOUND;
                  } else if(eack_time_correction < -SYNC_IE_BOUND) {
                    drift_correction = -SYNC_IE_BOUND;
                  } else {
                    drift_correction = eack_time_correction;
                  }
                  if(drift_correction != eack_time_correction) {
                    TSCH_LOG_ADD(tsch_log_message,
                        snprintf(log->message, sizeof(log->message),
                            "!truncated dr %d %d", (int)eack_time_correction, (int)drift_correction);
                    );
                  }
                  tsch_stats_on_time_synchronization(eack_time_correction);
                  is_drift_correction_used = 1;
                  tsch_timesync_update(current_neighbor, since_last_timesync, drift_correction);
                  /* Keep track of sync time */
                  tsch_last_sync_asn = tsch_current_asn;
                  tsch_schedule_keepalive(0);
                }
                mac_tx_status = MAC_TX_OK;

                /* We requested an extra slot and got an ack. This means
                the extra slot will be scheduled at the received */
                if(burst_link_requested) {
                  burst_link_scheduled = 1;
                }
              } else {
                mac_tx_status = MAC_TX_NOACK;
              }
            } else {
              mac_tx_status = MAC_TX_OK;
            }
          } else {
            mac_tx_status = MAC_TX_ERR;
          }
        }
      } else { //if(packet_ready
        mac_tx_status = MAC_TX_ERR;
      }
    }

    current_packet->transmissions++;
    current_packet->ret = mac_tx_status;

    /* Post TX: Update neighbor queue state */
    in_queue = tsch_queue_packet_sent(current_neighbor, current_packet, current_link, mac_tx_status);

    /* The packet was dequeued, add it to dequeued_ringbuf for later processing */
    if(in_queue == 0) {
      dequeued_array[dequeued_index] = current_packet;
      ringbufindex_put(&dequeued_ringbuf);
    }

    /* If this is an unicast packet to timesource, update stats */
    if(current_neighbor != NULL && current_neighbor->is_time_source) {
      tsch_stats_tx_packet(current_neighbor, mac_tx_status, tsch_current_channel);
    }

    /* Log every tx attempt */
    TSCH_LOG_ADD(tsch_log_tx,
        log->tx.mac_tx_status = mac_tx_status;
    log->tx.num_tx = current_packet->transmissions;
    log->tx.datalen = queuebuf_datalen(current_packet->qb);
    log->tx.drift = drift_correction;
    log->tx.drift_used = is_drift_correction_used;
    log->tx.is_data = ((((uint8_t *)(queuebuf_dataptr(current_packet->qb)))[0]) & 7);// == FRAME802154_DATAFRAME;
#if LLSEC802154_ENABLED
    log->tx.sec_level = queuebuf_attr(current_packet->qb, PACKETBUF_ATTR_SECURITY_LEVEL);
    log->tx.sec_key   = queuebuf_attr(current_packet->qb, PACKETBUF_ATTR_KEY_INDEX);
#else /* LLSEC802154_ENABLED */
    log->tx.sec_level = 0;
#endif /* LLSEC802154_ENABLED */
        linkaddr_copy(&log->tx.dest, queuebuf_addr(current_packet->qb, PACKETBUF_ADDR_RECEIVER));
        log->tx.seqno = queuebuf_attr(current_packet->qb, PACKETBUF_ATTR_MAC_SEQNO);
    );

    /* Poll process for later processing of packet sent events and logs */
    process_poll(&tsch_pending_events_process);
  }
#ifdef TSCH_ON_SLOT_TX
  TSCH_ON_SLOT_TX(mac_tx_status);
#endif

  TSCH_DEBUG_TX_EVENT();

  PT_END(pt);
}
/*---------------------------------------------------------------------------*/
static
PT_THREAD(tsch_rx_slot(struct pt *pt, struct rtimer *t))
{
  /**
   * RX slot:
   * 1. Check if it is used for TIME_KEEPING
   * 2. Sleep and wake up just before expected RX time (with a guard time: TS_LONG_GT)
   * 3. Check for radio activity for the guard time: TS_LONG_GT
   * 4. Prepare and send ACK if needed
   * 5. Drift calculated in the ACK callback registered with the radio driver. Use it if receiving from a time source neighbor.
   **/

  struct tsch_neighbor *n;
  static linkaddr_t source_address;
  static linkaddr_t destination_address;
  static int16_t input_index;
  static int input_queue_drop = 0;
  static bool frame_valid = 0;

  PT_BEGIN(pt);

  TSCH_DEBUG_RX_EVENT();

  frame_valid = 0;
  input_index = ringbufindex_peek_put(&input_ringbuf);
  if(input_index == -1) {
    input_queue_drop++;
  } else {
    static struct input_packet *current_input;
    /* Estimated drift based on RX time */
    static int32_t estimated_drift;
    /* Rx timestamps */
    static rtimer_clock_t rx_start_time;
    static rtimer_clock_t rx_end_time;
    static rtimer_clock_t expected_rx_time;
    uint8_t packet_seen;

    expected_rx_time = current_slot_start + tsch_timing[tsch_ts_tx_offset];
    /* Default start time: expected Rx time */
    rx_start_time = expected_rx_time;

    current_input = &input_array[input_index];

    // clenup receiving buffer from packets that have ocasionaly received not
    //  in this time-slot
#if TSCH_RADIO_ON_DURING_TIMESLOT
    while (NETSTACK_RADIO.pending_packet()){
        NETSTACK_RADIO.read(NULL, 0);
    }
#endif

    /* Wait before starting to listen */
    TSCH_SCHEDULE_AND_YIELD(pt, t, current_slot_start
                            , tsch_timing[tsch_ts_rx_offset] - RADIO_DELAY_BEFORE_RX
                            , "RxBeforeListen");
    TSCH_DEBUG_RX_EVENT();

    /* Start radio for at least guard time */
    tsch_radio_on(TSCH_RADIO_CMD_ON_WITHIN_TIMESLOT);

#if TSCH_RESYNC_WITH_SFD_TIMESTAMPS
    TSCH_SCHEDULE_AND_YIELD(pt, t, current_slot_start
                            , tsch_timing[tsch_ts_rx_offset] + tsch_timing[tsch_ts_rx_wait] + RADIO_DELAY_BEFORE_RX
                            , "RxWait");
    packet_seen = NETSTACK_RADIO.receiving_packet();
    const unsigned pend_limit = tsch_timing[tsch_ts_rx_offset] + tsch_timing[tsch_ts_rx_wait] + RADIO_DELAY_BEFORE_RX;
    const unsigned wait_limit = pend_limit + tsch_timing[tsch_ts_max_tx];
#else
    const unsigned pend_limit = tsch_timing[tsch_ts_rx_offset] + tsch_timing[tsch_ts_rx_wait];
    const unsigned wait_limit = pend_limit + tsch_timing[tsch_ts_max_tx];

    //need to take accurate packet rx time
    packet_seen = NETSTACK_RADIO.receiving_packet();
    if(!packet_seen) {
      /* Check if receiving within guard time */
      RTIMER_BUSYWAIT_UNTIL_ABS((packet_seen = NETSTACK_RADIO.receiving_packet()),
          current_slot_start, pend_limit + RADIO_DELAY_BEFORE_DETECT);
      if (packet_seen){
      rx_start_time = RTIMER_NOW() - RADIO_DELAY_BEFORE_DETECT;
      }
    }
    if (packet_seen) {
      TSCH_DEBUG_RX_EVENT();
      // sure that packet receives till rx wait time. this should help filter out
      // false receiving detection before packet actualy starts receive
      RTIMER_BUSYWAIT_UNTIL_ABS(!(packet_seen = NETSTACK_RADIO.receiving_packet()),
          current_slot_start, pend_limit);
    }
#endif
    if (!packet_seen)
        packet_seen = NETSTACK_RADIO.pending_packet();
    if(!packet_seen) {
      /* no packets on air */
      tsch_radio_off(TSCH_RADIO_CMD_OFF_FORCE);
    } else {
      TSCH_DEBUG_RX_EVENT();

      /* Wait until packet is received, turn radio off */
      RTIMER_BUSYWAIT_UNTIL_ABS( ( !NETSTACK_RADIO.receiving_packet()
#if TSCH_HW_FEATURE & TSCH_HW_FEATURE_RECV_BY_PENDING
                              || (NETSTACK_RADIO.pending_packet() > 0)
#endif
                              ), current_slot_start, wait_limit);
        rx_end_time = RTIMER_NOW();

      TSCH_DEBUG_RX_EVENT();
      tsch_radio_off(TSCH_RADIO_CMD_OFF_WITHIN_TIMESLOT);

      current_input->len = 0;
      /* Read packet */
      // take last packet, and drop others - since them are looks trash
      do {
          current_input->len = NETSTACK_RADIO.read((void *)current_input->payload, TSCH_PACKET_MAX_LEN);
          //  this is protection vs spurous packets, generated by some bad radio
          //    on jumming air, or hardware issue.
          //  \sa TSCH_RADIO_ON_DURING_TIMESLOT - that alredy do this when radio
          //                                      was on between slots
      } while( TSCH_HW_SPUROUS_RX && (NETSTACK_RADIO.pending_packet() > 0) );
      packet_seen = (current_input->len > 0);
      if (!packet_seen){
          // looks like some noise as on air. need to reser RF receiver to prepare
          //    for next frame
          tsch_radio_off(TSCH_RADIO_CMD_BREAK_NOISE_TIMESLOT);
      }
    }
    if(packet_seen) {
        static int header_len;
        static frame802154_t frame;
        radio_value_t radio_last_rssi;
        radio_value_t radio_last_lqi;

        NETSTACK_RADIO.get_value(RADIO_PARAM_LAST_RSSI, &radio_last_rssi);
        current_input->rx_asn = tsch_current_asn;
        current_input->rssi = (signed)radio_last_rssi;
        current_input->channel = tsch_current_channel;
#if TSCH_WITH_LINK_SELECTOR > 1
        current_input->slotframe = current_link->slotframe_handle;
        current_input->timeslot  = current_link->timeslot;
        current_input->choffs    = tsch_current_channel_offset;
#endif
        header_len = frame802154_parse((uint8_t *)current_input->payload, current_input->len, &frame);
        frame_valid = header_len > 0;
        if( !frame802154_check_dest_panid(&frame))
            frame_valid = false;
        if( !frame802154_extract_linkaddr(&frame, &source_address, &destination_address) )
            frame_valid = false;

#if TSCH_RESYNC_WITH_SFD_TIMESTAMPS
        /* At the end of the reception, get an more accurate estimate of SFD arrival time */
        NETSTACK_RADIO.get_object(RADIO_PARAM_LAST_PACKET_TIMESTAMP, &rx_start_time, sizeof(rtimer_clock_t));
#endif

        if(!frame_valid) {
          TSCH_LOG_ADD(tsch_log_message,
              snprintf(log->message, sizeof(log->message),
              "!failed to parse frame %u %u", header_len, current_input->len));
        }

        if(frame_valid) {
          if(frame.fcf.frame_type != FRAME802154_DATAFRAME
            && frame.fcf.frame_type != FRAME802154_BEACONFRAME) {
              TSCH_LOG_ADD(tsch_log_message,
                  snprintf(log->message, sizeof(log->message),
                  "!discarding frame with type %u, len %u", frame.fcf.frame_type, current_input->len));
              frame_valid = 0;
          }
        }

#if LLSEC802154_ENABLED
        /* Decrypt and verify incoming frame */
        if(frame_valid) {
          if (frame.fcf.security_enabled){
          int data_len = current_input->len - tsch_security_mic_len(&frame);
          if(tsch_security_parse_frame(
               current_input->payload, header_len
               , data_len - header_len,
               &frame, &source_address, &tsch_current_asn))
          {
            current_input->len = data_len;
          } else {
            TSCH_LOGF("!failed to authenticate sec%d[%d] frame%d[%u]\n"
                    , frame.aux_hdr.security_control.security_level
                    , frame.aux_hdr.key_index
                    , frame.fcf.frame_type, current_input->len);
            frame_valid = 0;
          }
          }//if (frame.fcf.security_enabled)
        } else {
          TSCH_LOG_ADD(tsch_log_message,
              snprintf(log->message, sizeof(log->message),
              "!failed to authenticate frame %u %u", header_len, current_input->len));
          frame_valid = 0;
        }
#endif /* LLSEC802154_ENABLED */

        if(frame_valid) {
          /* Check that frome is for us or broadcast, AND that it is not from
           * ourselves. This is for consistency with CSMA and to avoid adding
           * ourselves to neighbor tables in case frames are being replayed. */
          if((linkaddr_cmp(&destination_address, &linkaddr_node_addr)
               || linkaddr_cmp(&destination_address, &linkaddr_null))
             && !linkaddr_cmp(&source_address, &linkaddr_node_addr)) {
            int do_nack = 0;
            rx_count++;
            estimated_drift = RTIMER_CLOCK_DIFF(expected_rx_time, rx_start_time);

            tsch_stats_on_time_synchronization(estimated_drift);
            if (abs(estimated_drift) > tsch_timing[tsch_ts_timeslot_length]){
                TSCH_LOG_ADD(tsch_log_rx_drift,
                    log->rx_drift.drift     = estimated_drift;
                    log->rx_drift.expect_us = expected_rx_time;
                    log->rx_drift.start_us  = rx_start_time;
                );
                estimated_drift = 0;
            }

#if TSCH_TIMESYNC_REMOVE_JITTER
            /* remove jitter due to measurement errors */
            if(ABS(estimated_drift) <= TSCH_TIMESYNC_MEASUREMENT_ERROR) {
              estimated_drift = 0;
            } else if(estimated_drift > 0) {
              estimated_drift -= TSCH_TIMESYNC_MEASUREMENT_ERROR;
            } else {
              estimated_drift += TSCH_TIMESYNC_MEASUREMENT_ERROR;
            }
#endif

#ifdef TSCH_CALLBACK_DO_NACK
            if(frame.fcf.ack_required) {
              do_nack = TSCH_CALLBACK_DO_NACK(current_link,
                  &source_address, &destination_address);
            }
#endif

            if(frame.fcf.ack_required) {
              static uint8_t ack_buf[TSCH_PACKET_MAX_LEN];
              static int ack_len;

              /* Build ACK frame */
              ack_len = tsch_packet_create_eack(ack_buf, sizeof(ack_buf),
                  &source_address, &frame, (int16_t)RTIMERTICKS_TO_US(estimated_drift), do_nack);

#if LLSEC802154_ENABLED
              if(ack_len > 0) {
                if(frame.fcf.security_enabled) {
                  /* Secure ACK frame. There is only header and header IEs, therefore data len == 0. */
                  //ack_len += tsch_security_secure_frame(ack_buf, ack_buf, ack_len, 0, &tsch_current_asn);
                    int len = tsch_security_secure_packet(ack_buf, ack_buf, ack_len, 0
                                  , frame.aux_hdr.key_index
                                  , frame.aux_hdr.security_control.security_level
                                  , &source_address, &tsch_current_asn);
                    if (len >= 0){
                        ack_len += len;
                    }
                    else
                        ack_len = 0;
                }
                }
#endif /* LLSEC802154_ENABLED */

              if(ack_len > 0) {
                tsch_radio_off(TSCH_RADIO_CMD_ON_WITHIN_TIMESLOT);
                /* Copy to radio buffer */
                NETSTACK_RADIO.prepare((const void *)ack_buf, ack_len);

#if TSCH_ACK_TIMING_STYLE == TSCH_ACK_TIMING_OLD
                rtimer_clock_t packet_duration;
                packet_duration = TSCH_PACKET_DURATION(current_input->len);
                rx_end_time = rx_start_time + packet_duration;
#endif
                /* Wait for time to ACK and transmit ACK */
                RTIMER_BUSYWAIT_UNTIL_ABS(0, rx_end_time
                         , tsch_timing[tsch_ts_tx_ack_delay] - RADIO_DELAY_BEFORE_TX
                         ); //"RxBeforeAck"
                TSCH_DEBUG_RX_EVENT();
                NETSTACK_RADIO.transmit(ack_len);
                tsch_radio_off(TSCH_RADIO_CMD_OFF_WITHIN_TIMESLOT);

                /* Schedule a burst link iff the frame pending bit was set */
                burst_link_scheduled = tsch_packet_get_frame_pending(current_input->payload, current_input->len);
              }
            }

            /* If the sender is a time source, proceed to clock drift compensation */
            n = tsch_queue_get_nbr(&source_address);
            if(n != NULL && n->is_time_source) {
              int32_t since_last_timesync = TSCH_ASN_DIFF(tsch_current_asn, tsch_last_sync_asn);
              /* Keep track of last sync time */
              tsch_last_sync_asn = tsch_current_asn;
              tsch_last_sync_time = clock_time();
              /* Save estimated drift */
              drift_correction = -estimated_drift;
              is_drift_correction_used = 1;
              sync_count++;
              tsch_timesync_update(n, since_last_timesync, -estimated_drift);
              tsch_schedule_keepalive(0);
            }

            /* Add current input to ringbuf */
            ringbufindex_put(&input_ringbuf);

            /* If the neighbor is known, update its stats */
            if(n != NULL) {
              NETSTACK_RADIO.get_value(RADIO_PARAM_LAST_LINK_QUALITY, &radio_last_lqi);
              tsch_stats_rx_packet(n, current_input->rssi, radio_last_lqi, tsch_current_channel);
            }

            /* Log every reception */
            TSCH_LOG_ADD(tsch_log_rx,
              linkaddr_copy(&log->rx.src, (linkaddr_t *)&frame.src_addr);
              log->rx.is_unicast = frame.fcf.ack_required;
              log->rx.datalen = current_input->len;
              log->rx.drift = drift_correction;
              log->rx.drift_used = is_drift_correction_used;
              log->rx.is_data = frame.fcf.frame_type;// == FRAME802154_DATAFRAME;
              log->rx.estimated_drift = estimated_drift;
              log->rx.seqno = frame.seq;
              if (frame.fcf.security_enabled){
              log->rx.sec_level = frame.aux_hdr.security_control.security_level;
              log->rx.sec_key   = frame.aux_hdr.key_index;
              }
              else {
                  log->rx.sec_level = 0;
                  log->rx.sec_key   = 0;
              }
            );
          }//if(linkaddr_cmp(&destination_address...

          /* Poll process for processing of pending input and logs */
          process_poll(&tsch_pending_events_process);
        }//if(frame_valid)
    }//else(!packet_seen)

    if(input_queue_drop != 0) {
      TSCH_LOG_ADD(tsch_log_message,
          snprintf(log->message, sizeof(log->message),
              "!queue full skipped %u", input_queue_drop);
      );
      input_queue_drop = 0;
    }
  }
#ifdef TSCH_ON_SLOT_RX
  TSCH_ON_SLOT_RX(frame_valid);
#endif

  TSCH_DEBUG_RX_EVENT();

  PT_END(pt);
}
/*---------------------------------------------------------------------------*/
#ifndef TSCH_DESYNC_THRESHOLD_SLOTS
#define TSCH_DESYNC_THRESHOLD_SLOTS() (100 * TSCH_CLOCK_TO_SLOTS(TSCH_DESYNC_THRESHOLD / 100, tsch_timing[tsch_ts_timeslot_length]))
#endif

/* Protothread for slot operation, called from rtimer interrupt
 * and scheduled from tsch_schedule_slot_operation */
extern struct tsch_link *signaling_link;

static
PT_THREAD(tsch_slot_operation(struct rtimer *t, void *ptr))
{
  TSCH_DEBUG_INTERRUPT();
  PT_BEGIN(&slot_operation_pt);

  /* Loop over all active slots */
  while(tsch_is_associated) {

    if(current_link == NULL || tsch_lock_requested) { /* Skip slot operation if there is no link
                                                          or if there is a pending request for getting the lock */
      /* Issue a log whenever skipping a slot */
      TSCH_LOG_ADD(tsch_log_message,
                      snprintf(log->message, sizeof(log->message),
                          "!skipped slot %u %u %u",
                            tsch_locked,
                            tsch_lock_requested,
                            current_link == NULL);
      );

    } else {
      int is_active_slot;
      TSCH_DEBUG_SLOT_START();
      tsch_in_slot_operation = 1;
      /* Measure on-air noise level while TSCH is idle */
      tsch_stats_sample_rssi();
      /* Reset drift correction */
      drift_correction = 0;
      is_drift_correction_used = 0;

      (void)signaling_link; // hide unused warning if no TSCH_CALLBACK_LINK_SIGNAL
#ifdef TSCH_CALLBACK_LINK_SIGNAL
      if ( (current_link->link_options & (LINK_OPTION_SIGNAL | LINK_OPTION_SIGNAL_ONCE)) != 0 ){
          // current link need signal;
          if (signaling_link == NULL)
              signaling_link = current_link;
          TSCH_CALLBACK_LINK_SIGNAL(signaling_link);
          current_link->link_options &= ~LINK_OPTION_SIGNAL_ONCE;
      }
#endif

      /* Get a packet ready to be sent */
      current_packet = get_packet_and_neighbor_for_link(current_link, &current_neighbor);
      /* There is no packet to send, and this link does not have Rx flag. Instead of doing
       * nothing, switch to the backup link (has Rx flag) if any. */
      if(current_packet == NULL
         && !(current_link->link_options & LINK_OPTION_RX)
         && backup_link != NULL)
      {
        // skiped TX slot, so refresh it's backoff if one has blocked
        tsch_slot_operation_update_current_bacokff();

        current_link = backup_link;
        current_packet = get_packet_and_neighbor_for_link(current_link, &current_neighbor);
#ifdef TSCH_CALLBACK_LINK_SIGNAL
        backup_link->link_options  &= ~LINK_OPTION_SIGNAL_ONCE;
#endif
      }
      if ((current_link->link_options & LINK_OPTION_DISABLE) != 0)
          is_active_slot = false;
      else
      is_active_slot = current_packet != NULL || (current_link->link_options & LINK_OPTION_RX);

      if(is_active_slot) {
        /* If we are in a burst, we stick to current channel instead of
         * doing channel hopping, as per IEEE 802.15.4-2015 */
        if(burst_link_scheduled) {
          /* Reset burst_link_scheduled flag. Will be set again if burst continue. */
          burst_link_scheduled = 0;
        } else {
          /* Hop channel */
          tsch_current_channel_offset = tsch_get_channel_offset(current_link, current_packet);
          tsch_current_channel = tsch_calculate_channel(&tsch_current_asn, tsch_current_channel_offset);
        }
        NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, tsch_current_channel);
        /* Turn the radio on already here if configured so; necessary for radios with slow startup */
        tsch_radio_on(TSCH_RADIO_CMD_ON_START_OF_TIMESLOT);
        /* Decide whether it is a TX/RX/IDLE or OFF slot */
        /* Actual slot operation */
        if(current_packet != NULL) {
          /* We have something to transmit, do the following:
           * 1. send
           * 2. update_backoff_state(current_neighbor)
           * 3. post tx callback
           **/
          static struct pt slot_tx_pt;
          PT_SPAWN(&slot_operation_pt, &slot_tx_pt, tsch_tx_slot(&slot_tx_pt, t));
        } else {
          /* Listen */
          static struct pt slot_rx_pt;
          PT_SPAWN(&slot_operation_pt, &slot_rx_pt, tsch_rx_slot(&slot_rx_pt, t));
        }
      }
      else { //if is_active_slot
          if ((current_link->link_options & LINK_OPTION_IDLED_AUTOOFF) != 0)
          if ( (current_neighbor == NULL) || tsch_queue_is_empty(current_neighbor) )
          {
              current_link->link_options |= LINK_OPTION_DISABLE;
          }
        /* Make sure to end the burst in cast, for some reason, we were
         * in a burst but now without any more packet to send. */
        burst_link_scheduled = 0;
      }
      TSCH_DEBUG_SLOT_END();
    }

    /* End of slot operation, schedule next slot or resynchronize */

    tsch_slot_offset_t timeslot_desync = TSCH_DESYNC_THRESHOLD_SLOTS();
    /* Do we need to resynchronize? i.e., wait for EB again */
    if(!tsch_is_coordinator && (TSCH_ASN_DIFF(tsch_current_asn, tsch_last_sync_asn) >timeslot_desync)) 
    {
      TSCH_LOG_ADD(tsch_log_message,
            snprintf(log->message, sizeof(log->message),
                "! leaving the network, last sync %u",
                          (unsigned)TSCH_ASN_DIFF(tsch_current_asn, tsch_last_sync_asn));
      );
      last_timesource_neighbor = NULL;
      tsch_disassociate();
    } else {
      /* backup of drift correction for printing debug messages */
      /* int32_t drift_correction_backup = drift_correction; */
      tsch_slot_offset_t timeslot_diff = 0;
      rtimer_clock_t prev_slot_start;
      /* Time to next wake up */
      rtimer_clock_t time_to_next_active_slot;
      /* Schedule next wakeup skipping slots if missed deadline */
      do {
        tsch_slot_operation_update_current_bacokff();

        /* A burst link was scheduled. Replay the current link at the
        next time offset */
        if(burst_link_scheduled && current_link != NULL) {
          timeslot_diff = 1;
          backup_link = NULL;
          /* Keep track of the number of repetitions */
          tsch_current_burst_count++;
        } else {
          /* Get next active link */
          timeslot_diff = timeslot_desync;
          current_link = tsch_schedule_get_next_active_link(&tsch_current_asn, &timeslot_diff, &backup_link);
          if(current_link == NULL) {
            /* There is no next link. Fall back to default
             * behavior: wake up at the next slot. */
            timeslot_diff = 1;
          } else {
            /* Reset burst index now that the link was scheduled from
              normal schedule (as opposed to from ongoing burst) */
            tsch_current_burst_count = 0;
          }
        }
        /* Update ASN */
        TSCH_ASN_INC(tsch_current_asn, timeslot_diff);
        /* Time to next wake up */
        time_to_next_active_slot = timeslot_diff * tsch_timing[tsch_ts_timeslot_length] + drift_correction;
        time_to_next_active_slot += tsch_timesync_adaptive_compensate(time_to_next_active_slot);
        drift_correction = 0;
        is_drift_correction_used = 0;
        /* Update current slot start */
        prev_slot_start = current_slot_start;
        current_slot_start += time_to_next_active_slot;
        if (tsch_next_timeslot_far(current_slot_start)){
            tsch_radio_off(TSCH_RADIO_CMD_OFF_END_OF_TIMESLOT);
        }
        else {
                TSCH_LOG_ADD(tsch_log_message,
                             snprintf(log->message, sizeof(log->message)
                                 , "TSCH:supress rf off, slot %ldus\n"
                                 , (long)(time_to_next_active_slot-RTIMER_NOW())
                                 );
                );
        }
        if (tsch_rf_state < tsch_rfON)
            time_to_next_active_slot = tsch_next_slot_prefetched_time(time_to_next_active_slot);
      } while(!tsch_schedule_slot_operation(t, prev_slot_start, time_to_next_active_slot, "main"));
    }

    tsch_in_slot_operation = 0;

    if (!tsch_is_associated)
        break;
    PT_YIELD(&slot_operation_pt);
  }//while(tsch_is_associated)
  tsch_radio_off(TSCH_RADIO_CMD_OFF_FORCE);

  PT_END(&slot_operation_pt);
}
/*---------------------------------------------------------------------------*/
/* Set global time before starting slot operation,
 * with a rtimer time and an ASN */
static struct rtimer tsch_slot_operation_timer;
void
tsch_slot_operation_start(void)
{
  rtimer_clock_t time_to_next_active_slot;
  rtimer_clock_t prev_slot_start;
  TSCH_DEBUG_INIT();
  do {
    tsch_slot_offset_t timeslot_diff;
    /* Get next active link */
    current_link = tsch_schedule_get_next_active_link(&tsch_current_asn, &timeslot_diff, &backup_link);
    if(current_link == NULL) {
      /* There is no next link. Fall back to default
       * behavior: wake up at the next slot. */
      timeslot_diff = 1;
    }
    /* Update ASN */
    TSCH_ASN_INC(tsch_current_asn, timeslot_diff);
    /* Time to next wake up */
    time_to_next_active_slot = timeslot_diff * tsch_timing[tsch_ts_timeslot_length];
    /* Update current slot start */
    prev_slot_start = current_slot_start;
    current_slot_start += time_to_next_active_slot;
    // forced use RF power prefetch, to ensure if RF powered off.
    time_to_next_active_slot = tsch_next_slot_prefetched_time(time_to_next_active_slot);
  } while(!tsch_schedule_slot_operation(&tsch_slot_operation_timer, prev_slot_start, time_to_next_active_slot, "association"));
}
/*---------------------------------------------------------------------------*/
void tsch_slot_operation_stop(void){
    rtimer_cancel(&tsch_slot_operation_timer);
    tsch_in_slot_operation = 0;
}

/*---------------------------------------------------------------------------*/
bool tsch_slot_operation_break_before(rtimer_clock_t timeout){
    if (!tsch_in_slot_operation)
        return true;
    rtimer_clock_t now = RTIMER_NOW();
    rtimer_clock_t time_to_next_active_slot = current_slot_start - now;
    if (time_to_next_active_slot < timeout)
        return false;

    tsch_slot_operation_stop();
    // evaluate current ASN.
    unsigned asns = time_to_next_active_slot/tsch_timing[tsch_ts_timeslot_length];
    rtimer_clock_t time_estimated_slots;
    time_estimated_slots = asns*tsch_timing[tsch_ts_timeslot_length];
    time_estimated_slots += tsch_timesync_adaptive_compensate(time_estimated_slots);
    TSCH_ASN_DEC(tsch_current_asn, asns);
    current_slot_start -= time_estimated_slots;
    return true;
}

bool tsch_slot_operation_invalidate_before(rtimer_clock_t timeout){
      if( tsch_slot_operation_break_before(timeout) ){
          tsch_slot_operation_start();
          return true;
      }
      return false;
}

/*---------------------------------------------------------------------------*/
/* Start actual slot operation */
void
tsch_slot_operation_sync(rtimer_clock_t next_slot_start,
    struct tsch_asn_t *next_slot_asn)
{
  int_master_status_t status;

  current_slot_start = next_slot_start;
  tsch_current_asn = *next_slot_asn;
  status = critical_enter();
  tsch_last_sync_asn = tsch_current_asn;
  tsch_last_sync_time = clock_time();
  critical_exit(status);
  current_link = NULL;
  TSCH_PRINTF("TSCH: sync at %x.%lx[asn] - %lu[rtc]\n"
          , tsch_current_asn.ms1b , tsch_current_asn.ls4b
          , current_slot_start);
}
/*---------------------------------------------------------------------------*/
