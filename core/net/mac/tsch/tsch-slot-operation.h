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

#ifndef __TSCH_SLOT_OPERATION_H__
#define __TSCH_SLOT_OPERATION_H__

/********** Includes **********/

#include <stdbool.h>
#include "contiki.h"
#include <stdbool.h>
#include "lib/ringbufindex.h"
#include "net/mac/tsch/tsch-private.h"
#include "net/mac/tsch/tsch-packet.h"

/******** Configuration *******/

/* Size of the ring buffer storing dequeued outgoing packets (only an array of pointers).
 * Must be power of two, and greater or equal to QUEUEBUF_NUM */
#ifdef TSCH_CONF_DEQUEUED_ARRAY_SIZE
#define TSCH_DEQUEUED_ARRAY_SIZE TSCH_CONF_DEQUEUED_ARRAY_SIZE
#else
/* By default, round QUEUEBUF_CONF_NUM to next power of two
 * (in the range [4;256]) */
#if QUEUEBUF_CONF_NUM <= 4
#define TSCH_DEQUEUED_ARRAY_SIZE 4
#elif QUEUEBUF_CONF_NUM <= 8
#define TSCH_DEQUEUED_ARRAY_SIZE 8
#elif QUEUEBUF_CONF_NUM <= 16
#define TSCH_DEQUEUED_ARRAY_SIZE 16
#elif QUEUEBUF_CONF_NUM <= 32
#define TSCH_DEQUEUED_ARRAY_SIZE 32
#elif QUEUEBUF_CONF_NUM <= 64
#define TSCH_DEQUEUED_ARRAY_SIZE 64
#elif QUEUEBUF_CONF_NUM <= 128
#define TSCH_DEQUEUED_ARRAY_SIZE 128
#else
#define TSCH_DEQUEUED_ARRAY_SIZE 256
#endif
#endif

/* Size of the ring buffer storing incoming packets.
 * Must be power of two */
#ifdef TSCH_CONF_MAX_INCOMING_PACKETS
#define TSCH_MAX_INCOMING_PACKETS TSCH_CONF_MAX_INCOMING_PACKETS
#else
#define TSCH_MAX_INCOMING_PACKETS 4
#endif

/*********** Callbacks *********/

/* Called by TSCH form interrupt after receiving a frame, enabled upper-layer to decide
 * whether to ACK or NACK */
#ifdef TSCH_CALLBACK_DO_NACK
int TSCH_CALLBACK_DO_NACK(struct tsch_link *link, linkaddr_t *src, linkaddr_t *dst);
#endif

/************ Types ***********/

/***** External Variables *****/

/* A ringbuf storing outgoing packets after they were dequeued.
 * Will be processed layer by tsch_tx_process_pending */
extern struct ringbufindex dequeued_ringbuf;
extern struct tsch_packet *dequeued_array[TSCH_DEQUEUED_ARRAY_SIZE];
/* A ringbuf storing incoming packets.
 * Will be processed layer by tsch_rx_process_pending */
extern struct ringbufindex input_ringbuf;
extern struct input_packet input_array[TSCH_MAX_INCOMING_PACKETS];

/********** Functions *********/

/* Returns a 802.15.4 channel from an ASN and channel offset */
uint8_t tsch_calculate_channel(struct tsch_asn_t *asn, int_fast8_t channel_offset);
/* Set global time before starting slot operation,
 * with a rtimer time and an ASN */
void tsch_slot_operation_sync(rtimer_clock_t next_slot_start,
    struct tsch_asn_t *next_slot_asn);
/* Start actual slot operation */
void tsch_slot_operation_start(void);
void tsch_slot_operation_stop(void);
// \brief Break current slot operation correctrly - in sync state, valid for later
//  tsch_slot_operation_start invoke.
//  \arg timeout - time [rtc] to planed next slot, that not breaks.
//          Break later oparation that planed: now+timeout > current_slot_start
// * usage:
//      if( tsch_slot_operation_break_before(timeout) ){
//          make_some_scheduler_changes();
//          tsch_slot_operation_start();
//      }
bool tsch_slot_operation_break_before(rtimer_clock_t timeout);
// this is a kind of:
//      if( tsch_slot_operation_break_before(timeout) ){
//          tsch_slot_operation_start();
//      }
bool tsch_slot_operation_invalidate_before(rtimer_clock_t timeout);



#ifndef LIB_INLINES
#if (PACKETBUF_CONF_ATTRS_INLINE) //|| defined(__GNUC__)
#define LIB_INLINES     1
#else
#define LIB_INLINES     0
#endif
#endif //LIB_INLINES

#if LIB_INLINES
extern
volatile bool tsch_locked;
static inline
bool tsch_is_locked(void){return tsch_locked;};
static inline
void tsch_release_lock(void){tsch_locked = 0;};

#else
/* Is TSCH locked? */
bool tsch_is_locked(void);
/* Release TSCH lock */
void tsch_release_lock(void);
#endif
/* Lock TSCH (no link operation) */
bool tsch_get_lock(void);


#endif /* __TSCH_SLOT_OPERATION_H__ */