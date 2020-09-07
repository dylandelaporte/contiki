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
* \addtogroup tsch
* @{
 * \file
 *         TSCH configuration
 * \author
 *         Simon Duquennoy <simonduq@sics.se>
 */

#ifndef __TSCH_CONF_H__
#define __TSCH_CONF_H__

/********** Includes **********/

#include "contiki.h"

/* Include Project Specific conf for TSCH*/
#ifdef TSCH_CONF_H
#include TSCH_CONF_H
#endif /* PROJECT_CONF_H */

/******** Configuration: synchronization *******/

/* Max time before sending a unicast keep-alive message to the time source */
#ifdef TSCH_CONF_KEEPALIVE_TIMEOUT
#define TSCH_KEEPALIVE_TIMEOUT TSCH_CONF_KEEPALIVE_TIMEOUT
#else
/* Time to desynch assuming a drift of 40 PPM (80 PPM between two nodes) and guard time of +/-1ms: 12.5s. */
#define TSCH_KEEPALIVE_TIMEOUT (12 * CLOCK_SECOND)
#endif

/* With TSCH_ADAPTIVE_TIMESYNC enabled: keep-alive timeout used after reaching
 * accurate drift compensation. */
#ifdef TSCH_CONF_MAX_KEEPALIVE_TIMEOUT
#define TSCH_MAX_KEEPALIVE_TIMEOUT TSCH_CONF_MAX_KEEPALIVE_TIMEOUT
#else
#define TSCH_MAX_KEEPALIVE_TIMEOUT (60 * CLOCK_SECOND)
#endif

/* Max time without synchronization before leaving the PAN */
#ifdef TSCH_CONF_DESYNC_THRESHOLD
#define TSCH_DESYNC_THRESHOLD TSCH_CONF_DESYNC_THRESHOLD
#else
#define TSCH_DESYNC_THRESHOLD (2 * TSCH_MAX_KEEPALIVE_TIMEOUT)
#endif

/* The default period between two consecutive EBs (not taking into account any randomization).
 * When TSCH_CONF_EB_PERIOD is set to 0, sending EBs is disabled completely; the EB process is not started.
 * Otherwise, if RPL is used, TSCH_CONF_EB_PERIOD used only before joining the RPL network;
 * afterwards, the EB period is set dynamically based on RPL DIO period, updated whenever
 * the DIO period changes, and is upper bounded by TSCH_MAX_EB_PERIOD.
 */
#ifdef TSCH_CONF_EB_PERIOD
#define TSCH_EB_PERIOD TSCH_CONF_EB_PERIOD
#else
#define TSCH_EB_PERIOD (16 * CLOCK_SECOND)
#endif

/* Max Period between two consecutive EBs.
 * Has no effect when TSCH_EB_PERIOD is zero. */
#ifdef TSCH_CONF_MAX_EB_PERIOD
#define TSCH_MAX_EB_PERIOD TSCH_CONF_MAX_EB_PERIOD
#else
#define TSCH_MAX_EB_PERIOD (16 * CLOCK_SECOND)
#endif

/* Use SFD timestamp for synchronization? By default we merely rely on rtimer and busy wait
 * until SFD is high, which we found to provide greater accuracy on JN516x and CC2420.
 * Note: for association, however, we always use SFD timestamp to know the time of arrival
 * of the EB (because we do not busy-wait for the whole scanning process)
 * */
#ifdef TSCH_CONF_RESYNC_WITH_SFD_TIMESTAMPS
#define TSCH_RESYNC_WITH_SFD_TIMESTAMPS TSCH_CONF_RESYNC_WITH_SFD_TIMESTAMPS
#else
#define TSCH_RESYNC_WITH_SFD_TIMESTAMPS 1
#endif

/* If enabled, remove jitter due to measurement errors */
#ifdef TSCH_CONF_TIMESYNC_REMOVE_JITTER
#define TSCH_TIMESYNC_REMOVE_JITTER TSCH_CONF_TIMESYNC_REMOVE_JITTER
#else
// if have hw SFD stamps - measure errors miserable, so save a few codesize,
//      by ommiting jitter threshold.
#define TSCH_TIMESYNC_REMOVE_JITTER (!TSCH_RESYNC_WITH_SFD_TIMESTAMPS)
#endif

/* Base drift value.
 * Used to compensate locally know inaccuracies, such as
 * the effect of having a binary 32.768 kHz timer as the TSCH time base. */
#ifdef TSCH_CONF_BASE_DRIFT_PPM
#define TSCH_BASE_DRIFT_PPM TSCH_CONF_BASE_DRIFT_PPM
#else
#define TSCH_BASE_DRIFT_PPM 0
#endif

/* Estimate the drift of the time-source neighbor and compensate for it? */
#ifdef TSCH_CONF_ADAPTIVE_TIMESYNC
#define TSCH_ADAPTIVE_TIMESYNC TSCH_CONF_ADAPTIVE_TIMESYNC
#else
#define TSCH_ADAPTIVE_TIMESYNC 1
#endif

/* By default: TSCH loads slot timing from coordinator EB
 * but for debug purposes it can be ommited by enabling this macro*/
//#define TSCH_DEBUG_NO_TIMING_FROM_EB

/* An ad-hoc mechanism to have TSCH select its time source without the
 * help of an upper-layer, simply by collecting statistics on received
 * EBs and their join priority. Disabled by default as we recomment
 * mapping the time source on the RPL preferred parent
 * (via tsch_rpl_callback_parent_switch) */
#ifdef TSCH_CONF_AUTOSELECT_TIME_SOURCE
#define TSCH_AUTOSELECT_TIME_SOURCE TSCH_CONF_AUTOSELECT_TIME_SOURCE
#else
#define TSCH_AUTOSELECT_TIME_SOURCE 0
#endif /* TSCH_CONF_EB_AUTOSELECT */

/******** Configuration: channel hopping *******/

/* Default hopping sequence, used in case hopping sequence ID == 0 */
#ifdef TSCH_CONF_DEFAULT_HOPPING_SEQUENCE
#define TSCH_DEFAULT_HOPPING_SEQUENCE TSCH_CONF_DEFAULT_HOPPING_SEQUENCE
#else
#define TSCH_DEFAULT_HOPPING_SEQUENCE TSCH_HOPPING_SEQUENCE_4_4
#endif

/* Hopping sequence used for joining (scan channels) */
#ifdef TSCH_CONF_JOIN_HOPPING_SEQUENCE
#define TSCH_JOIN_HOPPING_SEQUENCE TSCH_CONF_JOIN_HOPPING_SEQUENCE
#else
#define TSCH_JOIN_HOPPING_SEQUENCE TSCH_DEFAULT_HOPPING_SEQUENCE
#endif

#ifndef TSCH_JOIN_HOPPING_SEQUENCE_SIZE
#define TSCH_JOIN_HOPPING_SEQUENCE_SIZE() sizeof(TSCH_JOIN_HOPPING_SEQUENCE)
#endif

/* Maximum length of the TSCH channel hopping sequence. Must be greater or
 * equal to the length of TSCH_DEFAULT_HOPPING_SEQUENCE. */
#ifdef TSCH_CONF_HOPPING_SEQUENCE_MAX_LEN
#define TSCH_HOPPING_SEQUENCE_MAX_LEN TSCH_CONF_HOPPING_SEQUENCE_MAX_LEN
#else
#define TSCH_HOPPING_SEQUENCE_MAX_LEN sizeof(TSCH_DEFAULT_HOPPING_SEQUENCE)
#endif

/******** Configuration: association *******/

/* Start TSCH automatically after init? If not, the upper layers
 * must call NETSTACK_MAC.on() to start it. Useful when the
 * application needs to control when the nodes are to start
 * scanning or advertising.*/
#ifdef TSCH_CONF_AUTOSTART
#define TSCH_AUTOSTART TSCH_CONF_AUTOSTART
#else
#define TSCH_AUTOSTART 1
#endif

/* Max acceptable join priority */
#ifdef TSCH_CONF_MAX_JOIN_PRIORITY
#define TSCH_MAX_JOIN_PRIORITY TSCH_CONF_MAX_JOIN_PRIORITY
#else
#define TSCH_MAX_JOIN_PRIORITY 32
#endif

/* Join only secured networks? (discard EBs with security disabled) */
#ifdef TSCH_CONF_JOIN_SECURED_ONLY
#define TSCH_JOIN_SECURED_ONLY TSCH_CONF_JOIN_SECURED_ONLY
#else
/* By default, set if LLSEC802154_ENABLED is also non-zero */
#define TSCH_JOIN_SECURED_ONLY LLSEC802154_ENABLED
#endif
// Style of nodes scan chanels during association
//< select random chanel in TSCH_JOIN_HOPPING_SEQUENCE
#define TSCH_JOIN_HOPPING_RANDOM  0
//< select consequntly chanel in TSCH_JOIN_HOPPING_SEQUENCE, one by one
#define TSCH_JOIN_HOPPING_STEPPED 1

#ifdef TSCH_CONF_JOIN_STYLE
#define TSCH_JOIN_STYLE TSCH_CONF_JOIN_STYLE
#else
#define TSCH_JOIN_STYLE TSCH_JOIN_HOPPING_RANDOM
#endif

/* By default, join any PAN ID. Otherwise, wait for an EB from IEEE802154_PANID */
#ifdef TSCH_CONF_JOIN_MY_PANID_ONLY
#define TSCH_JOIN_MY_PANID_ONLY TSCH_CONF_JOIN_MY_PANID_ONLY
#else
#define TSCH_JOIN_MY_PANID_ONLY 1
#endif

/* The radio polling frequency (in Hz) during association process */
#ifdef TSCH_CONF_ASSOCIATION_POLL_FREQUENCY
#define TSCH_ASSOCIATION_POLL_FREQUENCY TSCH_CONF_ASSOCIATION_POLL_FREQUENCY
#else
#define TSCH_ASSOCIATION_POLL_FREQUENCY 100
#endif

/* When associating, check ASN against our own uptime (time in minutes)..
 * Useful to force joining only with nodes started roughly at the same time.
 * Set to the max number of minutes acceptable. */
#ifdef TSCH_CONF_CHECK_TIME_AT_ASSOCIATION
#define TSCH_CHECK_TIME_AT_ASSOCIATION TSCH_CONF_CHECK_TIME_AT_ASSOCIATION
#else
#define TSCH_CHECK_TIME_AT_ASSOCIATION 0
#endif

/* Association on turn-on strategy:
 * 0 - associate cycling until success
 * 1 - associate once, if not succeed, invoke disassociate */
#ifdef TSCH_CONF_ASSOCIATION_SINGLE
#define TSCH_ASSOCIATION_SINGLE TSCH_CONF_ASSOCIATION_SINGLE
#else
#define TSCH_ASSOCIATION_SINGLE 0
#endif

/* How long to scan each channel in the scanning phase */
#ifdef TSCH_CONF_CHANNEL_SCAN_DURATION
#define TSCH_CHANNEL_SCAN_DURATION TSCH_CONF_CHANNEL_SCAN_DURATION
#else
#define TSCH_CHANNEL_SCAN_DURATION CLOCK_SECOND
#endif

//  select start conditions on scan.
//  should define macro on TSCH_JOIN_HOPPING_START(ch, time)
//      where ch - local scaner ch variable
//            time - local scaner last scan variable
//  * usage:
//      - assign start scaning with  my_faivorite_chanel
//  #define TSCH_CONF_JOIN_HOPPING_START(ch, time) {ch = my_faivorite_chanel;}
//      - force to choose new chanel for scan
//  #define TSCH_CONF_JOIN_HOPPING_START(ch, time) {time -= TSCH_CHANNEL_SCAN_DURATION;}
//
#ifdef TSCH_CONF_JOIN_HOPPING_START
#define TSCH_JOIN_HOPPING_START(ch, time)  TSCH_CONF_JOIN_HOPPING_START(ch, time)
#else
#define TSCH_JOIN_HOPPING_START(ch, time)
#endif

/* By default: initialize schedule from EB when associating, using the
 * slotframe and links Information Element */
#ifdef TSCH_CONF_INIT_SCHEDULE_FROM_EB
#define TSCH_INIT_SCHEDULE_FROM_EB TSCH_CONF_INIT_SCHEDULE_FROM_EB
#else
#define TSCH_INIT_SCHEDULE_FROM_EB 1
#endif

/* How long to scan each channel in the scanning phase */
#ifdef TSCH_CONF_CHANNEL_SCAN_DURATION
#define TSCH_CHANNEL_SCAN_DURATION TSCH_CONF_CHANNEL_SCAN_DURATION
#else
#define TSCH_CHANNEL_SCAN_DURATION CLOCK_SECOND
#endif

/* TSCH EB: include timeslot timing Information Element? */
#ifdef TSCH_PACKET_CONF_EB_WITH_TIMESLOT_TIMING
#define TSCH_PACKET_EB_WITH_TIMESLOT_TIMING TSCH_PACKET_CONF_EB_WITH_TIMESLOT_TIMING
#else
#define TSCH_PACKET_EB_WITH_TIMESLOT_TIMING 0
#endif

/* TSCH EB: include hopping sequence Information Element? */
#ifdef TSCH_PACKET_CONF_EB_WITH_HOPPING_SEQUENCE
#define TSCH_PACKET_EB_WITH_HOPPING_SEQUENCE TSCH_PACKET_CONF_EB_WITH_HOPPING_SEQUENCE
#else
#define TSCH_PACKET_EB_WITH_HOPPING_SEQUENCE 0
#endif

/* TSCH EB: include slotframe and link Information Element? */
#ifdef TSCH_PACKET_CONF_EB_WITH_SLOTFRAME_AND_LINK
#define TSCH_PACKET_EB_WITH_SLOTFRAME_AND_LINK TSCH_PACKET_CONF_EB_WITH_SLOTFRAME_AND_LINK
#else
#define TSCH_PACKET_EB_WITH_SLOTFRAME_AND_LINK 0
#endif

/******** Configuration: queues  *******/

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

/* The maximum number of outgoing packets towards each neighbor
 * Must be power of two to enable atomic ringbuf operations.
 * Note: the total number of outgoing packets in the system (for
 * all neighbors) is defined via QUEUEBUF_CONF_NUM */
#ifdef TSCH_QUEUE_CONF_NUM_PER_NEIGHBOR
#define TSCH_QUEUE_NUM_PER_NEIGHBOR TSCH_QUEUE_CONF_NUM_PER_NEIGHBOR
#else
/* By default, round QUEUEBUF_CONF_NUM to next power of two
 * (in the range [4;256]) */
#if QUEUEBUF_CONF_NUM <= 4
#define TSCH_QUEUE_NUM_PER_NEIGHBOR 4
#elif QUEUEBUF_CONF_NUM <= 8
#define TSCH_QUEUE_NUM_PER_NEIGHBOR 8
#elif QUEUEBUF_CONF_NUM <= 16
#define TSCH_QUEUE_NUM_PER_NEIGHBOR 16
#elif QUEUEBUF_CONF_NUM <= 32
#define TSCH_QUEUE_NUM_PER_NEIGHBOR 32
#elif QUEUEBUF_CONF_NUM <= 64
#define TSCH_QUEUE_NUM_PER_NEIGHBOR 64
#elif QUEUEBUF_CONF_NUM <= 128
#define TSCH_QUEUE_NUM_PER_NEIGHBOR 128
#else
#define TSCH_QUEUE_NUM_PER_NEIGHBOR 256
#endif
#endif

/* The number of neighbor queues. There are two queues allocated at all times:
 * one for EBs, one for broadcasts. Other queues are for unicast to neighbors */
#ifdef TSCH_QUEUE_CONF_MAX_NEIGHBOR_QUEUES
#define TSCH_QUEUE_MAX_NEIGHBOR_QUEUES TSCH_QUEUE_CONF_MAX_NEIGHBOR_QUEUES
#else
#define TSCH_QUEUE_MAX_NEIGHBOR_QUEUES ((NBR_TABLE_CONF_MAX_NEIGHBORS) + 2)
#endif

/******** Configuration: scheduling  *******/

/* Initializes TSCH with a 6TiSCH minimal schedule */
#ifdef TSCH_SCHEDULE_CONF_WITH_6TISCH_MINIMAL
#define TSCH_SCHEDULE_WITH_6TISCH_MINIMAL TSCH_SCHEDULE_CONF_WITH_6TISCH_MINIMAL
#else
#define TSCH_SCHEDULE_WITH_6TISCH_MINIMAL (!(BUILD_WITH_ORCHESTRA))
#endif

/* Set an upper bound on burst length. Set to 0 to never set the frame pending
 * bit, i.e., never trigger a burst. Note that receiver-side support for burst
 * is always enabled, as it is part of IEEE 802.1.5.4-2015 (Section 7.2.1.3)*/
#ifdef TSCH_CONF_BURST_MAX_LEN
#define TSCH_BURST_MAX_LEN TSCH_CONF_BURST_MAX_LEN
#else
#define TSCH_BURST_MAX_LEN 32
#endif

/* 6TiSCH Minimal schedule slotframe length */
#ifdef TSCH_SCHEDULE_CONF_DEFAULT_LENGTH
#define TSCH_SCHEDULE_DEFAULT_LENGTH TSCH_SCHEDULE_CONF_DEFAULT_LENGTH
#else
#define TSCH_SCHEDULE_DEFAULT_LENGTH 7
#endif

/* Max number of TSCH slotframes */
#ifdef TSCH_SCHEDULE_CONF_MAX_SLOTFRAMES
#define TSCH_SCHEDULE_MAX_SLOTFRAMES TSCH_SCHEDULE_CONF_MAX_SLOTFRAMES
#else
#define TSCH_SCHEDULE_MAX_SLOTFRAMES 4
#endif

/* Max number of links */
#ifdef TSCH_SCHEDULE_CONF_MAX_LINKS
#define TSCH_SCHEDULE_MAX_LINKS TSCH_SCHEDULE_CONF_MAX_LINKS
#else
#define TSCH_SCHEDULE_MAX_LINKS 32
#endif

/* To include Sixtop Implementation */
#ifdef TSCH_CONF_WITH_SIXTOP
#define TSCH_WITH_SIXTOP TSCH_CONF_WITH_SIXTOP
#else
#define TSCH_WITH_SIXTOP 0
#endif

/* A custom feature allowing upper layers to assign packets to
 * a specific slotframe and link */
// 1 - enables PACKETBUF_ATTR_TSCH_SLOTFRAME/TIMESLOT attributes
#define  TSCH_LINK_SELECTOR_ENABLED 1
// 2 - enbles this attributes for received packets.
//      it costs more memory for receiving packets atributes.
#define  TSCH_LINK_SELECTOR_ENABLEDRX 2

/* A custom feature allowing upper layers to assign packets to
 * a specific slotframe and link */
#ifdef TSCH_CONF_WITH_LINK_SELECTOR
#define TSCH_WITH_LINK_SELECTOR TSCH_CONF_WITH_LINK_SELECTOR
#else /* TSCH_CONF_WITH_LINK_SELECTOR */
#define TSCH_WITH_LINK_SELECTOR (BUILD_WITH_ORCHESTRA)
#endif /* TSCH_CONF_WITH_LINK_SELECTOR */

/* Configurable link comparator in case multiple links are scheduled at the same slot */
#ifdef TSCH_CONF_LINK_COMPARATOR
#define TSCH_LINK_COMPARATOR TSCH_CONF_LINK_COMPARATOR
#else
#define TSCH_LINK_COMPARATOR(a, b) default_tsch_link_comparator(a, b)
#endif

/* sheduling policy*/
// Enable skip slots that have ho xfer activity - options RX or TX are disabled
#define TSCH_SCHEDULE_OMMIT_NOXFER    1

/* shedule policies set */
#ifdef TSCH_SCHEDULE_CONF_POLICY
#define TSCH_SCHEDULE_POLICY TSCH_SCHEDULE_CONF_POLICY
#else
#define TSCH_SCHEDULE_POLICY 0
#endif

/******** Configuration: CSMA *******/

/* TSCH CSMA-CA parameters, see IEEE 802.15.4e-2012 */

/* Min backoff exponent */
#ifdef TSCH_CONF_MAC_MIN_BE
#define TSCH_MAC_MIN_BE TSCH_CONF_MAC_MIN_BE
#else
#define TSCH_MAC_MIN_BE 1
#endif

/* Max backoff exponent */
#ifdef TSCH_CONF_MAC_MAX_BE
#define TSCH_MAC_MAX_BE TSCH_CONF_MAC_MAX_BE
#else
#define TSCH_MAC_MAX_BE 5
#endif

/* Max number of re-transmissions */
#ifdef TSCH_CONF_MAC_MAX_FRAME_RETRIES
#define TSCH_MAC_MAX_FRAME_RETRIES TSCH_CONF_MAC_MAX_FRAME_RETRIES
#else
#define TSCH_MAC_MAX_FRAME_RETRIES 7
#endif

/* Include source address in ACK? */
#ifdef TSCH_PACKET_CONF_EACK_WITH_SRC_ADDR
#define TSCH_PACKET_EACK_WITH_SRC_ADDR TSCH_PACKET_CONF_EACK_WITH_SRC_ADDR
#else
#define TSCH_PACKET_EACK_WITH_SRC_ADDR 0
#endif

/* Perform CCA before sending? */
#ifdef TSCH_CONF_CCA_ENABLED
#define TSCH_CCA_ENABLED TSCH_CONF_CCA_ENABLED
#else
#define TSCH_CCA_ENABLED 0
#endif

/* Include destination address in ACK? */
#ifdef TSCH_PACKET_CONF_EACK_WITH_DEST_ADDR
#define TSCH_PACKET_EACK_WITH_DEST_ADDR TSCH_PACKET_CONF_EACK_WITH_DEST_ADDR
#else
#define TSCH_PACKET_EACK_WITH_DEST_ADDR 1 /* Include destination address
by default, useful in case of duplicate seqno */
#endif

/* Estimate possible looses fo timesource EB. need to prevent timesync loose
 * when used EB slot with option LINK_OPTION_TIME_EB_ESCAPE
 * \sa LINK_OPTION_TIME_EB_ESCAPE
 * */
#ifdef TSCH_CONF_TIMESYNC_EB_LOOSES
#define TSCH_TIMESYNC_EB_LOOSES TSCH_CONF_TIMESYNC_EB_LOOSES
#else
//#define TSCH_TIMESYNC_EB_LOOSES 5
#define TSCH_TIMESYNC_EB_LOOSES TSCH_MAC_MAX_FRAME_RETRIES
#endif

/* phantom TSCH adress, if != eb_adress, binds width adress declared in links
 * this allows use different queues on same receiver adress,
 * denoted to different links
 * */
#ifdef TSCH_CONF_WITH_PHANTOM_NBR
#define TSCH_WITH_PHANTOM_NBR TSCH_CONF_WITH_PHANTOM_NBR
#else
#define TSCH_WITH_PHANTOM_NBR  0
#endif /* TSCH_CONF_EB_AUTOSELECT */

//* Initiates sequence no of packets from RTclock at strtup.
//* this should help to pass though duplicates filter when fast
//* chip restarts.
#ifndef TSCH_CONF_SEQ_FROMRT
#define TSCH_CONF_SEQ_FROMRT 0
#endif

/* TSCH send/receive events polling style.
 * */
//< tsch events are executed all in one cycle. Simpler code.
#define TSCH_POLLING_STRONG    0
//< tsch events are executed yielding by pause. Gives better cooperative
//      it still enSUREs TX events before RX
#define TSCH_POLLING_RELAXED   1
#ifdef TSCH_CONF_POLLING_STYLE
#define TSCH_POLLING_STYLE TSCH_CONF_POLLING_STYLE
#else
#define TSCH_POLLING_STYLE 0
#endif

/******** Configuration: hardware-specific settings *******/

/* HW frame filtering enabled */
#ifdef TSCH_CONF_HW_FRAME_FILTERING
#define TSCH_HW_FRAME_FILTERING TSCH_CONF_HW_FRAME_FILTERING
#else /* TSCH_CONF_HW_FRAME_FILTERING */
// at present TSCH not use ADRESS filtering.
#define TSCH_HW_FRAME_FILTERING 0
#endif /* TSCH_CONF_HW_FRAME_FILTERING */

/* HW issues, that TSCH should handle  */
// denotes that radio can generate spurous RX events, or frames, while wait received
//  packet. and this packets should be droped while last readen packet read.
#define TSCH_HW_FEATURE_SPUROUS_RX  1
// denotes that radio receive can be reset by TransmitPower reset. Else use
//   ordinar radio.off
#define TSCH_HW_FEATURE_BREAK_BY_POWER  2
// denotes use radio.pending() as test for received packet.
//      helpful when radio.receiving is not relyable
#define TSCH_HW_FEATURE_RECV_BY_PENDING 4
#ifdef TSCH_CONF_HW_FEATURE
#define TSCH_HW_FEATURE TSCH_CONF_HW_FEATURE
#else /* TSCH_CONF_HW_FEATURE */
#define TSCH_HW_FEATURE 0
#endif /* TSCH_CONF_HW_FEATURE */

/* Keep radio always on within TSCH timeslot (1) or turn it off between packet and ACK? (0) */
#ifdef TSCH_CONF_RADIO_ON_DURING_TIMESLOT
#define TSCH_RADIO_ON_DURING_TIMESLOT TSCH_CONF_RADIO_ON_DURING_TIMESLOT
#else
#define TSCH_RADIO_ON_DURING_TIMESLOT 0
#endif

/* This is level, below that received packets/ack s a droped as unreceived.
 * Useful for redio-sensitivity tests, or debug radio for long distances
 * */
//#define TSCH_RADIO_RSSI_TH_DBM 0

/* ACK signal use for RX statistics.
 * Useful for connections that are most sends, and rare receives, so only via
 *      receive ACK can estimate sended RSSI (suggests that itis simmetrical).
 * This should affects connection stats, andtherefore -> metricrs.
 * */
#ifdef TSCH_CONF_ACK_STATS
#define TSCH_ACK_STATS TSCH_CONF_ACK_STATS
#else
#define TSCH_ACK_STATS 0
#endif

/* ACK timing style:
 * \value 0 - default: old behaviour - where ACK at time position from estimated
 *              packet trasmition time.
 *              tx_duration(t) + TSCH_DEFAULT_TS_TX_ACK_DELAY
 * \value 1 - immediate ACK after receive + TSCH_DEFAULT_TS_TX_ACK_DELAY.
 *          rely on radio_driver behaviour - that blocks receive/transmit operation
 *          right for operation time.
 *  */
#define TSCH_ACK_TIMING_OLD         0
#define TSCH_ACK_TIMING_IMMEDIATE   1
#ifdef TSCH_CONF_ACK_TIMING_STYLE
#define TSCH_ACK_TIMING_STYLE TSCH_CONF_ACK_TIMING_STYLE
#elif CONTIKI_TARGET_COOJA || CONTIKI_TARGET_COOJA_IP64
// cooja default RTimer resolution poor
#   if defined(RTIMER_CONF_ARCH_SECOND) && (RTIMER_CONF_ARCH_SECOND > 2000)
#       define TSCH_ACK_TIMING_STYLE TSCH_ACK_TIMING_IMMEDIATE
#   else
#       define TSCH_ACK_TIMING_STYLE TSCH_ACK_TIMING_OLD
#   endif
#else
#define TSCH_ACK_TIMING_STYLE TSCH_ACK_TIMING_IMMEDIATE
#endif

/* TSCH timeslot Recive polling timeout [us]
 *      if > 0, receive waits compete by polling with defined period. Or IDLE waits for complete.
 *      This releases CPU from ISR to main program during receives
 * */
#ifdef TSCH_CONF_TIMING_POLL_RX_US
#define TSCH_TIMING_POLL_RX_US TSCH_CONF_TIMING_POLL_RX_US
#else
#define TSCH_TIMING_POLL_RX_US 0
#endif

/* TSCH timeslot timing template */
#ifdef TSCH_CONF_DEFAULT_TIMESLOT_TIMING
#define TSCH_DEFAULT_TIMESLOT_TIMING TSCH_CONF_DEFAULT_TIMESLOT_TIMING
#else
#define TSCH_DEFAULT_TIMESLOT_TIMING tsch_timeslot_timing_us_10000
#endif

/* Configurable Rx guard time is micro-seconds */
#ifndef TSCH_CONF_RX_WAIT
#define TSCH_CONF_RX_WAIT 2200
#endif /* TSCH_CONF_RX_WAIT */

/* tsch_radiodelay_prefetch_tx provide distinguish for 2 slot layouts:
 * A) Simple detection - with prefetched by BEFORE_TX transmitionstart:
 *                                                 |Txoffs
 *    |<----< CCA > ->Tx|<---BEFORE_TX|------------|------------
 *                                    |    RX  guard window     |
 *    |<-----------|Rxofs-------------|<- RX wait/2|RX wait/2 ->|
 *                 ^                                 received?
 *                RX_offset allows BEFORE_TX+RX wait/2
 *            and CCA allows BEFORE_TX
 *
 * B) Early detection - when have no prefetch time, use 2 step detecting:
 *         by chanel_clear in RXguarding, and by received() check after it
 *                               Tx|Txoffs
 *    |<----< CCA > <--------------|----BEFORE_TX-------->|
 *                    |      RX  guard window   |
 *    |<------|Rxofs--|<- RX wait/2|RX wait/2 ->|-------->|-------
 *                 ^                chanel_clear?      received?
 *                RX_offset allows BEFORE_TX+RX wait/2
 *
 *
 * */
//< use RADIO_DELAY_BEFORE_TX for prefetching (default style)
#define TSCH_RADIODELAY_PREFETCH_TX_BEFORE  1
//< no prefetching, use early detection
#define TSCH_RADIODELAY_PREFETCH_TX_NO      0
//< style of prefetching detects from TSCH slot layout at tsch starting
#define TSCH_RADIODELAY_PREFETCH_TX_EVAL   -1

//#define TSCH_RADIODELAY_PREFETCH_TX TSCH_RADIODELAY_PREFETCH_TX_BEFORE

/* Configurable guard time [us] for turn on radio, before slot activity */
#ifndef TSCH_CONF_RFON_GUARD_TIME
#define TSCH_CONF_RFON_GUARD_TIME 0
#endif

#endif /* __TSCH_CONF_H__ */
/** @} */
