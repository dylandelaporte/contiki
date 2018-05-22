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

/******** Configuration *******/

/* Default IEEE 802.15.4e hopping sequences, obtained from https://gist.github.com/twatteyne/2e22ee3c1a802b685695 */
/* 16 channels, sequence length 16 */
#define TSCH_HOPPING_SEQUENCE_16_16 (uint8_t[]){ 16, 17, 23, 18, 26, 15, 25, 22, 19, 11, 12, 13, 24, 14, 20, 21 }
/* 4 channels, sequence length 16 */
#define TSCH_HOPPING_SEQUENCE_4_16 (uint8_t[]){ 20, 26, 25, 26, 15, 15, 25, 20, 26, 15, 26, 25, 20, 15, 20, 25 }
/* 4 channels, sequence length 4 */
#define TSCH_HOPPING_SEQUENCE_4_4 (uint8_t[]){ 15, 25, 26, 20 }
/* 2 channels, sequence length 2 */
#define TSCH_HOPPING_SEQUENCE_2_2 (uint8_t[]){ 20, 25 }
/* 1 channel, sequence length 1 */
#define TSCH_HOPPING_SEQUENCE_1_1 (uint8_t[]){ 20 }

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
#define TSCH_HOPPING_SEQUENCE_MAX_LEN 16
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


/* Timeslot timing */

#ifndef TSCH_CONF_DEFAULT_TIMESLOT_LENGTH
#define TSCH_CONF_DEFAULT_TIMESLOT_LENGTH 10000
#endif /* TSCH_CONF_DEFAULT_TIMESLOT_LENGTH */

/* Configurable Rx guard time is micro-seconds */
#ifndef TSCH_CONF_RX_WAIT
#define TSCH_CONF_RX_WAIT 2200
#endif /* TSCH_CONF_RX_WAIT */

/* Configurable guard time [us] for turn on radio, before slot activity */
#ifndef TSCH_CONF_RFON_GUARD_TIME
#define TSCH_CONF_RFON_GUARD_TIME 0
#endif /* TSCH_CONF_RX_WAIT */


#ifndef TSCH_CONF_TS_MAX_TX
#define TSCH_CONF_TS_MAX_TX 4256
#endif /* TSCH_CONF_RX_WAIT */

#ifndef TSCH_CONF_TS_MAX_ACK
#define TSCH_CONF_TS_MAX_ACK 2400
#endif /* TSCH_CONF_RX_WAIT */

/* The default timeslot timing in the standard is a guard time of
 * 2200 us, a Tx offset of 2120 us and a Rx offset of 1120 us.
 * As a result, the listening device has a guard time not centered
 * on the expected Tx time. This is to be fixed in the next iteration
 * of the standard. This can be enabled with:
 * #define TSCH_DEFAULT_TS_TX_OFFSET          2120
 * #define TSCH_DEFAULT_TS_RX_OFFSET          1120
 * #define TSCH_DEFAULT_TS_RX_WAIT            2200
 *
 * Instead, we align the Rx guard time on expected Tx time. The Rx
 * guard time is user-configurable with TSCH_CONF_RX_WAIT.

 * (TS_TX_OFFSET - (TS_RX_WAIT / 2)) instead */

#if TSCH_CONF_DEFAULT_TIMESLOT_LENGTH == 10000
/* Default timeslot timing as per IEEE 802.15.4e */

#define TSCH_DEFAULT_TS_CCA_OFFSET         1800
#define TSCH_DEFAULT_TS_CCA                128
#define TSCH_DEFAULT_TS_TX_OFFSET          2120
#define TSCH_DEFAULT_TS_RX_OFFSET          (TSCH_DEFAULT_TS_TX_OFFSET - (TSCH_CONF_RX_WAIT / 2))
#define TSCH_DEFAULT_TS_RX_ACK_DELAY       800
#define TSCH_DEFAULT_TS_TX_ACK_DELAY       1000
#define TSCH_DEFAULT_TS_RX_WAIT            TSCH_CONF_RX_WAIT
#define TSCH_DEFAULT_TS_ACK_WAIT           400
#define TSCH_DEFAULT_TS_RX_TX              192
#define TSCH_DEFAULT_TS_MAX_ACK            2400
#define TSCH_DEFAULT_TS_MAX_TX             TSCH_CONF_TS_MAX_TX
#define TSCH_DEFAULT_TS_TIMESLOT_LENGTH    10000

#elif TSCH_CONF_DEFAULT_TIMESLOT_LENGTH == 15000
/* Default timeslot timing for platforms requiring 15ms slots */

#define TSCH_DEFAULT_TS_CCA_OFFSET         1800
#define TSCH_DEFAULT_TS_CCA                128
#define TSCH_DEFAULT_TS_TX_OFFSET          4000
#define TSCH_DEFAULT_TS_RX_OFFSET          (TSCH_DEFAULT_TS_TX_OFFSET - (TSCH_CONF_RX_WAIT / 2))
#define TSCH_DEFAULT_TS_TX_ACK_DELAY       4000
#define TSCH_DEFAULT_TS_RX_WAIT            TSCH_CONF_RX_WAIT
#define TSCH_DEFAULT_TS_ACK_WAIT           800
#define TSCH_DEFAULT_TS_RX_ACK_DELAY       (TSCH_DEFAULT_TS_TX_ACK_DELAY - (TSCH_DEFAULT_TS_ACK_WAIT/2))
#define TSCH_DEFAULT_TS_RX_TX              2072
#define TSCH_DEFAULT_TS_MAX_ACK            TSCH_CONF_TS_MAX_ACK
#define TSCH_DEFAULT_TS_MAX_TX             TSCH_CONF_TS_MAX_TX
#define TSCH_DEFAULT_TS_TIMESLOT_LENGTH    15000

#elif TSCH_CONF_DEFAULT_TIMESLOT_LENGTH == 65000U
/* 65ms timeslot, i.e. nearly the max length allowed by standard (16-bit unsigned in micro-seconds).
 * Useful for running link-layer security on sky or z1 in Cooja, where only S/W security is supported.
 * Note: this slot timing would require a total of 120ms. If a slot overlaps with the next active slot,
 * the latter will be skipped.
 * This configuration is mostly a work-around to test link-layer security in Cooja, it is recommended
 * to use it with a 6TiSCH minimal schedule of length >= 2. */

#define TSCH_DEFAULT_TS_CCA_OFFSET         1800
#define TSCH_DEFAULT_TS_CCA                128
#define TSCH_DEFAULT_TS_TX_OFFSET          52000
#define TSCH_DEFAULT_TS_RX_OFFSET          (TSCH_DEFAULT_TS_TX_OFFSET - (TSCH_CONF_RX_WAIT / 2))
#define TSCH_DEFAULT_TS_RX_ACK_DELAY       58600
#define TSCH_DEFAULT_TS_TX_ACK_DELAY       59000
#define TSCH_DEFAULT_TS_RX_WAIT            TSCH_CONF_RX_WAIT
#define TSCH_DEFAULT_TS_ACK_WAIT           800
#define TSCH_DEFAULT_TS_RX_TX              2072
#define TSCH_DEFAULT_TS_MAX_ACK            2400
#define TSCH_DEFAULT_TS_MAX_TX             TSCH_CONF_TS_MAX_TX
#define TSCH_DEFAULT_TS_TIMESLOT_LENGTH    65000

#else
//* should be user defined slot timing
#endif

/* A custom feature allowing upper layers to assign packets to
 * a specific slotframe and link */
// 1 - enables PACKETBUF_ATTR_TSCH_SLOTFRAME/TIMESLOT attributes
#define  TSCH_LINK_SELECTOR_ENABLED 1
// 2 - enbles this attributes for received packets
#define  TSCH_LINK_SELECTOR_ENABLEDRX 2

#ifdef TSCH_CONF_WITH_LINK_SELECTOR
#define TSCH_WITH_LINK_SELECTOR TSCH_CONF_WITH_LINK_SELECTOR
#else /* TSCH_CONF_WITH_LINK_SELECTOR */
#define TSCH_WITH_LINK_SELECTOR 0
#endif /* TSCH_CONF_WITH_LINK_SELECTOR */

/* Estimate the drift of the time-source neighbor and compensate for it? */
#ifdef TSCH_CONF_ADAPTIVE_TIMESYNC
#define TSCH_ADAPTIVE_TIMESYNC TSCH_CONF_ADAPTIVE_TIMESYNC
#else
#define TSCH_ADAPTIVE_TIMESYNC 1
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

/* HW frame filtering enabled */
#ifdef TSCH_CONF_HW_FRAME_FILTERING
#define TSCH_HW_FRAME_FILTERING TSCH_CONF_HW_FRAME_FILTERING
#else /* TSCH_CONF_HW_FRAME_FILTERING */
#define TSCH_HW_FRAME_FILTERING 1
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
#else
#define TSCH_ACK_TIMING_STYLE TSCH_ACK_TIMING_IMMEDIATE
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


#endif /* __TSCH_CONF_H__ */
