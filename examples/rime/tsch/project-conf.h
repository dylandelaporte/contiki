/*
 * Copyright (c) 2016, Inria.
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
 *         Project config file
 * \author
 *         Simon Duquennoy <simon.duquennoy@inria.fr>
 *
 */

#ifndef __PROJECT_CONF_H__
#define __PROJECT_CONF_H__

/* Set to enable TSCH security */
#ifndef WITH_SECURITY
#define WITH_SECURITY 0
#endif /* WITH_SECURITY */

/* USB serial takes space, free more space elsewhere */
#define SICSLOWPAN_CONF_FRAG 0
#define UIP_CONF_BUFFER_SIZE 160

// provide buffer for log outpu. cc13 have buffered uart
#define UART_TXBUFSIZE  4096
//#define UART_TXBUF_OVERMARK '*'

//#define LPM_MODE_MAX_SUPPORTED_CONF LPM_MODE_SLEEP
#define LPM_MODE_MAX_SUPPORTED_CONF LPM_MODE_AWAKE
#define CONTIKI_WATCHDOG_CONF       CONTIKI_WATCHDOG_OFF

// disable useless shell commands
#define SHELL_COMMANDS_CONF_PING        0
#define SHELL_COMMANDS_CONF_SETROOT     0
#define SHELL_COMMANDS_CONF_TSCH_SCHEDULE   0

#ifndef LINKADDR_CONF_SIZE
#if NETSTACK_CONF_WITH_RIME
#define LINKADDR_CONF_SIZE 2
#endif
#endif
/*******************************************************/
/******************* Configure RF ********************/
/*******************************************************/
// for 433MHz use RF band 431-527
#define DOT_15_4G_CONF_FREQUENCY_BAND_ID    DOT_15_4G_FREQUENCY_BAND_431
#define IEEE802154_CONF_DEFAULT_CHANNEL     14

/*******************************************************/

/* Netstack layers */
#undef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC     tschmac_driver
#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     nordc_driver
#undef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER  framer_802154

/* IEEE802.15.4 frame version */
//#undef FRAME802154_CONF_VERSION
//#define FRAME802154_CONF_VERSION FRAME802154_IEEE802154E_2012

//#undef TSCH_CONF_AUTOSELECT_TIME_SOURCE
//#define TSCH_CONF_AUTOSELECT_TIME_SOURCE 1

/* Needed for cc2420 platforms only */
/* Disable DCO calibration (uses timerB) */
#undef DCOSYNCH_CONF_ENABLED
#define DCOSYNCH_CONF_ENABLED            0
/* Enable SFD timestamps (uses timerB) */
#undef CC2420_CONF_SFD_TIMESTAMPS
#define CC2420_CONF_SFD_TIMESTAMPS       1

/* TSCH logging. 0: disabled. 1: basic log. 2: with delayed
 * log messages from interrupt */
#define LOG_CONF_LEVEL_IPV6                        LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_RPL                         LOG_LEVEL_INFO
#define LOG_CONF_LEVEL_6LOWPAN                     LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_TCPIP                       LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_FRAMER                      LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_MAC                         LOG_LEVEL_DBG
#define LOG_CONF_LEVEL_MAIN                        LOG_LEVEL_INFO
#define LOG_CONF_LEVEL_6TOP                         LOG_LEVEL_NONE
//#define LOG_LEVEL LOG_LEVEL_ORCHESRTA             LOG_LEVEL_DBG
#define LOG_CONF_LEVEL_MSF                          LOG_LEVEL_INFO
#define LOG_CONF_LEVEL_NET                          LOG_LEVEL_NONE

#define TSCH_LOG_CONF_PER_SLOT                     1
//#define TSCH_LOG_CONF_LEVEL                        2

/* IEEE802.15.4 PANID */
#undef IEEE802154_CONF_PANID
#define IEEE802154_CONF_PANID 0xabcd

/* Do not start TSCH at init, wait for NETSTACK_MAC.on() */
#undef TSCH_CONF_AUTOSTART
#define TSCH_CONF_AUTOSTART 0

/* 6TiSCH minimal schedule length.
 * Larger values result in less frequent active slots: reduces capacity and saves energy. */
#undef TSCH_SCHEDULE_CONF_DEFAULT_LENGTH
#define TSCH_SCHEDULE_CONF_DEFAULT_LENGTH 3

#define TSCH_CONF_TIMING_POLL_RX_US   100

#define TSCH_CONF_ASSOCIATION_POLL_FREQUENCY CLOCK_SECOND

#undef TSCH_CONF_DEFAULT_HOPPING_SEQUENCE
//#define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE TSCH_HOPPING_SEQUENCE_4_4
#define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE TSCH_HOPPING_SEQUENCE_1_1

#define TSCH_CONF_EB_PERIOD (5 * CLOCK_SECOND)


#endif /* __PROJECT_CONF_H__ */
