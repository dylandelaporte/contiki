/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
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
 *         Packet buffer (packetbuf) management
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

/**
 * \addtogroup packetbuf
 * @{
 */

#include <string.h>

#include "contiki-net.h"
#include "net/packetbuf.h"
#include "net/rime/rime.h"
#include "sys/cc.h"
#include "packetbuf.h"

struct packetbuf_attr packetbuf_attrs[PACKETBUF_NUM_ATTRS];
struct packetbuf_addr packetbuf_addrs[PACKETBUF_NUM_ADDRS];


//    [   packet buf                                   ]
//    [                   |      origin data packet    ]
//    [ <header allocate  | hdr remove >|  rest data   ]
//    ^                   ^             ^              ^
//    packetbuf      +hlen|            data     +buflen|
//    |                                            +len|
uint16_t packetbuf_buflen;
uint8_t  packetbuf_hlen;
#define buflen  packetbuf_buflen
#define hdrlen  packetbuf_hlen

/* The declarations below ensure that the packet buffer is aligned on
   an even 32-bit boundary. On some platforms (most notably the
   msp430 or OpenRISC), having a potentially misaligned packet buffer may lead to
   problems when accessing words. */
uint32_t packetbuf_aligned[(PACKETBUF_SIZE + 3) / 4];
uint8_t *packetbuf_store = (uint8_t *)packetbuf_aligned;
#define packetbuf   ((uint8_t *)packetbuf_aligned)

uint8_t*      packetbuf_data;
uint_fast16_t packetbuf_len;

#undef DEBUG
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/
void
packetbuf_clear(void)
{
  //bufptr = 0;
  hdrlen = 0;
  buflen = 0;
  packetbuf_len = 0;
  packetbuf_data = packetbuf;

  packetbuf_attr_clear();
}
/*---------------------------------------------------------------------------*/
int
packetbuf_copyfrom(const void *from, uint16_t len)
{
  uint16_t L;

  packetbuf_clear();
  L = MIN(PACKETBUF_SIZE, len);
  memcpy(packetbuf, from, L);
  buflen = L;
  packetbuf_len = L;
  return L;
}
/*---------------------------------------------------------------------------*/
int
packetbuf_copyto(void *to)
{
  if(hdrlen + buflen > PACKETBUF_SIZE) {
    return 0;
  }
  memcpy(to, packetbuf_hdrptr(), hdrlen);
  memcpy((uint8_t *)to + hdrlen, packetbuf_dataptr(), buflen);
  return hdrlen + buflen;
}
/*---------------------------------------------------------------------------*/
int
packetbuf_hdralloc(int size)
{
  if(size + packetbuf_totlen() > PACKETBUF_SIZE) {
    return 0;
  }

  /* shift data to the right */
  memmove(&packetbuf[size], packetbuf, packetbuf_totlen());
  hdrlen += size;
  packetbuf_data += size;
  packetbuf_len += size;
  return 1;
}
/*---------------------------------------------------------------------------*/
int
packetbuf_hdrreduce(int size)
{
  if(buflen < size) {
    return 0;
  }

  packetbuf_data += size;
  buflen -= size;
  return 1;
}
/*---------------------------------------------------------------------------*/
void packetbuf_compact(void)
{
  int bufptr = packetbuf_hdrlen() - hdrlen;
  if(bufptr > 0) {
    /* shift data to the left */
    memmove(&packetbuf[hdrlen], packetbuf_data, buflen);
    packetbuf_data = &packetbuf[hdrlen];
    packetbuf_len -= bufptr;
  }
}
/*---------------------------------------------------------------------------*/
void packetbuf_set_datalen(unsigned len)
{
  PRINTF("packetbuf_set_len: len %d\n", len);
  buflen = len;
  packetbuf_len = packetbuf_hdrlen() + len;
}
/*---------------------------------------------------------------------------*/
#if PACKETBUF_CONF_ATTRS_INLINE < PACKETBUF_ATTRS_INLINE_FAST
uint8_t
packetbuf_hdrlen(void)
{
  return (packetbuf_data-packetbuf); //-bufptr;
  //return bufptr + hdrlen;
}
#endif
/*---------------------------------------------------------------------------*/
uint16_t
packetbuf_remaininglen(void)
{
  return PACKETBUF_SIZE - packetbuf_totlen();
}
/*---------------------------------------------------------------------------*/
void
packetbuf_attr_clear(void)
{
  int i;
  memset(packetbuf_attrs, 0, sizeof(packetbuf_attrs));
  for(i = 0; i < PACKETBUF_NUM_ADDRS; ++i) {
    linkaddr_copy(&packetbuf_addrs[i].addr, &linkaddr_null);
  }
#if TSCH_WITH_LINK_SELECTOR
  packetbuf_linksel_clear();
#endif
}
/*---------------------------------------------------------------------------*/
void
packetbuf_attr_copyto(struct packetbuf_attr *attrs,
                      struct packetbuf_addr *addrs)
{
  memcpy(attrs, packetbuf_attrs, sizeof(packetbuf_attrs));
  memcpy(addrs, packetbuf_addrs, sizeof(packetbuf_addrs));
}
/*---------------------------------------------------------------------------*/
void
packetbuf_attr_copyfrom(struct packetbuf_attr *attrs,
                        struct packetbuf_addr *addrs)
{
  memcpy(packetbuf_attrs, attrs, sizeof(packetbuf_attrs));
  memcpy(packetbuf_addrs, addrs, sizeof(packetbuf_addrs));
}
/*---------------------------------------------------------------------------*/
#if !PACKETBUF_CONF_ATTRS_INLINE
int
packetbuf_set_attr(uint8_t type, const packetbuf_attr_t val)
{
  packetbuf_attrs[type].val = val;
  return 1;
}
/*---------------------------------------------------------------------------*/
packetbuf_attr_t
packetbuf_attr(uint8_t type)
{
  return packetbuf_attrs[type].val;
}
/*---------------------------------------------------------------------------*/
const linkaddr_t *
packetbuf_addr(uint8_t type)
{
  return &packetbuf_addrs[type - PACKETBUF_ADDR_FIRST].addr;
}
#endif /* PACKETBUF_CONF_ATTRS_INLINE */
/*---------------------------------------------------------------------------*/
int
packetbuf_set_addr(uint8_t type, const linkaddr_t *addr)
{
  linkaddr_copy(&packetbuf_addrs[type - PACKETBUF_ADDR_FIRST].addr, addr);
  return 1;
}
/*---------------------------------------------------------------------------*/
int
packetbuf_holds_broadcast(void)
{
  return linkaddr_cmp(&packetbuf_addrs[PACKETBUF_ADDR_RECEIVER - PACKETBUF_ADDR_FIRST].addr, &linkaddr_null);
}
/*---------------------------------------------------------------------------*/
#if TSCH_WITH_LINK_SELECTOR
void packetbuf_set_linksel(uint16_t  sfh, uint16_t  slot, uint16_t  choffs){
    packetbuf_set_attr(PACKETBUF_ATTR_TSCH_SLOTFRAME        , sfh );
    packetbuf_set_attr(PACKETBUF_ATTR_TSCH_TIMESLOT         , slot );
    packetbuf_set_attr(PACKETBUF_ATTR_TSCH_CHANNEL_OFFSET   , choffs);
}

void packetbuf_linksel_set(const packetbuf_linkselector val){
    packetbuf_set_linksel(val.sfh, val.slot, val.choffs);
}

void packetbuf_linksel_clear(const packetbuf_linkselector val){
    packetbuf_set_linksel(0xffff, 0xffff, 0xffff);
}

packetbuf_linkselector packetbuf_linksel(){
    packetbuf_linkselector save;
    save.sfh        = packetbuf_attr(PACKETBUF_ATTR_TSCH_SLOTFRAME);
    save.slot       = packetbuf_attr(PACKETBUF_ATTR_TSCH_TIMESLOT);
    save.choffs     = packetbuf_attr(PACKETBUF_ATTR_TSCH_CHANNEL_OFFSET);
    return save;
}

#endif

/** @} */
