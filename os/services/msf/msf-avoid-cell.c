/*
 * Copyright (c) 2020, Alexrayne.
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
 *         MSF Avoiding Cell APIs
 * \author
 *         alexrayne <alexraynepe196@gmail.com>
 */

#include "contiki.h"
#include <stdint.h>
#include "assert.h"

#include "lib/random.h"
#include "net/linkaddr.h"
#include "net/mac/tsch/tsch.h"

#include "msf-conf.h"
#include "msf-avoid-cell.h"

#include "sys/log.h"
#define LOG_MODULE "MSF void"
#define LOG_LEVEL LOG_LEVEL_MSF

#ifndef MSF_USED_LIST_LIMIT
#ifdef TSCH_JOIN_HOPPING_SEQUENCE_SIZE
#define MSF_USED_LIST_LIMIT (MSF_SLOTFRAME_LENGTH*TSCH_JOIN_HOPPING_SEQUENCE_SIZE())
#else
#define MSF_USED_LIST_LIMIT (MSF_SLOTFRAME_LENGTH*TSCH_HOPPING_SEQUENCE_MAX_LEN)
#endif
#endif

unsigned                avoids_list_num = 0;
msf_cell_t              avoids_list[MSF_USED_LIST_LIMIT] = {0};
const tsch_neighbor_t*  avoids_nbrs[MSF_USED_LIST_LIMIT];

// special codes for cells.raw
enum {
    //< cell ffff.ffff - no value, use for deleted cells
    cellFREE = ~0ul,

    // this is trigegr value to cleanup free values from nouse list
    cellFREE_COUNT_TRIGGER = 8
};

unsigned    nouse_free_count = 0;
static void msf_unuse_cleanup();
static int msf_avoids_cell_idx(msf_cell_t x);
static int msf_avoids_nbr_cell_idx(msf_cell_t x, const tsch_neighbor_t *n);

static
tsch_neighbor_t* get_addr_nbr(const linkaddr_t *addr){
    if (addr == NULL)
        return NULL;
    if (linkaddr_cmp(addr,&linkaddr_node_addr))
        return NULL;
    //if (linkaddr_cmp(addr,&tsch_broadcast_address))
    //    return NULL;
    return tsch_queue_get_nbr(addr);
}


int msf_is_avoid_cell(msf_cell_t x){
    return msf_avoids_cell_idx(x) >= 0;
}

int  msf_is_avoid_cell_at(uint16_t slot_offset, uint16_t channel_offset){
    msf_cell_t x;
    x.field.slot    = slot_offset;
    x.field.chanel  = channel_offset;
    return msf_is_avoid_cell(x);
}

int msf_is_avoid_cell_from(msf_cell_t x, const linkaddr_t *peer_addr){
    if (peer_addr == NULL)
        return msf_is_avoid_cell(x);
    else
        return msf_is_avoid_nbr_cell(x, get_addr_nbr(peer_addr));
}

int  msf_is_avoid_nbr_cell(msf_cell_t x, const tsch_neighbor_t *n){
    return msf_avoids_nbr_cell_idx(x, n) >= 0;
}

static
int msf_avoids_cell_idx(msf_cell_t x){
    msf_cell_t* cell = avoids_list;
    for (unsigned idx = avoids_list_num; idx > 0; --idx, cell++){
        if (cell->raw == x.raw)
            return cell-avoids_list;
    }
    return -1;
}

msf_chanel_mask_t msf_avoid_slot_chanels(uint16_t slot_offset){
    msf_chanel_mask_t res = 0;
    msf_cell_t* cell = avoids_list;
    const unsigned bitnmsk = ((sizeof(msf_chanel_mask_t)<<3) -1);

    for (unsigned idx = avoids_list_num; idx > 0; --idx, cell++){
        if (cell->field.slot == slot_offset){
#ifdef TSCH_JOIN_HOPPING_SEQUENCE_SIZE
            unsigned ch = cell->field.chanel % TSCH_JOIN_HOPPING_SEQUENCE_SIZE();
#else
            unsigned ch = TSCH_ASN_MOD(cell->field.chanel, tsch_hopping_sequence_length);
#endif
            res |= 1 << (ch & bitnmsk);
        }
    }
    return res;
}


static
int msf_avoids_nbr_cell_idx(msf_cell_t x, const tsch_neighbor_t *n){
    msf_cell_t* cell = avoids_list;
    for (unsigned idx = 0; idx < avoids_list_num; ++idx, cell++){
        if (cell->raw == x.raw){
            if (avoids_nbrs[idx] == n)
                return idx;
        }
    }
    return -1;
}

int  msf_is_avoid_slot(uint16_t slot_offset){
    msf_cell_t* cell = avoids_list;
    for (unsigned idx = avoids_list_num; idx > 0; --idx, cell++){
        if (cell->field.slot == slot_offset)
            return 1;
    }
    return 0;
}

int  msf_is_avoid_nbr_slot(uint16_t slot_offset, const tsch_neighbor_t *n){
    msf_cell_t* cell = avoids_list;
    for (unsigned idx = 0; idx < avoids_list_num; ++idx, cell++){
        if (cell->field.slot == slot_offset){
            if (avoids_nbrs[idx] == n)
                return 1;
        }
    }
    return 0;
}



void msf_avoid_link_cell(const tsch_link_t* x){
    msf_avoid_nbr_cell(msf_cell_of_link(x), NULL);
}

void msf_avoid_cell(msf_cell_t x){
    msf_avoid_nbr_cell(x, NULL);
}

void msf_avoid_nbr_cell(msf_cell_t x, const tsch_neighbor_t *n){
    if (msf_is_avoid_nbr_cell(x, n))
        return;
    if (avoids_list_num < MSF_USED_LIST_LIMIT){
        LOG_DBG("avoid %u+%u ->", (unsigned)(x.field.slot), (unsigned)(x.field.chanel));
        LOG_DBG_LLADDR( tsch_queue_get_nbr_address(n) );
        LOG_DBG_("\n");
        avoids_list[avoids_list_num] = x;
        avoids_nbrs[avoids_list_num] = n;
        ++avoids_list_num;
    }
    else {
        LOG_WARN("rich limit reserve for cell %x\n", x.raw);
    }
}

void msf_unvoid_link_cell(const tsch_link_t* x){
    msf_unvoid_nbr_cell(msf_cell_of_link(x), NULL);
}

void msf_unvoid_cell(msf_cell_t x){
    msf_unvoid_nbr_cell(x, NULL);
}

void msf_unvoid_nbr_cell(msf_cell_t x, const tsch_neighbor_t *n){
    int idx = msf_avoids_nbr_cell_idx(x, n);
    if (idx >= 0){
        msf_cell_t* cell = avoids_list + idx;
        LOG_DBG("free %u+%u ->", (unsigned)(cell->field.slot), (unsigned)(cell->field.chanel));
        LOG_DBG_LLADDR( tsch_queue_get_nbr_address(avoids_nbrs[idx]) );
        LOG_DBG_("\n");
        cell->raw = cellFREE;
        avoids_nbrs[idx]     = NULL;
        nouse_free_count++;
        if (nouse_free_count >= cellFREE_COUNT_TRIGGER){
            msf_unuse_cleanup();
        }
    }
}

void msf_unvoid_all_cells(){
    LOG_DBG("unvoid all\n");
    // protect own slot0 - since it is for EB
    avoids_list[0].raw = 0;
    avoids_nbrs[0]     = NULL;

    avoids_list_num  = 1;
    nouse_free_count = 0;
}


void msf_unvoid_nbr_cells(const tsch_neighbor_t* n){
    LOG_DBG("unvoid %p\n", n);

    msf_cell_t*             cells= avoids_list;
    const tsch_neighbor_t** nbrs = avoids_nbrs;
    for (unsigned idx = 0; idx < avoids_list_num; ++idx){
        if (nbrs[idx] != n)
            continue;

        if (cells[idx].raw != cellFREE)
            ++nouse_free_count;
        LOG_DBG("free %u+%u\n", (unsigned)cells[idx].field.slot, (unsigned)cells[idx].field.chanel);
        cells[idx].raw = cellFREE;
    }
    if (nouse_free_count >= cellFREE_COUNT_TRIGGER){
        msf_unuse_cleanup();
    }
}

static
void msf_unuse_cleanup(){
    LOG_DBG("cleanup for %u\n", nouse_free_count);

    msf_cell_t*             cell = avoids_list;
    const tsch_neighbor_t** nbrs = avoids_nbrs;

    //drop tail free cells
    for (int idx = avoids_list_num-1; idx >= 0; --idx){
        if (cell[idx].raw != cellFREE)
            break;
        avoids_list_num = idx;
    }

    for (int idx = avoids_list_num-1; idx >= 0; --idx){
        if (cell[idx].raw != cellFREE)
            continue;
        int k = idx;
        for (; k > 0; --k){
            if (cell[-1].raw != cellFREE)
                break;
        }
        memmove(cell+k, cell+idx, sizeof(cell[0])*(avoids_list_num-idx) );
        memmove(nbrs+k, nbrs+idx, sizeof(nbrs[0])*(avoids_list_num-idx) );
        avoids_list_num -= (idx-k);
        idx = k-2;
    }
    nouse_free_count = 0;
}

//@return - amount of avoid cells
int msf_avoid_num_cells(){
    int res = 0;
    msf_cell_t* cell = avoids_list;
    for (unsigned idx = 0; idx < avoids_list_num; ++idx, cell++){
        if (avoids_nbrs[idx] != NULL)
            continue;
        if (cell->raw != cellFREE)
            ++res;
    }
    return res;
}

// @result - cells->head.num_cells= amount of filled cells
// @return - >0 - amount of cells append
// @return - =0 - no cells to enumerate
// @return - <0 - no cells append, not room to <cells>
int msf_avoid_enum_cells(SIXPCellsPkt* pkt, unsigned limit){
    if ( pkt->head.num_cells >= limit)
        return -1;

    int res = 0;
    msf_cell_t* cell = avoids_list;

    for (unsigned idx = 0; idx < avoids_list_num; ++idx, cell++){
        if (avoids_nbrs[idx] != NULL)
            continue;
        if (cell->raw != cellFREE){
            if ( pkt->head.num_cells < limit){
                msf_cell_t* x = pkt->cells + pkt->head.num_cells;
                x->field.slot   = cell->field.slot;
                x->field.chanel = cell->field.chanel;
                ++res;
                ++pkt->head.num_cells;
                if (pkt->head.num_cells >= limit)
                    return res;
            }
        }
    }
    return res;
}
