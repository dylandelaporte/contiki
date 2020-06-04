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
 * \addtogroup msf
 * @{
 */
/**
 * \file
 *         MSF Avoiding Cell APIs
 * \author
 *         alexrayne <alexraynepe196@gmail.com>
 */

#ifndef _MSF_AVOID_CELL_H_
#define _MSF_AVOID_CELL_H_

#include <stdint.h>

#include "net/linkaddr.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/sixtop/sixp-pkt.h"
#include "net/mac/tsch/sixtop/sixp-pkt-ex.h"

typedef sixp_cell_t msf_cell_t;

static inline
msf_cell_t msf_cell_of_link(const tsch_link_t* x){
    msf_cell_t cell;
    cell.field.slot    = x->timeslot;
    cell.field.chanel  = x->channel_offset;
    return cell;
}


// same as reset all
void msf_unvoid_all_cells();

void msf_avoid_cell(msf_cell_t x);
void msf_unvoid_cell(msf_cell_t x);

void msf_avoid_link_cell(const tsch_link_t* x);
void msf_unvoid_link_cell(const tsch_link_t* x);

int  msf_is_avoid_cell(msf_cell_t x);
int  msf_is_avoid_cell_at(uint16_t slot_offset, uint16_t channel_offset);
int  msf_is_avoid_slot(uint16_t slot_offset);

typedef int msf_chanel_mask_t;
msf_chanel_mask_t  msf_avoid_slot_chanels(uint16_t slot_offset);

void msf_avoid_nbr_cell(msf_cell_t x, const tsch_neighbor_t *n);
void msf_unvoid_nbr_cell(msf_cell_t x, const tsch_neighbor_t *n);
void msf_unvoid_nbr_cells(const tsch_neighbor_t* n);

int  msf_is_avoid_cell_from(msf_cell_t x, const linkaddr_t *peer_addr);
int  msf_is_avoid_nbr_cell(msf_cell_t x, const tsch_neighbor_t *n);
int  msf_is_avoid_nbr_slot(uint16_t slot_offset, const tsch_neighbor_t *n);

//@return - amount of avoid cells
int msf_avoid_num_cells();

// @result - cells->head.num_cells= amount of filled cells
// @return - >0 - amount of cells append
// @return - =0 - no cells to enumerate
// @return - <0 - no cells append, not room to <cells>
int msf_avoid_enum_cells(SIXPCellsPkt* cells, unsigned limit);



#endif /* _MSF_RESERVED_CELL_H_ */
/** @} */
