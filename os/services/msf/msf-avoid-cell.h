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

static inline
msf_cell_t msf_cell_at(uint16_t slot_offset, uint16_t channel_offset){
    msf_cell_t cell;
    cell.field.slot    = slot_offset;
    cell.field.chanel  = channel_offset;
    return cell;
}

enum AvoidOption{
    aoRX    = LINK_OPTION_RX,
    aoTX    = LINK_OPTION_TX,

    //cell is known by remotes
    aoDEFAULT       = 0x8,
    //mark used by NRSF to ident calls that are exposed to nbrs
    aoMARK          = 0x10,

    //cell is local used, and should be shared to nbrs
    aoUSE_LOCAL     = 0x20,
    aoUSE_REMOTE    = 0xC0,



    //cell is remote used
    aoUSE_REMOTE_1HOP   = 0x40,
    aoUSE_REMOTE_2HOP   = 0x80,
    aoUSE_REMOTE_3HOP   = 0xC0,

    aoUSE           = aoUSE_LOCAL | aoUSE_REMOTE,

    // this cell is should be removed. NRSF use it for enum cells to expose deleted
    aoDROPED        = 0,
};
typedef enum AvoidOption AvoidOption;

enum AvoidResult{
    arNEW = 1 ,             //< for new cell appends
    arEXIST_CHANGE = 0,     //< for updates present cell
    arEXIST_KEEP = -1       //< for keep unchanged present cell
};
typedef enum AvoidResult AvoidResult;

// same as reset all
void msf_unvoid_all_cells();

// locals avoid
AvoidResult msf_avoid_link_cell(const tsch_link_t* x);
void msf_unvoid_link_cell(const tsch_link_t* x);

/* check that cell have used local or nbr
 * @return < 0 - no cell found
 *         >= 0 - have some cell
 */
int  msf_is_avoid_cell(msf_cell_t x);
int  msf_is_avoid_cell_at(uint16_t slot_offset, uint16_t channel_offset);
int  msf_is_avoid_slot(uint16_t slot_offset);

/* @brief check that cell is used local
 * @return < 0 - no cell found
 *         >= 0 - have some cell
*/
int  msf_is_avoid_local_cell(msf_cell_t x);
int  msf_is_avoid_local_slot(uint16_t slot_offset);

typedef int msf_chanel_mask_t;
msf_chanel_mask_t  msf_avoided_slot_chanels(uint16_t slot_offset);

// avoids of nbr by local/remote using.
AvoidResult msf_avoid_nbr_use_cell(msf_cell_t x, const tsch_neighbor_t *n, AvoidOption userange);

// completely unvoids nbr cells (forget nbr)
void msf_unvoid_nbr_cell(msf_cell_t x, const tsch_neighbor_t *n);
void msf_unvoid_nbr_cells(const tsch_neighbor_t* n);

// unvoids nbr remote using cells
void msf_release_nbr_cell(msf_cell_t x, const tsch_neighbor_t *n);
void msf_release_nbr_cells(const tsch_neighbor_t* n);
// unvoids nbr local using cells
void msf_release_nbr_cell_local(msf_cell_t x, const tsch_neighbor_t *n);
void msf_release_nbr_cells_local(const tsch_neighbor_t* n);

/* @return < 0 - no cell found
 *         >= 0 - AvoidOptions for cell
 * */
int  msf_is_avoid_nbr_cell(msf_cell_t x, const tsch_neighbor_t *n);
int  msf_is_avoid_nbr_slot(uint16_t slot_offset, const tsch_neighbor_t *n);


// mark avoid cell defult - denotes, that all nbrs are know it.
//      default cells are ignored in enumerations
void msf_avoid_link_cell_default(const tsch_link_t* x);

//@return - amount of avoid cells with specified range
int msf_avoid_num_cells_at_range(unsigned range);
//@return - amount of avoid cells with range <= specified
int msf_avoid_num_cells_in_range(unsigned range);
int msf_avoid_num_local_cells();

// enum cells no belong nbr_skip, no marked aoMARK, no DEFAULT
// @arg calls - NULL, not collect cells, just calculates cells amount.
// @arg range - AvoidOption set of aoUSE_xxx.
//                  | aoMARK - denotes not skip marked cells
// @result - cells->head.num_cells= amount of filled cells
// @return - >0 - amount of cells append
// @return - =0 - no cells to enumerate
// @return - <0 - no cells append, not room to <cells>
int msf_avoid_enum_cells(SIXPCellsPkt* cells, unsigned limit
                            , unsigned range // @sa AvoidRange
                            , tsch_neighbor_t* nbr_skip);

// clenup cells, that are known by n
int msf_avoid_clean_cells_for_nbr(SIXPCellsPkt* cells, const tsch_neighbor_t *n);

// NRSF use it to notify cells exposed to nbrs
void msf_avoid_mark_all();

//   mark cell as no aoUSE_REMOTE_xxx, if range < stored one.
//   @return 0 - no cells used
//           >0 - cell is used/updated
int  msf_avoid_use_nbr_cell(msf_cell_t x, const tsch_neighbor_t *n, AvoidOption range);

//   mark cell as no aoUSE_REMOTE_xxx.
//          if range < stored one, cell remark as aoDROPED, else unvoids
//   @return <0 - no cells unused
//           0  - cells unused
//           >0 - cell is marked as aoDROPED, return last cell  AvoidOptions
int  msf_unvoid_drop_nbr_cell(msf_cell_t x, const tsch_neighbor_t *n, AvoidOption range);

// this unvoids all cells that are no any aoUSE
//      such cells registred NRSF for deleted cells
void msf_release_unused();


#endif /* _MSF_RESERVED_CELL_H_ */
/** @} */
