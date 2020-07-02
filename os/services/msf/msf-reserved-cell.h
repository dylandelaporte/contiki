/*
 * Copyright (c) 2019, Inria.
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
 *         MSF Reserved Cell APIs
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#ifndef _MSF_RESERVED_CELL_H_
#define _MSF_RESERVED_CELL_H_

#include <stdint.h>

#include "net/linkaddr.h"
#include "net/mac/tsch/tsch.h"
#include "msf-negotiated-cell.h"

/**
 * \brief Return the number of reserved cells for a peer
 * \param peer_addr The MAC address of the target peer
 */
int msf_reserved_cell_get_num_cells(const linkaddr_t *peer_addr);

/**
 * \brief Return a reserved cell matching conditions
 * \param peer_addr The MAC address of a target cell
 * \param slot_offset The slot offset of a target cell
 * \param channel_offset The channel offset of a target cell
 * \return non-NULL if found, otherwise NULL
 */
tsch_link_t *msf_reserved_cell_get(const linkaddr_t *peer_addr,
                                   long slot_offset,
                                   long channel_offset);

/* Modes for reservation selects new slots
 * */
enum ReserveMode {
    RESERVE_NEW_CELL = -1 ,
    //< search forslot, that have busy with close nbr
    RESERVE_NBR_BUSY_CELL = -2 ,
};
typedef enum ReserveMode ReserveMode;

/**
 * \brief Add (reserve) a cell in the slotframe for negotiated cells
 * \param peer_addr The MAC address of the peer
 * \param cell_type Type of a reserved cell (TX or RX)
 * \param slot_offset The slot offset of a reserved cell
 * \param channel_offset The channel offset of a reserved cell
 */
tsch_link_t *msf_reserved_cell_add(const linkaddr_t *peer_addr,
                                   msf_negotiated_cell_type_t cell_type,
                                   int32_t slot_offset,
                                   int32_t channel_offset);

/** This is less strictive msf_reserved_cell_add - it allow new cells in alredy ocupied slot.
 *  RELOCATIE use it for allocate links that migrate chanel in slot
 */
tsch_link_t *msf_reserved_cell_over(const linkaddr_t *peer_addr,
                                   msf_negotiated_cell_type_t cell_type,
                                   msf_cell_t new_cell);

/** This is less stricted msf_reserved_cell_add - it not check cell avoidance.
 *  RELOCATIE use it for allocate links that migrate chanel in slot
 */
tsch_link_t * msf_reserved_cell_add_anyvoid(const linkaddr_t *peer_addr,
                      msf_negotiated_cell_type_t cell_type,
                      int32_t slot_offset, int32_t channel_offset);

/**
 * \brief Delete all the cells reserved for a peer
 * \param peer_addr The MAC address of the peer
 */
void msf_reserved_cell_delete_all(const linkaddr_t *peer_addr);

/**
 * \brief Delete reserved link
 */
void msf_reserved_release_link(tsch_link_t *cell);

/**
 * \brief Check that peer have reserved cells. Need to avoid concurent operations
 *          on same peer, that can leed to SHEDULE INCONSISTENT
 * \param peer_addr The MAC address of the peer
 *       = NULL - checks that any link eserved
 */
bool msf_is_reserved_for_peer(const linkaddr_t *peer_addr);

//==============================================================================
long msf_find_unused_slot_offset(tsch_slotframe_t *slotframe
                                    , ReserveMode mode
                                    //< used for mode RESERVE_NBR_BUSY_CELL
                                    , const linkaddr_t *peer_addr
                                    );
int  msf_find_unused_slot_chanel(uint16_t slot, msf_chanel_mask_t skip );



#endif /* _MSF_RESERVED_CELL_H_ */
/** @} */
