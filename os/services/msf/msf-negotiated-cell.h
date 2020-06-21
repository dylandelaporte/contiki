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
 *         MSF Negotiated Cell APIs
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#ifndef _MSF_NEGOTIATED_CELL_H
#define _MSF_NEGOTIATED_CELL_H

#include "net/linkaddr.h"
#include "net/mac/tsch/tsch.h"
#include "msf-avoid-cell.h"

/* definitions */
/**
 * \brief Types of negotiated cells
 */
typedef enum {
  MSF_NEGOTIATED_CELL_TYPE_NO = 0,               /**< ??? */
  MSF_NEGOTIATED_CELL_TYPE_TX = LINK_OPTION_TX,  /**< Negotiated TX cell */
  MSF_NEGOTIATED_CELL_TYPE_RX = LINK_OPTION_RX,  /**< Negotiated RX cell */
} msf_negotiated_cell_type_t;

const char* msf_negotiated_cell_type_str(msf_negotiated_cell_type_t x);
const char* msf_negotiated_cell_type_arrow(msf_negotiated_cell_type_t x);

static inline
msf_negotiated_cell_type_t msf_negotiated_link_cell_type(const tsch_link_t* x){
    return (msf_negotiated_cell_type_t)
                ( x->link_options & (LINK_OPTION_TX|LINK_OPTION_RX) );
}

//----------------------------------------------------------------------------
// @brief compact cell link id, for pass as process event parameter
//  MSF rely on assumption that RF have <= 255 channels
union MSFCellID{
    struct {
        uint16_t    slot;
        uint8_t     chanel;
        // @sa AvoidOptions - this is a mix of cell:meta + options, that represents
        //           AvoidOptions of cell
        uint8_t     link_options;
    }           field;
    //unsigned long   raw;
    void*           as_void;
};
typedef union MSFCellID MSFCellID;

static inline
MSFCellID   msf_cellid_of_link(const tsch_link_t* x){
    MSFCellID cell;
    cell.field.slot         = x->timeslot;
    assert(x->channel_offset <= 0xff);
    cell.field.chanel       = x->channel_offset;
    // TODO: need aoFIXED status here somehow.
    cell.field.link_options = (uint8_t)(x->link_options & aoTX);
    return cell;
}

static inline
MSFCellID   msf_cellid_of_sixp(msf_cell_t x, uint8_t opts){
    MSFCellID cell;
    cell.field.slot         = x.field.slot;
    assert(x.field.chanel <= 0xff);
    cell.field.chanel       = x.field.chanel;
    cell.field.link_options = (uint8_t)opts;
    return cell;
}

static inline
bool msf_cellid_is_remote(MSFCellID x){
    return (x.field.link_options & aoUSE_REMOTE) != 0;
}

static inline
bool msf_cellid_is_tx(MSFCellID x){
    return (x.field.link_options & aoTX) != 0;
}



//----------------------------------------------------------------------------
/**
 * \brief Activate the negotiated cell scheduling
 */
void msf_negotiated_cell_activate(void);

/**
 * \brief Deactivate the negotiated cell scheduling
 * \details All the negotiated cells including reserved ones will be
 * deleted
 */
void msf_negotiated_cell_deactivate(void);

/**
 * \brief Return a pointer to the slotframe for negotiated cells
 */
static inline
tsch_slotframe_t * msf_negotiated_cell_get_slotframe(void)
{
  extern tsch_slotframe_t * msf_negotiate_slotframe;
  return msf_negotiate_slotframe;
}

/* @brief - check, that link is valid in negotiated slotframe
 * */
bool msf_is_negotiated_cell(tsch_link_t *cell);

/**
 * \brief Add a negotiated cell
 * \param peer_addr The MAC address of the peer
 * \param type Type of the negotiated cell (TX or RX)
 * \param slot_offset The slot offset of the cell
 * \param channel_offset The channel offset of the cell
 * \return 0 on success, -1 on failure
 */
int msf_negotiated_cell_add(const linkaddr_t *peer_addr,
                            msf_negotiated_cell_type_t type,
                            uint16_t slot_offset, uint16_t channel_offset);

/**
 * \brief Delete a negotiated cell
 * \param cell A pointer to the cell to delete
 */
void msf_negotiated_cell_delete(tsch_link_t *cell);

/**
 * \brief Delete all negotiated cells associated with a peer
 * \param peer_addr The MAC address of the target peer
 * \details Specify NULL for peer_addr to remove all the negotiated
 * cells in the schedule
 */
void msf_negotiated_cell_delete_all(const linkaddr_t *peer_addr);

/**
 * \brief Return whether a negotiated TX cell is scheduled with a peer
 * \param nbr A tsch_neighbor_t object for the peer
 * \return true if it is the case, otherwise false
 */
bool msf_negotiated_nbr_is_scheduled_tx(tsch_neighbor_t *nbr);

/**
 * \brief Return whether scheduled any negotiated with a nbr
 * \param nbr A tsch_neighbor_t object for the peer
 * \return true if it is the case, otherwise false
 */
bool msf_negotiated_is_scheduled_nbr(tsch_neighbor_t *nbr);
bool msf_negotiated_is_scheduled_peer(const linkaddr_t *peer_addr);

/**
 * \brief Return a negotiated cell to delete
 * \param peer_addr The MAC address of the target peer
 * \param cell_type The type of a negotiated cell to delete
 * \return non-NULL if there is a candidate, otherwise NULL
 */
tsch_link_t *msf_negotiated_cell_get_cell_to_delete(
  const linkaddr_t *peer_addr,
  msf_negotiated_cell_type_t cell_type);

/**
 * \brief Return the number of negotiated cells scheduled with a peer
 * \param cell_type The type of negotiated cells of interest
 * \param peer_addr The MAC address of the target peer
 */
uint16_t msf_negotiated_cell_get_num_cells(msf_negotiated_cell_type_t cell_type,
                                           const linkaddr_t *peer_addr);

/**
 * \brief Update the NumTx counter
 * \param slot_offset The slot offset at which the last transmission occurrs
 * \param num_tx The number of transmissions performed for the frame
 * \param mac_tx_status The resuting TX status
 */
void msf_negotiated_cell_update_num_tx(uint16_t slot_offset,
                                       uint16_t num_tx, uint8_t mac_tx_status);

/**
 * \brief lookup a cell with bad signs/features to relocate
 * \return A pointer to a negotiated TX cell to relocate if any,
 * otherwise NULL
 */
tsch_link_t *msf_negotiated_propose_cell_to_relocate(void);

// markup cell for relocation
bool msf_is_marked_as_relocate(const tsch_link_t *cell);

// \brief get negotiated link marked for relocate
tsch_link_t *msf_negotiated_get_cell_to_relocate(void);

/**
 * \brief Return NumTx of a negotiated TX cell
 * \return The value of NumTX, or 0 if the given cell is not a negotiated
 * TX cell
 */
uint16_t msf_negotiated_cell_get_num_tx(tsch_link_t *cell);

/**
 * \brief Return NumTxAck of a negotiated TX cell
 * \return The value of NumTX, or 0 if the given cell is not a negotiated
 * TX cell
 */
uint16_t msf_negotiated_cell_get_num_tx_ack(tsch_link_t *cell);

/**
 * \brief Mark a negotiated RX cell as used
 * \param src_addr The src MAC address of a received packet
 * \param slot_offset The slot offste of the target negotiated RX cell
 */
void msf_negotiated_cell_rx_mark_used(const linkaddr_t *src_addr,
                                    uint16_t slot_offset);

/**
 * \brief Detect and delete unused negotiated cells
 * \details Negotiated cells scheduled with a neighbor which is
 * thought being inactive will be delete
 */
void msf_negotiated_cell_delete_unused_cells(void);

enum MSFInspectResult{
    irNOCELL    = -1,   //< for no cells recognised - is no negotiated cell
    irFAIL      = 0,    //< conflicts not solves
    irOK        = 1,    //< no conflicts
    irRELOCATE  = 2,    //< requested relocation for conflict resolve
};
typedef enum MSFInspectResult MSFInspectResult;
/**
 * \brief Detect and resolve collisions of new link with negotiated links.
 * \details It try to relocate negotiateg link, that conflicts with cell
 *          Also cleanup reserved links, that overlaps cell
 */
MSFInspectResult msf_negotiated_inspect_vs_link(tsch_link_t *cell);
MSFInspectResult msf_negotiated_inspect_vs_cellid(MSFCellID cell);



/* callbacks */
// call after new cell use, but before marked avoiding
#ifdef MSF_AFTER_CELL_USE
void MSF_AFTER_CELL_USE(tsch_neighbor_t *nbr, tsch_link_t *cell);
#else
#define MSF_AFTER_CELL_USE(...)
#endif

// call after cell no use/deleted, after unmarked avoiding
#ifdef MSF_AFTER_CELL_RELEASE
void MSF_AFTER_CELL_RELEASE(tsch_neighbor_t *nbr, tsch_link_t *cell);
#else
#define MSF_AFTER_CELL_RELEASE(...)
#endif

#ifdef MSF_AFTER_CELL_CLEAN
void MSF_AFTER_CELL_CLEAN( tsch_neighbor_t *nbr );
#else
#define MSF_AFTER_CELL_CLEAN(...)
#endif

#ifdef MSF_ON_NEW_NBR
void MSF_ON_NEW_NBR( tsch_neighbor_t *nbr );
#else
#define MSF_ON_NEW_NBR(...)
#endif



#endif /* !_MSF_NEGOTIATED_CELL_H */
/** @} */
