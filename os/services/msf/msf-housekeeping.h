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
 *         MSF Housekeeping
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#ifndef _MSF_HOUSEKEEPING_H_
#define _MSF_HOUSEKEEPING_H_

#include "net/linkaddr.h"
//#include "services/shell/shell.h"
#include "msf-avoid-cell.h"


/**
 * \brief Start the housekeeping process
 */
void msf_housekeeping_start(void);

/**
 * \brief Stop the housekeeping process
 */
void msf_housekeeping_stop(void);

/**
 * \brief Set the parent (time-source) address
 * \param new_parent The MAC address of the new parent
 */
void msf_housekeeping_set_parent_addr(const linkaddr_t *new_parent);

/**
 * \brief Return the parent address
 * \return The MAC address of the parent if available, otherwise NULL
 */
static inline
const linkaddr_t * msf_housekeeping_get_parent_addr(void){
    extern const linkaddr_t* msf_housekeeping_parent_addr;
    return msf_housekeeping_parent_addr;
}

/**
 * \brief initiate relocation for cell
 */
void msf_housekeeping_request_cell_to_relocate(tsch_link_t *cell);

/**
 * \brief Delete a cell to be relocated
 */
void msf_housekeeping_delete_cell_to_relocate(void);

/**
 * \brief ckecks that cell not conflicts with any, and rquest relocations if need
 *        this used by local, auto-cells
 */
void msf_housekeeping_inspect_link_consintensy(tsch_link_t *cell);

/**
 * \brief ckecks that remote cell not conflicts with any, and rquest relocations if need
 */
void msf_housekeeping_inspect_cell_consintensy(
                    msf_cell_t cell,  tsch_neighbor_t *n,
                    uint8_t cell_avoid_opts //sixp_pkt_cell_options_t
                );

/**
 * \brief Establish negotiated cell for RX with parent.
 *        Need when parent rx cell conflicts with some other cells
 */
void msf_housekeeping_negotiate_for_parent_rx(void);

/**
 * \brief Establish negotiated cell for TX with nbr. not parent.
 *        Need when nbr rx cell conflicts with mine one.
 */
void msf_housekeeping_negotiate_for_nbr_tx(const linkaddr_t *peer_addr);

/**
 * \brief Resolve schedule inconsistency
 * \param peer_addr The MAC address of the peer to which we detecte
 * inconsitency in the schedule
 * \details This call triggers a CLEAR transaction
 */
void msf_housekeeping_resolve_inconsistency(const linkaddr_t *peer_addr);

/**
 * \brief Schedule to delete a cell
 * \param cell A pointer to a negotiated cell to delete
 */
void msf_housekeeping_delete_cell_later(tsch_link_t *cell);

/**
 * \brief When some close nbr free cells, msf can retry add cells
 *
 * \details Here start implementing blocking ADD requests on sequental failures.
 *      For second failure, ADD request blocks until close nbr free some cells.
        This helps save traffic from unnesesary requests

 * \param cell A pointer to a negotiated cell to delete
 */
void msf_housekeeping_on_free_close_cells( const linkaddr_t *peer_addr);

#endif /* !_MSF_HOUSEKEEPING_H_ */
/** @} */
