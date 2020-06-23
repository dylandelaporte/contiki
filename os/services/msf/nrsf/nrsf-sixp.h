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
 * \addtogroup link-layer
 * @{
 */
/**
 * \defgroup msf 6TiSCH Minimal Scheduling Function (MSF)
 * @{
 */

/**
 * \file
 *         NRSF 6P-related common helpers (for internal use)
 * \author
 *         alexrayne <alexraynepe196@gmail.com>
 */

#ifndef _NRSF_SIXP_H
#define _NRSF_SIXP_H

#include <stdint.h>
#include "net/linkaddr.h"
#include "net/mac/tsch/sixtop/sixtop.h"
#include "net/mac/tsch/sixtop/sixp-pkt.h"
#include "net/mac/tsch/sixtop/sixp-pkt-ex.h"



union NRSFMeta{
    struct fields_t{
        // nrsf_avoid_cells use it for append new avoids
        //< @sa AvoidOption:aoUSE_xxx
        uint8_t         avoid_use;
        uint8_t         dummy:4;
        //cells with aoFIXED placed first, count of this cells
        uint8_t         fixed_cnt:4;
    }                   field;
    sixp_pkt_metadata_t raw;
};
typedef union NRSFMeta NRSFMeta;

void nrsf_avoid_cells(SIXPeerHandle* hpeer, SIXPCellsHandle* hcells);
void nrsf_unvoid_cells(SIXPeerHandle* hpeer, SIXPCellsHandle* hcells);
void nrsf_check_cells(SIXPeerHandle* hpeer, SIXPCellsHandle* hcells);



//-----------------------------------------------------------------------------
enum {
    //< like ADD, but only checks list cells is avoids
    SIXP_PKT_CMD_CHECK = 0x18
};

/**
 * \brief Send a ADD request
 * \param cell_type Type of a negotiated cell to add
 */
void nrsf_sixp_add_send_request(const linkaddr_t *peer_addr, SIXPCellsPkt* hcells);

/**
 * \brief Handler for reception of a ADD request
 * \param peer_addr The source MAC address of the request
 * \param body A pointer to the body of the request
 * \param body_len The length of body in bytes
 */
SIXPError nrsf_sixp_add_recv_request(SIXPeerHandle* hpeer);


//-----------------------------------------------------------------------------
/**
 * \brief Send a DELETE request
 * \param cell_type Type of a negotiated cell to delete
 */
void nrsf_sixp_delete_send_request(const linkaddr_t *peer_addr, SIXPCellsPkt* hcells);

/**
 * \brief Handler for reception of a DELETE request
 * \param peer_addr The source MAC address of the request
 * \param body A pointer to the body of the request
 * \param body_len The length of body in bytes
 */
SIXPError nrsf_sixp_delete_recv_request(SIXPeerHandle* hpeer);

//-----------------------------------------------------------------------------
/**
 * \brief Send a LIST request
 */
void nrsf_sixp_check_send_request(const linkaddr_t *peer_addr, SIXPCellsPkt* hcells);

/**
 * \brief Handler for reception of a RELOCATE request
 * \param peer_addr The source MAC address of the request
 * \param body A pointer to the body of the request
 * \param body_len The length of body in bytes
 */
SIXPError nrsf_sixp_check_recv_request(SIXPeerHandle* hpeer);

/**
 * \brief Handler for reception of a response for RELOCATE
 * \param peer_addr The source MAC address of the response
 * \param rc Return code in the response
 * \param body A pointer to the body of the request
 * \param body_len The length of body in bytes
 */
SIXPError nrsf_sixp_check_recv_response(SIXPeerHandle* hpeer);


//-----------------------------------------------------------------------------
/**
 * \brief Send a CLEAR request
 * \param peer_addr The destination MAC address
 *        hcells === NULL
 */
void nrsf_sixp_clear_send_request(const linkaddr_t *peer_addr, SIXPCellsPkt* hcells);

/**
 * \brief Handler for reception of a CLEAR request
 * \param peer_addr The source MAC address of the request
 */
SIXPError nrsf_sixp_clear_recv_request(SIXPeerHandle* hpeer);



#endif /* !_NRSF_H */
/** @} */
/** @} */
