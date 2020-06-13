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
 *         6top Protocol (6P) Packet Manipulation APIs extended
 * \author
 *         alexrayne <alexraynepe196@gmail.com>
 */

#ifndef CONTIKI_OS_NET_MAC_TSCH_SIXTOP_SIXP_PKT_EX_H_
#define CONTIKI_OS_NET_MAC_TSCH_SIXTOP_SIXP_PKT_EX_H_

#include <stdint.h>
#include "assert.h"
#include "sixp.h"
#include "sixp-pkt.h"



struct SIXPHandle {
    const uint8_t*  body;
    unsigned        body_len;
    sixp_pkt_type_t type;
    sixp_pkt_code_t code;
};
typedef struct SIXPHandle SIXPHandle;

typedef struct sixp_trans sixp_trans_t;
struct SIXPeerHandle {
    SIXPHandle          h;
    const linkaddr_t*   addr;
    // this may be == NULL, if trans not specified
    sixp_trans_t*       trans;
};
typedef struct SIXPeerHandle SIXPeerHandle;

enum SIXPError {
    sixpOK      = 0,
    sixpFAIL    = -1,
};
typedef enum SIXPError SIXPError;



//==============================================================================
struct SIXPCellsHandle {
    sixp_pkt_metadata_t     meta;
    sixp_pkt_cell_options_t cell_options;
    sixp_pkt_num_cells_t    num_cells;
    sixp_pkt_cell_t*        cell_list;
    unsigned                cell_list_len;
};
typedef struct SIXPCellsHandle SIXPCellsHandle;

SIXPError sixp_pkt_parse_cell_list(SIXPHandle* h, SIXPCellsHandle* dst);
SIXPError sixp_pkt_parse_cells(SIXPHandle* h, SIXPCellsHandle* dst);

// steps h body to position after dst cells
SIXPError sixp_pkt_after_cells(SIXPHandle* h, SIXPCellsHandle* dst);

//< takes from h->body new cells sequence, after dst described
//  so in pack may be placed a few cells sets:
//         body>head       >cells0...n   > head      > cells0..m
//             :meta,op,cnt:s0c0, s1c1 ..:meta,op,cnt:s0c0, s1c1 ..
SIXPError sixp_pkt_parse_next_cells(SIXPHandle* h, SIXPCellsHandle* dst);


// strage for generating new cells packet body
struct __attribute__((aligned (4))) SIXPCellsPkt {
    struct __attribute__((packed)) head_t {
    sixp_pkt_metadata_t     meta;
    sixp_pkt_cell_options_t cell_options;
    sixp_pkt_num_cells_t    num_cells;
    }                       head;
    sixp_cell_t             cells[0];
};
typedef struct SIXPCellsPkt SIXPCellsPkt;

static inline
void sixp_pkt_cells_init(SIXPCellsPkt* pkt, unsigned size){
    pkt->head.num_cells = (size-sizeof(pkt->head))/sizeof(sixp_pkt_cell_t);
}

static inline
void sixp_pkt_cells_reset(SIXPCellsPkt* pkt){
    pkt->cells[-1].raw = 0;
}

static inline
void sixp_pkt_cells_assign(SIXPHandle* h, SIXPCellsPkt* pkt){
    h->body             = (const uint8_t*)&pkt->head;
    h->body_len         = sizeof(pkt->head)+(pkt->head.num_cells * sizeof(sixp_pkt_cell_t));
}

static inline
void sixp_pkt_cells_append(SIXPHandle* h, SIXPCellsPkt* pkt){
    const uint8_t* body =(const uint8_t*)pkt;
    assert( (h->body + h->body_len) == body);
    h->body_len         += sizeof(pkt->head)+(pkt->head.num_cells * sizeof(sixp_pkt_cell_t));
}

static inline
sixp_cell_t sixp_pkt_get_cell(const void* buf, unsigned idx) {
    sixp_cell_t res;
    //memcpy(&res, ((sixp_pkt_cell_t*)buf)+idx, sizeof(sixp_pkt_cell_t));
    res.raw = ((sixp_pkt_cell_t*)buf)[idx].raw;
    return res;
}


//==============================================================================
int
sixp_pkt_output(SIXPeerHandle* h, uint8_t sfid,
            sixp_sent_callback_t func, void *arg, uint16_t arg_len);


//==============================================================================
sixp_trans_t *sixp_trans_find_for_peer(SIXPeerHandle *h);


#endif /* CONTIKI_OS_NET_MAC_TSCH_SIXTOP_SIXP_PKT_EX_H_ */
