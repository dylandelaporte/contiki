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
 *         Neighbor Resolution Schedule Function (MSF extention)
 * \author
 *         alexrayne <alexraynepe196@gmail.com>
 */

#ifndef _NRSF_H
#define _NRSF_H

#include "net/mac/tsch/sixtop/sixtop.h"
#include "net/mac/tsch/sixtop/sixp-pkt-ex.h"

/* Constants */
/**
 * \brief Scheduling Function ID for MSF
 */
#define NRSF_SFID 2

/* Variables */
/**
 * \brief sixtop_sf_t of nrsf
 */
extern const sixtop_sf_t nrsf;



// calbacks from MSF
void nrsf_on_msf_use_cell(tsch_neighbor_t *nbr, tsch_link_t *cell);
void nrsf_on_msf_release_cell(tsch_neighbor_t *nbr, tsch_link_t *cell);
void nrsf_on_msf_nbr_clean(tsch_neighbor_t *nbr);
void nrsf_on_msf_new_nbr(tsch_neighbor_t *nbr);

// calbacks from 6P
// after 6P transactions are completes, try exec NRSF actions
void nrsf_on_6ptrans_free(void);

// tasks informs nbr
// @return - amount of sent 6p requests
int nrsf_used_cells_by_nbr(tsch_neighbor_t *nbr, SIXPCellsPkt* cells);
int nrsf_release_cells_by_nbr(tsch_neighbor_t *nbr, SIXPCellsPkt* cells);
int nrsf_notify_cells_2nbr(tsch_neighbor_t *nbr, SIXPCellsPkt* cells);
int nrsf_clean_cells_by_nbr(tsch_neighbor_t *nbr);


// MSF related routines

// enumerate negotiated cells list, skip belongs nbr
// @result - cells->head.num_cells= amount of filled cells
// @return - >0 - amount of cells append
// @return - =0 - no cells to enumerate
// @return - <0 - no cells append, not room to <cells>
int msf_negotiated_enum_cells_besides_nbr(tsch_neighbor_t *nbr, SIXPCellsPkt* cells, unsigned limit);



#endif /* !_NRSF_H */
/** @} */
/** @} */
