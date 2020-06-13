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
 *         Neighbor Resolution Schedule Function
 *         this is MSF extention
 * \author
 *         alexrayne <alexraynepe196@gmail.com>
 */

#include <stdbool.h>
#include <string.h>

#include "contiki.h"
#include "lib/assert.h"

#include "net/linkaddr.h"
#include "net/mac/mac.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/sixtop/sixtop.h"
#include "net/mac/tsch/sixtop/sixp.h"
#include "net/mac/tsch/sixtop/sixp-trans.h"
#include "services/shell/serial-shell.h"

#include "msf.h"
#include "msf-autonomous-cell.h"
#include "msf-housekeeping.h"
#include "msf-negotiated-cell.h"
#include "msf-reserved-cell.h"
#include "msf-avoid-cell.h"
#include "msf-sixp.h"

#include "nrsf.h"
#include "nrsf-sixp.h"

#include "sys/log.h"
#define LOG_MODULE "NRSF"
#define LOG_LEVEL LOG_LEVEL_MSF



//=============================================================================
static void nrsf_init_tasks();



//=============================================================================
void nrsf_avoid_cells(SIXPeerHandle* hpeer, SIXPCellsHandle* hcells){
    const tsch_neighbor_t *n = tsch_queue_get_nbr(hpeer->addr);
    for (unsigned i = 0; i < hcells->num_cells; ++i){
        msf_avoid_nbr_use_cell(sixp_pkt_get_cell(hcells->cell_list, i), n);
    }
}

void nrsf_unvoid_cells(SIXPeerHandle* hpeer, SIXPCellsHandle* hcells){
    const tsch_neighbor_t *n = tsch_queue_get_nbr(hpeer->addr);
    for (unsigned i = 0; i < hcells->num_cells; ++i){
        msf_release_nbr_cell(sixp_pkt_get_cell(hcells->cell_list, i), n);
    }
}

void nrsf_check_cells(SIXPeerHandle* hpeer, SIXPCellsHandle* hcells){
    unsigned j = 0;
    for (unsigned i = 0; i < hcells->num_cells; ++i){
        sixp_cell_t x = sixp_pkt_get_cell(hcells->cell_list, i);
        if (!msf_is_avoid_cell(x))
            continue;
        hcells->cell_list[j].raw = x.raw;
        ++j;
    }
    hcells->num_cells       = j;
    hcells->cell_list_len   = j * sizeof(sixp_pkt_cell_t);
}



//=============================================================================
/* static functions */
void nrsf_init(void);
void nrsf_input_handler(sixp_pkt_type_t type, sixp_pkt_code_t code,
                          const uint8_t *body, uint16_t body_len,
                          const linkaddr_t *src_addr);
void nrsf_timeout_handler(sixp_pkt_cmd_t cmd,
                            const linkaddr_t *peer_addr);
void nrsf_error_handler(sixp_error_t err, sixp_pkt_cmd_t cmd,
                          uint8_t seqno, const linkaddr_t *peer_addr);

const sixtop_sf_t nrsf = {
  NRSF_SFID,
  (((2 << (TSCH_MAC_MAX_BE - 1)) - 1) *
   TSCH_MAC_MAX_FRAME_RETRIES *
   MSF_SLOTFRAME_LENGTH * MSF_SLOT_LENGTH_MS / 1000 * CLOCK_SECOND),
  nrsf_init,
  nrsf_input_handler,
  nrsf_timeout_handler,
  nrsf_error_handler,
};

/*---------------------------------------------------------------------------*/
void nrsf_init(void)
{
    nrsf_init_tasks();
}
/*---------------------------------------------------------------------------*/
void nrsf_input_handler(sixp_pkt_type_t type, sixp_pkt_code_t code,
              const uint8_t *body, uint16_t body_len,
              const linkaddr_t *src_addr)
{
  sixp_trans_t *trans;
  sixp_pkt_cmd_t cmd;
  sixp_pkt_t     pkt;

  sixp_pkt_init_in(&pkt, type, code, NRSF_SFID);

  if(msf_is_activated() == false) {
    LOG_ERR("MSF is not activated; ignore the input 6P packet\n");
    return;
  }
  trans = sixp_trans_find_for_pkt(src_addr, &pkt);
  if(trans == NULL) {
    LOG_ERR("cannot find a 6P transaction of a received 6P packet\n");
    return;
  } else {
    cmd = sixp_trans_get_cmd(trans);
  }

  SIXPeerHandle hpeer;
  hpeer.addr        = src_addr;
  hpeer.trans       = trans;
  hpeer.h.body      = body;
  hpeer.h.body_len  = body_len;
  hpeer.h.type      = type;
  hpeer.h.code      = code;

  SIXPError ok = sixpOK;
  if(type == SIXP_PKT_TYPE_REQUEST) {
    assert(code.value == cmd);
    if(code.value == SIXP_PKT_CMD_ADD) {
      ok = nrsf_sixp_add_recv_request(&hpeer);
    } else if(code.value == SIXP_PKT_CMD_DELETE) {
      ok = nrsf_sixp_delete_recv_request(&hpeer);
    } else if(code.value == SIXP_PKT_CMD_CHECK) {
      ok = nrsf_sixp_check_recv_request(&hpeer);
    } else if(code.value == SIXP_PKT_CMD_CLEAR) {
      ok = nrsf_sixp_clear_recv_request(&hpeer);
    }
  } else if(type == SIXP_PKT_TYPE_RESPONSE) {

      LOG_INFO("response cmd%u with %s from ", cmd
                          , msf_sixp_get_rc_str(hpeer.h.code.rc));
      LOG_INFO_LLADDR(hpeer.addr);
      LOG_INFO_("\n");

    if((uint8_t)cmd == SIXP_PKT_CMD_CHECK) {
      ok = nrsf_sixp_check_recv_response(&hpeer);
    } else {
        LOG_ERR("response not expects!\n");
    }
    if (ok < 0)
        LOG_ERR("parse error\n");

  } else {
    /* MSF doesn't use 3-step transactions */
    LOG_ERR("received a 6P Confirmation, which is not supported by MSF\n");
  }
}
/*---------------------------------------------------------------------------*/
void nrsf_timeout_handler(sixp_pkt_cmd_t cmd, const linkaddr_t *peer_addr)
{
  assert(peer_addr != NULL);
  if(cmd == SIXP_PKT_CMD_ADD) {
    LOG_INFO("ADD transaction ends because of timeout\n");
  } else {
    /* do nothing */
  }

  const linkaddr_t * parent = msf_housekeeping_get_parent_addr();
  if (parent == NULL){
      // what's up ?
  }
  else if( linkaddr_cmp(peer_addr, parent) ) {
    /* we are the initiator */
  } else {
    /* we are the responder */
    /*
     * schedule inconsistency may happen because of this timeout of
     * the transaction, where the peer completes the transaction by
     * our L2 MAC, but we don't. Better to confirm if there is
     * schedule consistency.
     */
  }
}
/*---------------------------------------------------------------------------*/
void nrsf_error_handler(sixp_error_t err, sixp_pkt_cmd_t cmd, uint8_t seqno,
              const linkaddr_t *peer_addr)
{
  LOG_WARN("A 6P transaction for (cmd: %u) with ", cmd);
  LOG_WARN_LLADDR(peer_addr);
  LOG_WARN_(" ends with an error (err: %u)\n", err);
  if(err == SIXP_ERROR_SCHEDULE_INCONSISTENCY) {
    //nrsf_negotiated_cell_delete_all(peer_addr);
  }
}
/*---------------------------------------------------------------------------*/

void nrsf_sent_callback_responder(void *arg, uint16_t arg_len,
                              const linkaddr_t *dest_addr,
                              sixp_output_status_t status);

void nrsf_sent_complete_responder(void *arg, uint16_t arg_len,
                              const linkaddr_t *dest_addr,
                              sixp_output_status_t status);

static
void nrsf_send_complete(SIXPeerHandle* hpeer);

static
void nrsf_report_sent(SIXPeerHandle* hpeer, const char* name, SIXPError ok);

//=============================================================================
static
void nrsf_sixp_single_send_request(SIXPeerHandle* hpeer, const char* name);

void nrsf_sixp_add_send_request(const linkaddr_t *peer_addr, SIXPCellsPkt* hcells){
    assert(peer_addr != NULL);
    LOG_DBG("request ADD[%u] to", hcells->head.num_cells);
    LOG_INFO_LLADDR(peer_addr);
    LOG_DBG_("\n");

    SIXPeerHandle hpeer;
    sixp_pkt_cells_assign(&hpeer.h, hcells);
    hpeer.addr        = peer_addr;
    hpeer.h.code.cmd  = SIXP_PKT_CMD_ADD;

    nrsf_sixp_single_send_request(&hpeer, "ADD");
}

void nrsf_sixp_del_send_request(const linkaddr_t *peer_addr, SIXPCellsPkt* hcells){
    assert(peer_addr != NULL);
    LOG_DBG("request DEL[%u] to", hcells->head.num_cells);
    LOG_INFO_LLADDR(peer_addr);
    LOG_DBG_("\n");

    SIXPeerHandle hpeer;
    sixp_pkt_cells_assign(&hpeer.h, hcells);
    hpeer.addr        = peer_addr;
    hpeer.h.code.cmd  = SIXP_PKT_CMD_DELETE;

    nrsf_sixp_single_send_request(&hpeer, "DEL");
}

void nrsf_sixp_check_send_request(const linkaddr_t *peer_addr, SIXPCellsPkt* hcells){
    assert(peer_addr != NULL);
    LOG_DBG("request CHECK to");
    LOG_INFO_LLADDR(peer_addr);
    LOG_DBG_("\n");

    SIXPeerHandle hpeer;
    sixp_pkt_cells_assign(&hpeer.h, hcells);
    hpeer.addr        = peer_addr;
    hpeer.h.code.cmd  = SIXP_PKT_CMD_CHECK;

    nrsf_sixp_single_send_request(&hpeer, "CHECK");
}

void nrsf_sixp_clear_send_request(const linkaddr_t *peer_addr, SIXPCellsPkt* hcells){
    assert(peer_addr != NULL);
    LOG_DBG("request CLEAR to");
    LOG_INFO_LLADDR(peer_addr);
    LOG_DBG_("\n");

    (void)hcells;
    SIXPeerHandle hpeer;
    hpeer.h.body      = NULL;
    hpeer.h.body_len  = 0;
    hpeer.addr        = peer_addr;
    hpeer.h.code.cmd  = SIXP_PKT_CMD_CLEAR;

    nrsf_sixp_single_send_request(&hpeer, "CLEAR");
}


static
void nrsf_sixp_single_send_request(SIXPeerHandle* hpeer, const char* name){
    hpeer->h.type      = SIXP_PKT_TYPE_REQUEST;
    SIXPError ok = sixp_pkt_output(hpeer, NRSF_SFID, nrsf_sent_complete_responder, NULL, 0);
    nrsf_report_sent(hpeer, name, ok);
}
/*---------------------------------------------------------------------------*/
static
bool is_valid_add_request(SIXPCellsHandle*  h)
{
  bool ret = false;

  if(h->cell_options != SIXP_PKT_CELL_OPTION_TX &&
     h->cell_options != SIXP_PKT_CELL_OPTION_RX)
  {
    LOG_INFO("bad CellOptions - %02X (should be %02X or %02X)\n",
            h->cell_options, SIXP_PKT_CELL_OPTION_TX, SIXP_PKT_CELL_OPTION_RX);
  }
  /*else if(h->num_cells != 1) {
    LOG_INFO("bad NumCells - %u (should be 1)\n", h->num_cells);
  } */
  else if(h->cell_list == NULL) {
    LOG_INFO("no CellList\n");
  } else if(h->cell_list_len < sizeof(sixp_pkt_cell_t)*(h->num_cells) ) {
    LOG_INFO("too short CellList - %u octets\n", h->cell_list_len);
  } else {
    ret = true;
  }

  return ret;
}

/*---------------------------------------------------------------------------*/
static
SIXPError nrsf_sixp_add_request(SIXPeerHandle* hpeer)
{
    SIXPCellsHandle  hcells;
    SIXPError ok = sixp_pkt_parse_cells(&hpeer->h, &hcells);
    if (ok < 0)
        return ok;

    assert(hpeer->addr != NULL);

    if(is_valid_add_request(&hcells)) {
      nrsf_avoid_cells(hpeer, &hcells);
    }

    // no response for NRSF commands.
    return sixpOK;
}

SIXPError nrsf_sixp_add_recv_request(SIXPeerHandle* hpeer)
{
    SIXPError ok  = nrsf_sixp_add_request(hpeer);
    nrsf_send_complete(hpeer);

    // no response for NRSF commands.
    return ok;
}

static
void nrsf_report_sent(SIXPeerHandle* hpeer, const char* name, SIXPError ok){
    if(ok < 0) {
      LOG_ERR("failed to send %s to ", name);
      LOG_ERR_LLADDR(hpeer->addr);
      LOG_ERR_("\n");
    } else {
      LOG_INFO("sent %s ok%d to ", name, ok );
      LOG_INFO_LLADDR(hpeer->addr);
      LOG_INFO_("\n");
    }
}

/*---------------------------------------------------------------------------*/
SIXPError nrsf_sixp_delete_recv_request(SIXPeerHandle* hpeer){
    SIXPCellsHandle  hcells;
    SIXPError ok = sixp_pkt_parse_cells(&hpeer->h, &hcells);
    if (ok < 0)
        return ok;

    assert(hpeer->addr != NULL);

    if(is_valid_add_request(&hcells)) {
      nrsf_unvoid_cells(hpeer, &hcells);
    }

    nrsf_send_complete(hpeer);

    // no response for NRSF commands.
    return sixpOK;
}

/*---------------------------------------------------------------------------*/
SIXPError nrsf_sixp_check_recv_request(SIXPeerHandle* hpeer){
    assert(hpeer->addr != NULL);
    SIXPCellsHandle  hcells;
    SIXPError ok = sixp_pkt_parse_cells(&hpeer->h, &hcells);
    if (ok < 0)
        return ok;

    if(is_valid_add_request(&hcells)) {
      nrsf_check_cells(hpeer, &hcells);
      hpeer->h.code.rc  = SIXP_PKT_RC_SUCCESS;
      hpeer->h.body     = (uint8_t*)hcells.cell_list;
      hpeer->h.body_len = hcells.cell_list_len;
    }
    else{
        return sixpFAIL;
    }

    hpeer->h.type = SIXP_PKT_TYPE_RESPONSE;

    ok = sixp_pkt_output(hpeer, NRSF_SFID, nrsf_sent_callback_responder, NULL, 0);
    nrsf_report_sent(hpeer, "ADD response", ok);
    return sixpOK;
}

SIXPError nrsf_sixp_check_recv_response(SIXPeerHandle* hpeer){
    SIXPCellsHandle  hcells;
    hcells.cell_list        = (sixp_pkt_cell_t*)hpeer->h.body;
    hcells.cell_list_len    = hpeer->h.body_len;
    hcells.num_cells        = hcells.cell_list_len/sizeof(*hcells.cell_list);

    nrsf_avoid_cells(hpeer, &hcells);

    return sixpOK;
}

/*---------------------------------------------------------------------------*/
void nrsf_sent_callback_responder(void *arg, uint16_t arg_len,
                              const linkaddr_t *dest_addr,
                              sixp_output_status_t status)
{
  if(status == SIXP_OUTPUT_STATUS_SUCCESS) {
    LOG_INFO("transaction completes successfully\n");
  } else {
    LOG_ERR("transaction failed\n");
  }
}

void nrsf_sent_complete_responder(void *arg, uint16_t arg_len,
                              const linkaddr_t *dest_addr,
                              sixp_output_status_t status)
{
    nrsf_sent_callback_responder(arg, arg_len, dest_addr, status);
    (void)sixp_trans_transit_state(sixp_trans_now(), SIXP_TRANS_STATE_TERMINATING);
}

static
void nrsf_send_complete(SIXPeerHandle* hpeer){
    assert(hpeer->trans != NULL);
    if (hpeer->trans != NULL){
        LOG_INFO("transaction completes\n");
        (void)sixp_trans_transit_state(hpeer->trans, SIXP_TRANS_STATE_TERMINATING);
    }
    else
        LOG_ERR("!transaction loose for complete\n");
}

/*---------------------------------------------------------------------------*/
SIXPError nrsf_sixp_clear_recv_request(SIXPeerHandle* hpeer){
    assert(hpeer->addr != NULL);
    LOG_INFO("received a CLEAR request from ");
    LOG_INFO_LLADDR(hpeer->addr);
    LOG_INFO_("\n");
    msf_release_nbr_cells(tsch_queue_get_nbr(hpeer->addr));

    nrsf_send_complete(hpeer);

    return sixpOK;
}



//==============================================================================
#include "sys/ctimer.h"

static struct ctimer op_timer;
static void nrsf_tasks_exec();
static

void nrsf_tasks_poll(){
    ctimer_stop(&op_timer);
    ctimer_set(&op_timer, 0, nrsf_tasks_exec, NULL); /* expires immediately */
}

enum { nrsfTASK_PEERS_LIMIT = 4,
       nrsfTASK_LEN_LIMIT   = MSF_6P_CELL_LIST_MAX_LEN+1,
       nrsfOPS_LIMIT        = nrsfTASK_PEERS_LIMIT*nrsfTASK_LEN_LIMIT,
       nrsfOPS_CLEAN        = ~0ul, //< clean op item
};

//< index in nrsf_ops_list;
typedef signed char nrsf_ops_idx;
static sixp_cell_t ops_list[nrsfOPS_LIMIT];

static inline
SIXPCellsPkt*      nrsf_op_at(nrsf_ops_idx idx){
    assert(idx >= 0);
    assert(idx < nrsfOPS_LIMIT);
    return (SIXPCellsPkt*)(ops_list+idx);
}

struct nrsfTask{
    tsch_neighbor_t* nbr;
    // notify nbrs, except nbr, for added cells connect ->nbr
    nrsf_ops_idx     adds;
    // notify nbrs, except nbr, for released cells connect ->nbr
    nrsf_ops_idx     dels;
    // notify nbr for used cells
    nrsf_ops_idx     notify_use;
};
typedef struct nrsfTask nrsfTask;

typedef int nrsf_task_idx;
struct nrsfTask nrsf_tasks[nrsfTASK_PEERS_LIMIT];
//< this task is outstands from add/release
bool            nrsf_task_clear = false;

static void nrsf_tasks_clear();
static void nrsf_ops_clear();
static nrsf_task_idx nrsf_task_alloc(tsch_neighbor_t* nbr);

static
void nrsf_init_tasks(){
    LOG_DBG("nrsf init\n");
    memset(&op_timer, 0, sizeof(op_timer) );
    nrsf_tasks_clear();
    nrsf_task_clear = false;
    nrsf_ops_clear();
}

static
void nrsf_tasks_clear(){
    for (int i = 0; i < nrsfTASK_PEERS_LIMIT; ++i){
        nrsf_tasks[i].nbr = NULL;
    }
}

static
void nrsf_task_free(nrsfTask* t){
    t->nbr = NULL;
}

static
void nrsf_task_setup(nrsf_task_idx x, tsch_neighbor_t* nbr){
    assert(x >= 0);
    assert(x < nrsfTASK_PEERS_LIMIT);
    nrsfTask* t = nrsf_tasks+x;

    t->nbr = nbr;
    t->adds         = -1;
    t->dels         = -1;
    t->notify_use   = -1;
}

static
nrsf_task_idx nrsf_task_alloc(tsch_neighbor_t* nbr){
    nrsf_task_idx res = -1;
    for (int i = 0; i < nrsfTASK_PEERS_LIMIT; ++i){
        //peer tasks alredy have
        if (nrsf_tasks[i].nbr == nbr)
            return i;
        //alloc free task slot
        if (nrsf_tasks[i].nbr == NULL)
        if (res < 0)
            res = i;
    }
    if(res < 0)
        return -1;

    nrsf_task_setup(res, nbr);
    return res;
}

static
nrsf_task_idx nrsf_task_select(tsch_neighbor_t* nbr){
    for (int i = 0; i < nrsfTASK_PEERS_LIMIT; ++i){
        //peer tasks alredy have
        if (nrsf_tasks[i].nbr == nbr)
            return i;
    }
    return -1;
}

static
void nrsf_ops_clear(){
    for (int i = 0; i < nrsfOPS_LIMIT; ++i)
        ops_list[i].raw = nrsfOPS_CLEAN;
}

static
void nrsf_ops_free(nrsf_ops_idx x){
    if (x < 0)
        return;
    if (ops_list[(int)x].raw == nrsfOPS_CLEAN)
        return;

    SIXPCellsPkt* op = nrsf_op_at(x);
    unsigned len = op->head.num_cells;
    assert(len < nrsfTASK_LEN_LIMIT);

    for (int i = x; i <= x+len; ++i)
        ops_list[i].raw = nrsfOPS_CLEAN;
}

static
nrsf_ops_idx nrsf_ops_new(void){
    //find ops nrsfTASK_LEN_LIMIT clean seq
    nrsf_ops_idx res = 0;
    for (int i = 0; i < nrsfOPS_LIMIT; ){
        if (ops_list[i].raw != nrsfOPS_CLEAN){
            LOG_DBG("alloc: op[%d]= %x\n", i, (ops_list[i].raw) );
            // first element looks start of nrsfTASK_LEN_LIMIT bulk
            i   = i + nrsfTASK_LEN_LIMIT;
            res = i;
            continue;
        }
        if ((res - i) >= nrsfTASK_LEN_LIMIT)
            break;
        ++i;
    }
    if (res >= nrsfOPS_LIMIT)
        return -1;

    SIXPCellsPkt* op = nrsf_op_at(res);
    op->head.meta           = 0;
    op->head.cell_options   = SIXP_PKT_CELL_OPTION_TX;//for valid request
    op->head.num_cells      = 0;
    //ops_list[res].raw = nrsfOPS_INIT;
    return res;
}

static
nrsf_ops_idx nrsf_ops_alloc(nrsf_ops_idx x, unsigned append_num){
    if (x < 0){
        x = nrsf_ops_new();
        if (x < 0){
            LOG_ERR("fail alloc op\n");
            return -1;
        }
        LOG_DBG("alloc op[%d]\n", x);
    }
    return x;
}

static
bool nrsf_ops_remove_cell(nrsf_ops_idx x, tsch_link_t *cell, const char* name){
    if(x < 0)
        return false;

    SIXPCellsPkt* op = nrsf_op_at(x);
    sixp_cell_t* lastc = op->cells;
    sixp_cell_t* lookc = op->cells;
    sixp_cell_t  scanc = msf_cell_of_link(cell);

    for (unsigned i = op->head.num_cells; i > 0; --i, ++lookc){
        if (lookc->raw != scanc.raw){
            lastc->raw = lookc->raw;
            ++lastc;
        }
        else
            LOG_DBG("drop %s[%d] op[%d]=%x \n", name, x
                        , (lastc - (op->cells)), scanc.raw);
    }
    if (lastc != lookc) {
        op->head.num_cells = lastc - op->cells;
        return true;
    }
    return false;
}

static
nrsf_ops_idx nrsf_ops_append(nrsf_ops_idx x, tsch_link_t *cell, const char* name){
    x = nrsf_ops_alloc(x, 1);
    if(x < 0){
        return x;
    }
    SIXPCellsPkt* op = nrsf_op_at(x);
    if ( (op->head.num_cells + 1) >= (nrsfTASK_LEN_LIMIT-1) ) {
        LOG_ERR("fail append %s op[%d+%d]\n", name, x, op->head.num_cells);
        return -1;
    }

    op->cells[op->head.num_cells] = msf_cell_of_link(cell);
    LOG_DBG("append %s[%d] op[%u]=%x\n", name, x
                ,op->head.num_cells, op->cells[op->head.num_cells].raw);
    ++(op->head.num_cells);
    return x;
}


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

void nrsf_on_msf_use_cell(tsch_neighbor_t *nbr, tsch_link_t *cell){
    if (nbr == NULL) {
        nbr = get_addr_nbr(&cell->addr);
        if (nbr == NULL)
            return;
    }

    if ( msf_is_avoid_local_cell(msf_cell_of_link(cell)) )
            return;

    LOG_DBG("nrsf_on_msf_use_cell %u.%u\n", cell->timeslot, cell->channel_offset);
    nrsf_task_idx ti = nrsf_task_alloc(nbr);
    if (ti < 0){
        LOG_ERR("fail alloc ADD task\n");
        return;
    }
    nrsfTask* t = nrsf_tasks + ti;

    // compensate oposite op, if it exists
    if ( nrsf_ops_remove_cell(t->dels, cell, "REL") )
        return;

    // append new cell to task ops
    nrsf_ops_idx tmp = nrsf_ops_append(t->adds, cell, "ADD");
    if (tmp >= 0){
        t->adds = tmp;
        nrsf_tasks_poll();
    }
}

void nrsf_on_msf_release_cell(tsch_neighbor_t *nbr, tsch_link_t *cell){
    if (nbr == NULL) {
        nbr = get_addr_nbr(&cell->addr);
        if (nbr == NULL)
            return;
    }

    if ( msf_is_avoid_local_cell(msf_cell_of_link(cell)) )
            return;

    LOG_DBG("nrsf_on_msf_release_cell %u.%u\n", cell->timeslot, cell->channel_offset);

    nrsf_task_idx ti = nrsf_task_alloc(nbr);
    if (ti < 0){
        LOG_ERR("fail alloc RELEASE task\n");
        return;
    }
    nrsfTask* t = nrsf_tasks + ti;

    // compensate oposite op, if it exists
    if ( nrsf_ops_remove_cell(t->adds, cell, "ADD") )
        return;

    if (t->notify_use >= 0){
        // released cells now not need notify nbr
        nrsf_ops_remove_cell(t->notify_use, cell, "USE");
    }

    // append new cell to task ops
    nrsf_ops_idx tmp = nrsf_ops_append(t->dels, cell, "REL");
    if (tmp >= 0){
        t->dels = tmp;
        nrsf_tasks_poll();
    }
}

void nrsf_on_msf_nbr_clean(tsch_neighbor_t *nbr){
    // cleanup nbrs only
    if (nbr != NULL) {
        nrsf_task_idx ti = nrsf_task_select(nbr);
        if (ti < 0)
            return;
        LOG_DBG("nrsf_on_msf_clean task[%d]\n", ti);
        nrsf_task_free(nrsf_tasks+ti);
        return;
    }
    LOG_DBG("nrsf_on_msf_clean\n");
    nrsf_tasks_clear();
    //nrsf_task_clear = true;
    //nrsf_tasks_poll();
}

void nrsf_on_msf_new_nbr(tsch_neighbor_t *nbr){
    LOG_DBG("nrsf_on_msf_new_nbr ");
    LOG_INFO_LLADDR(tsch_queue_get_nbr_address(nbr));
    LOG_DBG_("\n");

    int num_voids = msf_avoid_num_local_cells();
    if (num_voids <= 0)
        return;

    nrsf_task_idx ti = nrsf_task_alloc(nbr);
    if (ti < 0){
        LOG_ERR("fail alloc USE task\n");
        return;
    }
    nrsfTask* t = nrsf_tasks + ti;

    if (t->notify_use >= 0){
        LOG_WARN("task alredy have USES!\n");
        return;
    }
    t->notify_use = nrsf_ops_alloc(t->notify_use, nrsfTASK_LEN_LIMIT-1);
    if(t->notify_use < 0){
        return;
    }

    SIXPCellsPkt* op = nrsf_op_at(t->notify_use);

    // report about all avoid and negotiated cells
    int ok = msf_avoid_enum_local_cells( op, nrsfTASK_LEN_LIMIT-1);
    if (ok > 0)
        ok = msf_avoid_clean_cells_for_nbr(op, nbr);

        ok+= msf_negotiated_enum_cells_besides_nbr(nbr, op, nrsfTASK_LEN_LIMIT-1);
    if (ok > 0){
        nrsf_tasks_poll();
    }
}

void nrsf_on_6ptrans_free(void){
    // after 6P transactions are completes, try exec NRSF actions
    if (!sixp_trans_any())
        nrsf_tasks_poll();
}

void nrsf_tasks_exec(){
    if (sixp_trans_any()){
        // exec only when no active transactions
        return;
    }

    LOG_DBG("nrsf_tasks_exec\n");
    if(0)// clearing for self alredy done by MSF, NRSF no need any job here
    if (nrsf_task_clear){
        nrsf_clean_cells_by_nbr(NULL);
        nrsf_task_clear = false;
        nrsf_tasks_poll();
        return;
    }

    // TODO: if have ability to pack multiple 6P commands/packet
    //      maybe more efficient to traverse over nbrs whole tasks - composing
    //      multiple commands for nbr

    for (int i = 0; i < nrsfTASK_PEERS_LIMIT; ++i)
    if (nrsf_tasks[i].nbr != NULL) {
        nrsfTask* t = nrsf_tasks + i;
        int ok = 0;

        if (t->notify_use >= 0) {
            ok = nrsf_notify_cells_2nbr(t->nbr, nrsf_op_at(t->notify_use));
            nrsf_ops_free(t->notify_use);
            t->notify_use = -1;
        }

        if (ok <= 0)
        if (t->dels >= 0) {
            ok = nrsf_release_cells_by_nbr(t->nbr, nrsf_op_at(t->dels));
            nrsf_ops_free(t->dels);
            t->dels = -1;
        }

        if (ok <= 0)
        if (t->adds >= 0){
            ok = nrsf_used_cells_by_nbr(t->nbr, nrsf_op_at(t->adds));
            nrsf_ops_free(t->adds);
            t->adds = -1;
        }

        if (ok <= 0) {
            nrsf_task_free(t);
            continue;
        }

        nrsf_tasks_poll();
        return;
    }
}


#include "net/ipv6/uip-ds6-route.h"
#include "net/mac/tsch/tsch-queue.h"
#include "net/mac/tsch/sixtop/sixp-nbr.h"

typedef void (*nbr_op)(const linkaddr_t *peer_addr, SIXPCellsPkt* hcells);

static
int nrsf_cells_to_all_nbrs(tsch_neighbor_t *skip_nbr, SIXPCellsPkt* cells, nbr_op f
                            , const char* info)
{
    if (cells->cells[-1].raw == nrsfOPS_CLEAN)
        return 0;
    if (cells->head.num_cells <= 0)
        return 0;

    int cnt = 0;
    // WARN: TODO: nbr lladr uses as a key in nbr table.
    linkaddr_t* nbr_key = tsch_queue_get_nbr_address(skip_nbr);
    LOG_DBG("%s hops /# [", info);
    LOG_INFO_LLADDR(nbr_key);
    LOG_DBG_("]\n");

#if 0
    // Does any other child need this notify (lookup all route next hops)
    nbr_table_item_t *item = nbr_table_head(nbr_routes);
    while(item != NULL) {
      linkaddr_t* idx = nbr_routes_lladr_item(item);
      if (idx == NULL) break;

      LOG_DBG("route[%u] -> hop[");
      LOG_INFO_LLADDR(idx);
      LOG_DBG_("]\n");

      // skip nbr
      if (idx != nbr_key) {
          f(idx, cells );
          ++cnt;
      }

      item = nbr_table_next(nbr_routes, item);
    }

    // Does any other defaults need this notify (lookup all route next hops)
    uip_ds6_defrt_t* ditem = uip_ds6_defrt_head();
    while(item != NULL) {
        linkaddr_t* idx = (linkaddr_t*)uip_ds6_nbr_lladdr_from_ipaddr(&(ditem->ipaddr));
        if (idx == NULL) break;

        LOG_DBG("default[%u] -> hop[");
        LOG_INFO_LLADDR(idx);
        LOG_DBG_("]\n");

        // skip nbr
        if (idx != nbr_key) {
            f(idx, cells );
            ++cnt;
        }

        item = list_item_next(ditem);
    }
#elif 0
    // Does any other child need this notify (lookup all route next hops)
    sixp_nbr_t *item = sixp_nbr_head();
    while(item != NULL) {
      linkaddr_t* idx = &item->addr;

      LOG_DBG("route[%u] -> hop[");
      LOG_INFO_LLADDR(idx);
      LOG_DBG_("]\n");

      bool ok = (nbr_key == NULL);
      if (!ok)
          ok = !linkaddr_cmp(idx, nbr_key);
      // skip nbr

      if (ok)
      {
          f(idx, cells );
          ++cnt;
      }

      item = sixp_nbr_next(item);
    }
#else
    struct tsch_neighbor* item = tsch_neighbors_head();
    for (; item != NULL
         ; item = tsch_neighbors_next(item))
    {
      if (item == n_eb)         continue;
      if (item == n_broadcast)  continue;

      linkaddr_t* idx = tsch_queue_get_nbr_address(item);

      bool ok = (nbr_key == NULL);
      if (!ok)
          ok = !linkaddr_cmp(idx, nbr_key);
      // skip nbr

      if (ok)
      {
          f(idx, cells );
          ++cnt;
      }
    }
#endif

    return cnt;
}

int nrsf_used_cells_by_nbr(tsch_neighbor_t *nbr, SIXPCellsPkt* cells){
    return nrsf_cells_to_all_nbrs(nbr, cells, nrsf_sixp_add_send_request, "USED");
}

int nrsf_release_cells_by_nbr(tsch_neighbor_t *nbr, SIXPCellsPkt* cells){
    return nrsf_cells_to_all_nbrs(nbr, cells, nrsf_sixp_del_send_request, "REL");
}

int nrsf_notify_cells_2nbr(tsch_neighbor_t *nbr, SIXPCellsPkt* cells){
    LOG_DBG("nrsf_notify_cells_2nbr\n");
    if (cells->cells[-1].raw == nrsfOPS_CLEAN)
        return 0;
    if (cells->head.num_cells <= 0)
        return 0;
    nrsf_sixp_add_send_request(tsch_queue_get_nbr_address(nbr), cells);
    return 1;
}

int nrsf_clean_cells_by_nbr(tsch_neighbor_t *nbr){
    return nrsf_cells_to_all_nbrs(nbr, NULL, nrsf_sixp_clear_send_request, "CLEAN");
}



int msf_negotiated_enum_cells_besides_nbr(tsch_neighbor_t *nbr
                            , SIXPCellsPkt* pkt, unsigned limit)
{
    if ( pkt->head.num_cells >= limit)
        return -1;

    tsch_slotframe_t *slotframe = msf_negotiated_cell_get_slotframe();
    int res = 0;

    tsch_link_t *cell;
    for(cell = list_head(slotframe->links_list);
        cell != NULL;
        cell = list_item_next(cell))
    {
        if ((cell->link_options & (LINK_OPTION_RESERVED_LINK|LINK_OPTION_LINK_TO_DELETE))!= 0)
            continue;
        if ( nbr == tsch_queue_get_nbr(&cell->addr))
            continue;

        if ( pkt->head.num_cells < limit){
            msf_cell_t* x = pkt->cells + pkt->head.num_cells;
            x->field.slot   = cell->timeslot;
            x->field.chanel = cell->channel_offset;
            ++res;
            ++pkt->head.num_cells;
            if (pkt->head.num_cells >= limit)
                return res;
        }
    }

    return res;
}
