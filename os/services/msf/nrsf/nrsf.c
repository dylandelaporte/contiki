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
static void nrsf_tasks_poll_use();
static void nrsf_tasks_poll_del();



//=============================================================================
/*
 * @return - amount of modify/new cells
 * TODO: (ru) проблема возникает когда несколько нодов используют одну €чейку -
 *          текущее решение - хранитс€ использование локальное + ближайший сосед.
 * TODO:    Ќеобходимо детектирование конкуренции за одну €чейку, и политика
 *              релокации €чеек, если обнаруживаетс€ конкуренци€.
 *     ¬озможные решени€:
 *      - согласовывать новую €чейку, если идет конкуренци€ за автономную TX €чейку
 *        должен ли так разруливатьс€ конфликт за RX€чейку?
 *        кто должен инициировать согласование?
 */
void nrsf_avoid_cells(SIXPeerHandle* hpeer, SIXPCellsHandle* hcells){
     tsch_neighbor_t* n = tsch_queue_get_nbr(hpeer->addr);

    NRSFMeta meta;
    meta.raw = hcells->meta;
    unsigned avoiduse = meta.field.avoid_use & aoUSE_REMOTE;
    if (avoiduse > NRSF_RANGE_HOPS*aoUSE_REMOTE_1HOP ){
        LOG_DBG("ingnore hop%x cells\n", avoiduse);
        return;
    }
    LOG_DBG("avoid hop%x cells[%u]:\n", avoiduse, hcells->num_cells);

    avoiduse += aoUSE_REMOTE_1HOP;

    //this set of options describe cell passes for conflict checks
    unsigned cell_avoidopt = (meta.field.avoid_use & ~aoUSE)
                            | (avoiduse & aoUSE_REMOTE)
                            | (hcells->cell_options & aoTX);

    int rel = 0;
    for (unsigned i = 0; i < hcells->num_cells; ++i){
        sixp_cell_t c = sixp_pkt_get_cell(hcells->cell_list, i);
        AvoidResult should_relay;

        unsigned avoid_fixed = 0;
        if (i < meta.field.fixed_cnt)
            avoid_fixed = aoFIXED;

        should_relay = msf_avoid_nbr_use_cell(c, n, avoiduse | avoid_fixed);

        if (avoiduse <= NRSF_RANGE_HOPS*aoUSE_REMOTE_1HOP )
        //if (avoiduse < aoUSE_REMOTE_3HOP)
        if (should_relay >= arEXIST_CHANGE)
            ++rel;

        if (should_relay >= arEXIST_CHANGE){
            msf_housekeeping_inspect_cell_consintensy(c, n, cell_avoidopt );
        }
    }

    // start DPC to flush new avoids
    if (rel > 0)
        nrsf_tasks_poll_use();
}

/*
 * here reaction on external request for unused cells.
 *  we unuse remote cell too, and if it is not far - relay it
 *
 * TODO: (ru) проблема возникает когда несколько нодов используют одну €чейку -
 *         «апрос может удалить не свою €чейку. надо вы€вл€ть этот конфликт.
 *         —уществующее решение - более дальн€€ команда игнорируетс€,
 *                  а при удалении ближней €чейки, система остаетс€ без информации
 *                    о дальней.
 *          ¬озможный выход - при приходе запроса удалени€ на локальную €чейку,
 *              форсировать отправку команды еЄ зан€тости - это обновит у соседей
 *              актуальную зан€тость €чейки.
 */
void nrsf_unvoid_cells(SIXPeerHandle* hpeer, SIXPCellsHandle* hcells){
    int avoiduse;
    int rel = 0;

    tsch_neighbor_t *n = tsch_queue_get_nbr(hpeer->addr);
    for (unsigned i = 0; i < hcells->num_cells; ++i){
        sixp_cell_t c = sixp_pkt_get_cell(hcells->cell_list, i);

        //drop remote cell in NRSF_RANGE_HOPS, and unuse if far cells
        avoiduse = msf_unvoid_drop_nbr_cell(c, n, NRSF_RANGE_HOPS*aoUSE_REMOTE_1HOP );
        if (avoiduse > arEXIST_CHANGE)
            ++rel;
    }

    // start DPC flush new avoids
    if (rel > 0)
        nrsf_tasks_poll_del();

}

void nrsf_check_cells(SIXPeerHandle* hpeer, SIXPCellsHandle* hcells){
    unsigned j = 0;
    for (unsigned i = 0; i < hcells->num_cells; ++i){
        sixp_cell_t x = sixp_pkt_get_cell(hcells->cell_list, i);
        if (msf_is_avoid_cell(x) < 0)
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
    assert(hpeer->addr != NULL);

    SIXPCellsHandle  hcells;
    SIXPError ok1 = sixp_pkt_parse_cells(&hpeer->h, &hcells);
    if (ok1 != sixpOK)
        return ok1;

    //bool is_single_cell = sixp_pkt_is_single_cell(&hpeer->h, hcells);
    for (SIXPError ok = ok1; ok == sixpOK
        ; ok = sixp_pkt_parse_next_cells(&hpeer->h, &hcells))
    {
        if(is_valid_add_request(&hcells)) {
          nrsf_avoid_cells(hpeer, &hcells);
        }
        else{
            LOG_DBG("pkt:%x*%d > %x,%x\n"
                    , hcells.meta, hcells.num_cells
                    , hcells.cell_list[-1], hcells.cell_list[0]);
            return sixpFAIL;
        }
    }

    // no response for NRSF commands here.
    return ok1;
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
    nrsf_report_sent(hpeer, "CHECK response", ok);
    return sixpOK;
}

SIXPError nrsf_sixp_check_recv_response(SIXPeerHandle* hpeer){
    SIXPCellsHandle  hcells;
    hcells.cell_list        = (sixp_pkt_cell_t*)hpeer->h.body;
    hcells.cell_list_len    = hpeer->h.body_len;
    hcells.num_cells        = hcells.cell_list_len/sizeof(*hcells.cell_list);

    SIXPError ok = sixp_pkt_parse_cells(&hpeer->h, &hcells);
    if (ok == sixpOK)
    if(is_valid_add_request(&hcells))
        nrsf_avoid_cells(hpeer, &hcells);

    return ok;
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

enum { nrsfTASK_PEERS_LIMIT = 4,
       //< limit of nrsf packet size
       nrsfREQ_OPS_LIMIT    = 100/sizeof(sixp_cell_t) ,
       nrsfOPS_CLEAN        = ~0ul, //< clean op item

};

struct nrsfTask{
    tsch_neighbor_t* nbr;
    // notify nbr for used cells
    int             notify_use;
};
typedef struct nrsfTask nrsfTask;

typedef int nrsf_task_idx;
struct nrsfTask nrsf_tasks[nrsfTASK_PEERS_LIMIT];
//< this task is outstands from add/release
bool            nrsf_task_clear = false;
nrsfTask        nrsf_task_use;
nrsfTask        nrsf_task_del;

static void nrsf_tasks_clear();
static nrsf_task_idx nrsf_task_alloc(tsch_neighbor_t* nbr);



static struct ctimer op_timer;
static void nrsf_tasks_exec();
static void nrsf_tasks_poll();

static
void nrsf_tasks_poll(){
    ctimer_stop(&op_timer);
    ctimer_set(&op_timer, 0, nrsf_tasks_exec, NULL); /* expires immediately */
}

static
void nrsf_tasks_poll_use()
{
    if (nrsf_task_use.notify_use < 0){
        nrsf_task_use.notify_use = aoUSE_LOCAL;//REMOTE_1HOP;
        LOG_DBG("use task %p:%x\n", nrsf_task_use.nbr, nrsf_task_use.notify_use);
    }
    nrsf_tasks_poll();
}

static
void nrsf_tasks_poll_del()
{
    if (nrsf_task_del.notify_use < 0){
        nrsf_task_del.notify_use = aoDROPED|aoMARK;
        LOG_DBG("del task %p-%x\n", nrsf_task_del.nbr, nrsf_task_del.notify_use);
    }
    nrsf_tasks_poll();
}


static
void nrsf_init_tasks(){
    LOG_DBG("nrsf init\n");
    memset(&op_timer, 0, sizeof(op_timer) );
    nrsf_tasks_clear();
}

static
void nrsf_task_free(nrsfTask* t){
    t->nbr = NULL;
    t->notify_use   = -1;
}

static
void nrsf_tasks_clear(){
    for (int i = 0; i < nrsfTASK_PEERS_LIMIT; ++i){
        nrsf_tasks[i].nbr = NULL;
    }
    nrsf_task_free(&nrsf_task_use);
    nrsf_task_free(&nrsf_task_del);
    nrsf_task_clear = false;
}

static
void nrsf_task_setup(nrsf_task_idx x, tsch_neighbor_t* nbr){
    assert(x >= 0);
    assert(x < nrsfTASK_PEERS_LIMIT);
    nrsfTask* t = nrsf_tasks+x;

    t->nbr = nbr;
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



//------------------------------------------------------------------------------
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


void nrsf_on_msf_use_link_cell(tsch_neighbor_t *nbr, tsch_link_t *cell){
    /*if (nbr == NULL) {
        nbr = get_addr_nbr(&cell->addr);
        if (nbr == NULL)
            return;
    }*/
    nrsf_on_msf_use_cell( NULL, msf_cell_of_link(cell));
}

void nrsf_on_msf_use_cell(tsch_neighbor_t *nbr, sixp_cell_t cell)
{
    //assert(nbr != NULL);
    if ( msf_is_avoid_local_cell(cell) >= 0 )
            return;

    LOG_DBG("nrsf_on_msf_use_link_cell %u.%u\n", cell.field.slot, cell.field.chanel);
    nrsf_tasks_poll_use();
}

void nrsf_on_msf_release_link_cell(tsch_neighbor_t *nbr, tsch_link_t *cell){
    if (nbr == NULL) {
        nbr = get_addr_nbr(&cell->addr);
        if (nbr == NULL)
            return;
    }
    nrsf_on_msf_release_cell(nbr, msf_cell_of_link(cell));
}

void nrsf_on_msf_release_cell(tsch_neighbor_t *nbr, sixp_cell_t cell)
{
    if (msf_is_avoid_local_cell(cell) >= 0 ){
        // autonomous cells may are not release imidiate
        return;
    }

    LOG_DBG("nrsf_on_msf_release_cell %u.%u\n", cell.field.slot, cell.field.chanel);
    msf_avoid_nbr_use_cell(cell, nbr, aoDROPED);
    nrsf_tasks_poll_del();
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

    const unsigned enum_range = (NRSF_RANGE_HOPS*aoUSE_REMOTE_1HOP) | aoMARK;
    int num_voids = msf_avoid_num_cells_in_range( enum_range );
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
    //< start enuerate from
    t->notify_use = aoUSE_LOCAL | aoMARK;
    nrsf_tasks_poll();
}

void nrsf_on_6ptrans_free(void){
    // after 6P transactions are completes, try exec NRSF actions
    if (!sixp_trans_any())
        nrsf_tasks_poll();
}



//------------------------------------------------------------------------------
// @return amount of nrsf requests sended
typedef int (*task_op)(nrsfTask* t, const char* info);
static int nrsf_task_to_all_nbrs( task_op f, nrsfTask* t, int use, const char* info);

static int nrsf_task_notify_cells(nrsfTask* t, const char* info);
static int nrsf_task_notify_dels(nrsfTask* t, const char* info);

void nrsf_tasks_exec(){
    LOG_DBG("nrsf_tasks_exec\n");

    // TODO: if have ability to pack multiple 6P commands/packet
    //      maybe more efficient to traverse over nbrs whole tasks - composing
    //      multiple commands for nbr

    int ok = 0;
    for (int i = 0; i < nrsfTASK_PEERS_LIMIT; ++i)
    if (nrsf_tasks[i].nbr != NULL) {
        nrsfTask* t = nrsf_tasks + i;

        if (t->notify_use >= 0) {
            ok = nrsf_task_notify_cells(t, "NOTIFY");
        }

        if (ok <= 0) {
            continue;
        }

        nrsf_tasks_poll();
        return;
    }

    // USE/DEL action can only inform all nbr together, so start them when 6P is clear.
    if (sixp_trans_any()){
        // exec only when no active transactions
        return;
    }

    if (ok <= 0)
    if (nrsf_task_del.notify_use >= 0){
        ok = nrsf_task_to_all_nbrs( nrsf_task_notify_dels
                                , &nrsf_task_del, aoDROPED|aoMARK
                                , "DEL"
                                );
        msf_release_unused();
    }

    if (ok <= 0)
    if (nrsf_task_use.notify_use >= 0){
        ok = nrsf_task_to_all_nbrs( nrsf_task_notify_cells
                                , &nrsf_task_use, aoUSE_LOCAL
                                , "USE"
                                );
        msf_avoid_mark_all_fresh();
    }

    if (ok > 0)
        nrsf_tasks_poll();
}

// @return total op cells, filled
static
int nrsf_build_task_local_cells(nrsfTask* t, SIXPCellsPkt* op, unsigned lim){
    //sixp_pkt_cells_reset(op);
    op->head.cell_options = SIXP_PKT_CELL_OPTION_TX;

    // report about all local TX cells
    // op    :fixed              :n
    // [head]:[ cells fixed ....]:[cells ...]
    int fixed = op->head.num_cells;
    msf_avoid_enum_cells(op, lim-1, t->notify_use | aoTX | aoFIXED, t->nbr);
    int n     = op->head.num_cells;
    msf_avoid_enum_cells(op, lim-1, t->notify_use | aoTX          , t->nbr);

    fixed = n - fixed;
    n = op->head.num_cells - n;

    if ( (n+fixed) > 0){
        NRSFMeta meta;
        meta.raw = 0;
        meta.field.avoid_use = t->notify_use;
        meta.field.fixed_cnt = fixed;
        op->head.meta = meta.raw;
        LOG_DBG("notify avoid local TX fix%d+%d cells ", fixed, n);
        if(LOG_LEVEL > LOG_LEVEL_DBG){
            sixp_pkt_cells_dump(op);
        }
        LOG_DBG_("\n");
    }

    // compose RX/TX lists in ope op:
    // op    :fixed              :n          :rop   :rfixed             :rn
    // [head]:[ cells fixed ....]:[cells ...]:[head]:[ cells fixed ....]:[cells ...]

    //reserve next - RX list head
    SIXPCellsPkt* rop = op;
    if(op->head.num_cells > 0) {
        rop = (SIXPCellsPkt*)(op->cells + op->head.num_cells);
        //alocate space for head
        op->cells[op->head.num_cells].raw = MSF_NOCELL;
        ++(op->head.num_cells);
    }

    // appends report about all local RX cells
    int rfixed = op->head.num_cells;
    msf_avoid_enum_cells(op, lim-1, t->notify_use | aoFIXED, t->nbr);
    int rn     = op->head.num_cells;
    msf_avoid_enum_cells(op, lim-1, t->notify_use          , t->nbr);

    int total = op->head.num_cells;
    rfixed = rn - rfixed;
    rn = op->head.num_cells - rn;

    if (rop != op) {
        //split rx-op from tail of common op
        sixp_pkt_cells_reset(rop);
        rop->head.num_cells = rfixed + rn;
        op->head.num_cells = ((sixp_cell_t*)rop - (op->cells));
    }

    rop->head.cell_options = SIXP_PKT_CELL_OPTION_RX;
    if ( (rn+rfixed) > 0){
        NRSFMeta meta;
        meta.raw = 0;
        meta.field.avoid_use = t->notify_use;
        meta.field.fixed_cnt = rfixed;
        rop->head.meta = meta.raw;
        LOG_DBG("notify avoid local RX fix%d+%d cells ", rfixed, rn);
        if(LOG_LEVEL > LOG_LEVEL_DBG){
            sixp_pkt_cells_dump(rop);
        }
        LOG_DBG_("\n");
    }

    return total;
}

int nrsf_build_task_cells(nrsfTask* t, SIXPCellsPkt* op, unsigned limit){
    int total = 0;
    if (t->notify_use < aoUSE_REMOTE_1HOP) {
        // report about all local avoid and negotiated cells
        total = nrsf_build_task_local_cells(t, op, limit) + 1;
        if (op->head.num_cells == 0){
            total = 0; //op->head.num_cells + 1; // enum header space
        }
        t->notify_use = (t->notify_use& ~aoUSE) | aoUSE_REMOTE_1HOP;
    }

    const unsigned limit_range = (NRSF_RANGE_HOPS*aoUSE_REMOTE_1HOP);
    for (; (t->notify_use & aoUSE) <= limit_range //aoUSE_REMOTE_3HOP
         ; t->notify_use += aoUSE_REMOTE_1HOP )
    {
        int lim = limit-total;
        if (lim <= 0)
            break;

        // report about all remote hop cells
        int n = msf_avoid_enum_cells(NULL, lim-1, t->notify_use, t->nbr);
        if (n <= 0)
            continue;
        LOG_DBG("notify avoid remote[%x] %d cells\n", t->notify_use, n);
        if (n >= lim)
            break;

        SIXPCellsPkt* hop = (SIXPCellsPkt*)(op+total);
        sixp_pkt_cells_reset(hop);
        hop->head.cell_options = SIXP_PKT_CELL_OPTION_TX;

        int fixed = op->head.num_cells;
        msf_avoid_enum_cells( hop, lim-1, t->notify_use | aoFIXED, t->nbr);
        n = op->head.num_cells;
        msf_avoid_enum_cells( hop, lim-1, t->notify_use, t->nbr);

        fixed = n - fixed;
        n = op->head.num_cells - n;

        if(hop->head.num_cells > 0)
            total += hop->head.num_cells + 1;//append cells with header
        else
        //if ( (n+fixed) <=0)
            continue;

        NRSFMeta meta;
        meta.raw = 0;
        meta.field.avoid_use = t->notify_use;
        meta.field.fixed_cnt = fixed;
        hop->head.meta = meta.raw;

    }
    if ((t->notify_use & aoUSE) >= limit_range)
        t->notify_use = -1;

    return total;
}

int nrsf_task_notify_cells(nrsfTask* t, const char* info){

    const linkaddr_t *peer_addr = tsch_queue_get_nbr_address(t->nbr);
    if (sixp_trans_find(peer_addr) != NULL){
        return 0;
    }

    SIXPeerHandle hpeer;
    sixp_cell_t ops[nrsfREQ_OPS_LIMIT];
    SIXPCellsPkt* op = (SIXPCellsPkt*)ops;
    sixp_pkt_cells_reset(op);
    op->head.cell_options = SIXP_PKT_CELL_OPTION_TX;

    LOG_DBG("%s task from %x\n", info, t->notify_use);

    int ok;
    ok = nrsf_build_task_cells(t, op, nrsfREQ_OPS_LIMIT-1);
    if (ok <= 1)
        return 0;

    sixp_pkt_cells_assign(&hpeer.h, op);
    hpeer.h.body_len  = sizeof(sixp_cell_t)*ok;
    hpeer.h.code.cmd  = SIXP_PKT_CMD_ADD;
    hpeer.addr        = peer_addr;
    nrsf_sixp_single_send_request(&hpeer, info);
    return 1;
}

int nrsf_task_notify_dels(nrsfTask* t, const char* info){
    SIXPeerHandle hpeer;
    sixp_cell_t ops[nrsfREQ_OPS_LIMIT];
    SIXPCellsPkt* op = (SIXPCellsPkt*)ops;
    sixp_pkt_cells_reset(op);
    op->head.cell_options = SIXP_PKT_CELL_OPTION_TX;

    int ok;
    ok = msf_avoid_enum_cells(op, nrsfREQ_OPS_LIMIT-1, aoDROPED|aoMARK, t->nbr);
    LOG_DBG("%s task from %x =%d\n", info, t->notify_use, ok);

    t->notify_use = -1;
    if (ok <= 0)
        return 0;

    sixp_pkt_cells_assign(&hpeer.h, op);
    hpeer.h.body_len  = sizeof(sixp_cell_t)*(ok+1);
    hpeer.h.code.cmd  = SIXP_PKT_CMD_DELETE;
    hpeer.addr        = tsch_queue_get_nbr_address(t->nbr);
    nrsf_sixp_single_send_request(&hpeer, info);
    return 1;
}



#include "net/mac/tsch/tsch-queue.h"

static
int nrsf_task_to_all_nbrs( task_op f, nrsfTask* t, int use, const char* info)
{
    int cnt = 0;

    if (t->nbr == NULL)
        t->nbr = tsch_neighbors_head();

    struct tsch_neighbor* item = t->nbr;
    for (; item != NULL
         ; item = tsch_neighbors_next(item), t->nbr = item )
    {
      if (item == n_eb)         continue;
      if (item == n_broadcast)  continue;

      LOG_DBG("%s hops /# [", info);
      LOG_INFO_LLADDR( tsch_queue_get_nbr_address(item) );
      LOG_DBG_("]\n");

      //if (ok)
      {
          t->nbr        = item;
          if (t->notify_use < 0)
              t->notify_use = use;
          cnt += f(t, info);
          if (t->notify_use >= 0){
              LOG_DBG("task next exec for %x\n", t->notify_use);
              //task not finished, continue it next exec
              return cnt;
          }
      }
    }
    t->nbr = NULL;

    return cnt;
}


//------------------------------------------------------------------------------
//                              LEGACY
#include "net/ipv6/uip-ds6-route.h"
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
