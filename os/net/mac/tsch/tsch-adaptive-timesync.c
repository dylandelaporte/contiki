/*
 * Copyright (c) 2015, SICS Swedish ICT.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         TSCH adaptive time synchronization
 * \author
 *         Atis Elsts <atis.elsts@sics.se>
 *
 */

/**
  * \addtogroup tsch
  * @{
*/

#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-conf.h"
#include "net/mac/tsch/tsch-adaptive-timesync.h"
#include "net/mac/tsch/tsch-log.h"
#include <stdio.h>
#include <inttypes.h>

#if TSCH_ADAPTIVE_TIMESYNC

/* Estimated drift of the time-source neighbor. Can be negative.
 * Units used: ppm multiplied by 256. */
static int32_t drift_ppm;
/* Ticks compensated locally since the last timesync time */
static int32_t compensated_ticks;
/* Number of already recorded timesync history entries */
static uint8_t timesync_entry_count;
/* Since last learning of the  drift; may be more than time since last timesync */
static uint32_t asn_since_last_learning;
/* The last neighbor used for timesync */
struct tsch_neighbor *last_timesource_neighbor;

/* Units in which drift is stored: ppm * 256 */
#define TSCH_DRIFT_UNIT (1000L * 1000 * 256)

/*---------------------------------------------------------------------------*/
long int
tsch_adaptive_timesync_get_drift_ppm(void)
{
  return (long int)drift_ppm / 256;
}
/*---------------------------------------------------------------------------*/
/* Add a value to a moving average estimator */
static int32_t
timesync_entry_add(int32_t val, uint32_t time_delta)
{
#define NUM_TIMESYNC_ENTRIES 8
  static int32_t buffer[NUM_TIMESYNC_ENTRIES];
  static uint8_t pos;
  int i;
  if(timesync_entry_count == 0) {
    pos = 0;
  }
  buffer[pos] = val;
  if(timesync_entry_count < NUM_TIMESYNC_ENTRIES) {
    timesync_entry_count++;
  } else {
    /* We now have accurate drift compensation.
     * Increase keep-alive timeout. */
    tsch_set_ka_timeout(TSCH_MAX_KEEPALIVE_TIMEOUT);
  }
  pos = (pos + 1) % NUM_TIMESYNC_ENTRIES;

  val = 0;
  for(i = 0; i < timesync_entry_count; ++i) {
    val += buffer[i];
  }
  return val / timesync_entry_count;
}
/*---------------------------------------------------------------------------*/
#if TSCH_DRIFT_SYNC_ESTIMATE

#define DRIFT_AVG2  2
#define TSCH_DRIFT_AVG_UNIT (256L)
static
uint32_t drift_accu = 0;

static inline
uint32_t drift1t_avg() {return drift_accu>>DRIFT_AVG2;};

void drift1t_reset(int x){
    drift_accu = (x) << DRIFT_AVG2;
}

void    drift1t_append(uint32_t error_time, uint32_t time_delta_asn) {
    TSCH_LOGF("drift %ld/256 [asn/tick]\n", error_time);
    uint32_t avg_time = drift1t_avg();
    if ((avg_time > 0) && (avg_time < error_time)) {
        drift_accu -= avg_time;
        drift_accu += error_time;
    }
    else {
        //on error_time lower then avg, just force avg to lowest value
        drift_accu = error_time << DRIFT_AVG2;
    }
}

// estimates maximum time for keep in sync, update must be turn around in this time
// \return timeout [ASN]
int tsch_timesync_estimate_sync_timeout(){
    uint32_t error_limit_ticks = tsch_timing[tsch_ts_rx_wait]/2;
    //* take a safety gap for error limit estimate
    error_limit_ticks -= error_limit_ticks/4;
    if (timesync_entry_count < NUM_TIMESYNC_ENTRIES)
        error_limit_ticks = error_limit_ticks / 4;
    return (drift1t_avg()*error_limit_ticks)/TSCH_DRIFT_AVG_UNIT;
}
#else
#define drift1t_append(x) (void)x
#define drift1t_reset(x)
#endif //TSCH_DRIFT_SYNC_ESTIMATE

/*---------------------------------------------------------------------------*/
/* Learn the neighbor drift rate at ppm */
static void
timesync_learn_drift_ticks(uint32_t time_delta_asn, int32_t drift_ticks)
{
  /* should fit in a 32-bit integer */
  int32_t time_delta_ticks = time_delta_asn * tsch_timing[tsch_ts_timeslot_length];
  int32_t real_drift_ticks = drift_ticks + compensated_ticks;
  int32_t last_drift_ppm = (int32_t)((int64_t)real_drift_ticks * TSCH_DRIFT_UNIT / time_delta_ticks);

#if TSCH_DRIFT_SYNC_ESTIMATE
  if(drift_ticks != 0){
      if (drift_ticks < 0)
          drift_ticks = -drift_ticks;
      uint32_t ppt = ((time_delta_asn * TSCH_DRIFT_AVG_UNIT) / ((uint32_t)drift_ticks));
      drift1t_append(ppt, time_delta_asn);
  }
#endif
#ifdef TSCH_TIMESYNC_ON_DRIFT
  last_drift_ppm = TSCH_TIMESYNC_ON_DRIFT(last_drift_ppm, time_delta_asn, drift_ticks);
#endif

  drift_ppm = timesync_entry_add(last_drift_ppm, time_delta_ticks);

  TSCH_LOGF("drift %ld (/%lu asn)\n", drift_ppm / 256, time_delta_asn);
}
/*---------------------------------------------------------------------------*/
/* Either reset or update the neighbor's drift */
unsigned tsch_timesync_learn_timeout(){
#if (TSCH_DRIFT_SYNC_ESTIMATE & TSCH_DRIFT_SYNC_ESTIMATE_FASTER_INIT) != 0
    if (timesync_entry_count < 4 )
        return TSCH_SLOTS_PER_SECOND;
#endif
    return 4 * TSCH_SLOTS_PER_SECOND;
}

void
tsch_timesync_update(struct tsch_neighbor *n, uint16_t time_delta_asn, int32_t drift_correction)
{
  /* Account the drift if either this is a new timesource,
   * or the timedelta is not too small, as smaller timedelta
   * means proportionally larger measurement error. */
  if(last_timesource_neighbor != n) {
    tsch_adaptive_timesync_reset();
    last_timesource_neighbor = n;
  } else {
    asn_since_last_learning += time_delta_asn;
    if( asn_since_last_learning >= tsch_timesync_learn_timeout() ) {
      timesync_learn_drift_ticks(asn_since_last_learning, drift_correction);
      compensated_ticks = 0;
      asn_since_last_learning = 0;
    } else {
      /* Too small timedelta, do not recalculate the drift to avoid introducing error. instead account for the corrected ticks */
      compensated_ticks += drift_correction;
    }
  }
  min_drift_seen = MIN(drift_correction, min_drift_seen);
  max_drift_seen = MAX(drift_correction, max_drift_seen);
}
/*---------------------------------------------------------------------------*/
/* Error-accumulation free compensation algorithm */
static int32_t
compensate_internal(uint32_t time_delta_usec, int32_t drift_ppm, int32_t *remainder, int16_t *tick_conversion_error)
{
  int64_t d = (int64_t)time_delta_usec * drift_ppm + *remainder;
  int32_t amount = d / TSCH_DRIFT_UNIT;
  int32_t amount_ticks;

  *remainder = (int32_t)(d - amount * TSCH_DRIFT_UNIT);

  amount += *tick_conversion_error;
  amount_ticks = us_to_rtimerticks(amount);
  *tick_conversion_error = amount - rtimerticks_to_us(amount_ticks);

  if(ABS(amount_ticks) > RTIMER_ARCH_SECOND / 128) {
    TSCH_LOGF("!too big compensation %ld delta %ld", amount_ticks, time_delta_usec);
    amount_ticks = (amount_ticks > 0 ? RTIMER_ARCH_SECOND : -RTIMER_ARCH_SECOND) / 128;
  }

  return amount_ticks;
}
/*---------------------------------------------------------------------------*/
/* Do the compensation step before scheduling a new timeslot */
int32_t
tsch_timesync_adaptive_compensate(rtimer_clock_t time_delta_ticks)
{
  int32_t result = 0;
  uint32_t time_delta_usec = RTIMERTICKS_TO_US_64(time_delta_ticks);

  /* compensate, but not if the neighbor is not known */
  if(drift_ppm && last_timesource_neighbor != NULL) {
    static int32_t remainder;
    static int16_t tick_conversion_error;
    result = compensate_internal(time_delta_usec, drift_ppm,
        &remainder, &tick_conversion_error);
    compensated_ticks += result;
  }

  if(TSCH_BASE_DRIFT_PPM) {
    static int32_t base_drift_remainder;
    static int16_t base_drift_tick_conversion_error;
    result += compensate_internal(time_delta_usec, 256L * TSCH_BASE_DRIFT_PPM,
        &base_drift_remainder, &base_drift_tick_conversion_error);
  }

  return result;
}
/*---------------------------------------------------------------------------*/
void
tsch_adaptive_timesync_reset(void)
{
  last_timesource_neighbor = NULL;
  drift_ppm = 0;
  timesync_entry_count = 0;
  compensated_ticks = 0;
  asn_since_last_learning = 0;
}
/*---------------------------------------------------------------------------*/
#else /* TSCH_ADAPTIVE_TIMESYNC */
/*---------------------------------------------------------------------------*/
void
tsch_timesync_update(struct tsch_neighbor *n, uint16_t time_delta_asn, int32_t drift_correction)
{
}
/*---------------------------------------------------------------------------*/
int32_t
tsch_timesync_adaptive_compensate(rtimer_clock_t delta_ticks)
{
  return 0;
}
void
tsch_adaptive_timesync_reset(void)
{
}
/*---------------------------------------------------------------------------*/
long int
tsch_adaptive_timesync_get_drift_ppm(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
#endif /* TSCH_ADAPTIVE_TIMESYNC */
