/*
 * Copyright (c) 2016, Texas Instruments Incorporated - http://www.ti.com/
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
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup rf-core
 * @{
 *
 * \defgroup rf-core-prop CC13xx Prop mode driver
 *
 * @{
 *
 * \file
 * Header file for the CC13xx prop mode NETSTACK_RADIO driver
 */
/*---------------------------------------------------------------------------*/
#ifndef PROP_MODE_H_
#define PROP_MODE_H_
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "rf-core/dot-15-4g.h"

#include <stdint.h>
#include "rf-core/tx-power.h"
/*---------------------------------------------------------------------------*/
typedef tx_power_table_t prop_mode_tx_power_config_t;

#include <dev/radio.h>
enum rf_power_style {
    RADIO_POWER_STYLE_FREE
    , RADIO_POWER_STYLE_GLDO
    , RADIO_POWER_STYLE_TOTAL
};
typedef enum rf_power_style rf_power_style;
enum {
      RADIO_CC26_PARAMS = RADIO_PARAM_TOTAL
    , RADIO_CC26_POWER_STYLE
};

/*---------------------------------------------------------------------------*/
#endif /* PROP_MODE_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
