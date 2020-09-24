/*
 * trace.h
 * cp1251 ru
 *
 *  Created on: 21 дек. 2016 г.
 *      Author: alexrayne <alexraynepe196@gmail.com>
  ------------------------------------------------------------------------
    Copyright (c) alexrayne

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE. *
  -------------------------------------------------------------------------
 * Тут положу средства трассировки низкогоо уровня:
 *   - пробы железа
 */

#ifndef MAIN_INC_TRACE_H_

#include "dev/leds.h"
#include "dev/gpio-hal.h"

#if defined(LEDS_LEGACY_API) && defined (GPIO_HAL_PORT_PIN_NUMBERING)
#define MAIN_INC_TRACE_H_

#ifdef __cplusplus
extern "C" {
#endif



void trace_init(void);

//* перечислю доступные пины для пробов
//* пины назначаются на конкретные порты, устройства вывода, наблюдаемые снаружи
#define probe_none_id   0
#define probe_none_on(x)
#define probe_none_off(x)
#define probe_none_twist(x)
#define is_probe_none(x) 0

#define probe_pin1_id   1
#define probe_pin2_id   2
#define probe_pin3_id   3
#define probe_pin4_id   4
#define probe_pin5_id   5
#define probe_pin6_id   6
/*
void probe_pin1_on(void);
void probe_pin1_off(void);
void probe_pin2_on(void);
void probe_pin2_off(void);
*/

#ifndef INLINE
#define INLINE static inline
#endif

#define TRACE_PIN_RED       LEDS_RED
#define TRACE_PIN_GREEN     LEDS_GREEN
#define TRACE_PIN_BLUE      LEDS_BLUE
#define TRACE_PIN_YELLOW    LEDS_YELLOW
#define TRACE_PIN_ORANGE    LEDS_ORANGE

#define probeled(N, PIN) \
INLINE uint32_t is_probe_##N(void)  { return 0; } \
INLINE void probe_##N##_on(void)    { leds_on(PIN); } \
INLINE void probe_##N##_off(void)   { leds_off(PIN); } \
INLINE void probe_##N##_twist(void) { leds_toggle(PIN); }

probeled(red,   TRACE_PIN_RED)
probeled(green, TRACE_PIN_GREEN)
probeled(blue,  TRACE_PIN_BLUE)
probeled(yellow,TRACE_PIN_YELLOW)
probeled(orange,TRACE_PIN_ORANGE)



#ifdef COOJA
#define TRACE_PIN_DIO21 21
#define TRACE_PIN_DIO22 22
#define TRACE_PIN_DIO23 23
#define TRACE_PIN_DIO24 24
#define TRACE_PIN_DIO25 25
#define TRACE_PIN_DIO26 26
#define TRACE_PIN_DIO27 27
#define TRACE_PIN_DIO28 28
#else
#include "board.h"
#define TRACE_PIN_DIO21 BOARD_IOID_DIO21
#define TRACE_PIN_DIO22 BOARD_IOID_DIO22
#define TRACE_PIN_DIO23 BOARD_IOID_DIO23
#define TRACE_PIN_DIO24 BOARD_IOID_DIO24
#define TRACE_PIN_DIO25 BOARD_IOID_DIO25
#define TRACE_PIN_DIO26 BOARD_IOID_DIO26
#define TRACE_PIN_DIO27 BOARD_IOID_DIO27
#define TRACE_PIN_DIO28 BOARD_IOID_DIO28
#endif

#define TRACE_PINS ( 1ul<< TRACE_PIN_DIO21 \
                    |1ul<< TRACE_PIN_DIO22 \
                    |1ul<< TRACE_PIN_DIO23 \
                    |1ul<< TRACE_PIN_DIO24 \
                    |1ul<< TRACE_PIN_DIO25 \
                    |1ul<< TRACE_PIN_DIO26 \
                    |1ul<< TRACE_PIN_DIO27 \
                    |1ul<< TRACE_PIN_DIO28 \
                    )

#define probepin(N, PIN) \
INLINE uint32_t is_probe_##N(void)  { return 0; } \
INLINE void probe_##N##_on(void)    { gpio_hal_arch_set_pin(0, PIN); } \
INLINE void probe_##N##_off(void)   { gpio_hal_arch_clear_pin(0, PIN); } \
INLINE void probe_##N##_twist(void) { gpio_hal_arch_toggle_pin(0, PIN); }


#define probe_dio21_id   1
#define probe_dio22_id   2
#define probe_dio23_id   3
#define probe_dio24_id   4
#define probe_dio25_id   5
#define probe_dio26_id   6
#define probe_dio27_id   7
#define probe_dio28_id   8

probepin(dio21, TRACE_PIN_DIO21)
probepin(dio22, TRACE_PIN_DIO22)
probepin(dio23, TRACE_PIN_DIO23)
probepin(dio24, TRACE_PIN_DIO24)
probepin(dio25, TRACE_PIN_DIO25)
probepin(dio26, TRACE_PIN_DIO26)
probepin(dio27, TRACE_PIN_DIO27)
probepin(dio28, TRACE_PIN_DIO28)

probepin(pin1, TRACE_PIN_DIO21)
probepin(pin2, TRACE_PIN_DIO22)
probepin(pin3, TRACE_PIN_DIO23)
probepin(pin4, TRACE_PIN_DIO24)
probepin(pin5, TRACE_PIN_DIO25)
probepin(pin6, TRACE_PIN_DIO26)
probepin(pin7, TRACE_PIN_DIO27)
probepin(pin8, TRACE_PIN_DIO28)



//* проб можно дернуть этими макро с указанием имени порба
#define trace_probe_on(name) probe_pin_on(trace_##name)
#define trace_probe_off(name) probe_pin_off(trace_##name)

#define TRC_ASSTR(name) name
#define TRC_ASSTR2(name) TRC_ASSTR(name)
#define TRC_CATSTR(name,suf) name##suf
#define TRC_CATSTR2(name,suf) TRC_CATSTR(name,suf)

#define PROBESTR(name, suf) TRC_CATSTR(name, suf)
//* это макро создает пару манипулятора пробов вида trace_<name>_on/off()
#define trace_probe(name) \
enum{ trace_##name##_id = TRC_CATSTR2( trace_##name, _id) };\
INLINE void trace_##name##_on(void)\
{PROBESTR(TRC_ASSTR(trace_##name), _on)();};\
INLINE void trace_##name##_off(void)\
{PROBESTR(TRC_ASSTR(trace_##name),_off)();};\
INLINE void trace_##name##_twist(void)\
{PROBESTR(TRC_ASSTR(trace_##name),_twist)();};\
INLINE uint32_t is_trace_##name(void)\
{return PROBESTR(is_,TRC_ASSTR(trace_##name))();};

#define TRACEID(name) TRC_CATSTR2(name, _id)

#define trace_need(name) \
        INLINE uint32_t is_trace_##name(void) {return false;}\
        INLINE void trace_##name##_on(void) {;}\
        INLINE void trace_##name##_off(void) {;}\
        INLINE void trace_##name##_twist(void) {;}


#include <trace-port.h>


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#define TRACE_GUARD(name, tr) struct TRC_ASSTR(trace_guard_##name) { \
        TRC_ASSTR(trace_guard_##name)(){ trace_##tr##_on();};\
        TRC_ASSTR(~trace_guard_##name)(){ trace_##tr##_off();};\
    } trace_##name

#endif

#endif //#if defined(LEDS_LEGACY_API) && defined (GPIO_HAL_PORT_PIN_NUMBERING)
#endif /* MAIN_INC_TRACE_H_ */
