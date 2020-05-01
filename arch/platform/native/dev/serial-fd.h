/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
 *               2020 alexrayne <alexraynepe196@gmail.com>
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
 * Header file for SerialIO via File driver.
 *         forked from service/board-router/native/slip-dev
 * \author alexrayne <alexraynepe196@gmail.com>
 * \author Adam Dunkels <adam@sics.se>
 *
 */
#ifndef FDSIO_H_
#define FDSIO_H_

#include <stdint.h>
#include <termios.h>
#include <unistd.h>

#define RS232_19200 1
#define RS232_38400 2
#define RS232_57600 3
#define RS232_115200 4


/**
 *  @brief      accept slip setup parsed by slip_config_handle_arguments(..)
 *  @sa <os/services/rpl-border-router/native/slip-config.h>
 */
void fdsio_assign_slip_config();

/**
 * \brief      Initialize the serial-on-file  module
 *
 */
void fdsio_init(void);

/**
 * \brief      Flow control style setup modes.
 *
 *  flow control with host specification leads to establish server on localhost
 *  type of server may be TCP(default ) or UDP
 */
enum FDSIO_FlowControl{
    FDSIO_NOFLOW = 0,
    FDSIO_SERVE_TCP = 1,
    FDSIO_SERVE_UDP = 2,
};

/**
 * \brief      Set an input handler for incoming fdsio data
 * \param f    A pointer to a byte input handler
 *
 *             This function sets the input handler for incoming fdsio
 *             data. The input handler function is called for every
 *             incoming data byte. The function is called from the
 *             fdsio select_set_callback.
 */
void fdsio_set_input(int (* f)(unsigned char));

/**
 * \brief      Configure the speed of the fdsio hardware
 * \param speed The speed
 *
 *             This function configures the speed of the TTY
 *             hardware. The allowed parameters are RS232_19200,
 *             RS232_38400, RS232_57600, and RS232_115200.
 */
void fdsio_set_speed(unsigned char speed);

/**
 * \brief      Print a text string on fdsio serial
 * \param text A pointer to the string that is to be printed
 *
 *             The string must be terminated by a null byte.
 */
void fdsio_puts(char *text);

/**
 * \brief      Print a character on fdsio serial
 * \param c    The character to be printed
 */
void fdsio_send(unsigned char c);


/**
 * \brief nonblocking put bytes to serial
 * \param data The characters to transmit
 * \param len  amount of characters to transmit
 * \return number of writen characters
 */
int fdsio_write_bytes(const void* data, unsigned len);



#ifdef FDSIO_CONF_BUF_SIZE
#define FDSIO_BUF_SIZE FDSIO_CONF_BUF_SIZE
#else
#define FDSIO_BUF_SIZE 2048
#endif

/**
 * \brief Returns the Serial status
 * \return <0 - serial not available, turned off
 * \return 0  - serial output is empty
 * \return >0 - serial have output for send, amount of sent data
*/
int fdsio_busy(void);

/**
 * \brief Returns the UART full status
 * \return <0 - UART not available, turned off
 * \return 0  - UART if full, and blocks for write
 * \return >0 - UART can write bytes without block
 *
 * ti_lib_uart_busy() will access UART registers. It is our responsibility
 * to first make sure the UART is accessible before calling it. Hence this
 * wrapper.
 *
 * @Return space free avail in outgoing buffer
 */
int fdsio_space_avail(void);

#endif /* FDSIO_H_ */

