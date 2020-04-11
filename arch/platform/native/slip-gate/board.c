/*
 * board.c
 *
 *  Created on: 9/04//2020.
 *      Author: alexrayne
 */

#include "platform-native.h"
#include "dev/serial-fd.h"
#include "services/rpl-border-router/native/slip-config.h"

void board_init(void){
    slip_config_handle_arguments(contiki_argc, contiki_argv);
    fdsio_assign_slip_config();
    fdsio_init();
}
