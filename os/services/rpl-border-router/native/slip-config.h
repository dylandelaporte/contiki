/*
 *      Author: alexrayne <apexraynepe196@gmail.com>
 */

#ifndef OS_SERVICES_RPL_BORDER_ROUTER_NATIVE_SLIP_CONFIG_H_
#define OS_SERVICES_RPL_BORDER_ROUTER_NATIVE_SLIP_CONFIG_H_


#include <stdint.h>
#include <termios.h>
#include <unistd.h>


struct SlipConfig {
    int         verbose;
    int         flowcontrol;
    int         timestamp;
    const char *ipaddr;
    const char *siodev;
    const char *host;
    const char *port;
    char        tundev[32];
    uint16_t    basedelay;
    speed_t     b_rate;
};

extern struct SlipConfig slip_config;

// return slip_config - fill in this setup by command line
int slip_config_handle_arguments(int argc, char **argv);


#endif /* OS_SERVICES_RPL_BORDER_ROUTER_NATIVE_SLIP_CONFIG_H_ */
