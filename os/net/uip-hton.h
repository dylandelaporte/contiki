/*
 * Copyright (c) 2001-2003, Adam Dunkels.
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
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the uIP TCP/IP stack.
 *
 *
 */

#ifndef OS_NET_UIP_HTON_H_
#define OS_NET_UIP_HTON_H_

/**
 * \addtogroup uip
 * @{
 */

/**
 * \file
 * Header file for the uIP TCP/IP stack.
 * \author  Adam Dunkels <adam@dunkels.com>
 * \author  Julien Abeille <jabeille@cisco.com> (IPv6 related code)
 * \author  Mathilde Durvy <mdurvy@cisco.com> (IPv6 related code)
 *
 */



#include <stdint.h>

/**
 * Convert 16-bit quantity from host byte order to network byte order.
 *
 * This macro is primarily used for converting constants from host
 * byte order to network byte order. For converting variables to
 * network byte order, use the uip_htons() function instead.
 *
 * \hideinitializer
 */
#ifndef UIP_HTONS
#   if UIP_BYTE_ORDER == UIP_BIG_ENDIAN
#      define UIP_HTONS(n) (n)
#      define UIP_HTONL(n) (n)

#   ifndef uip_htons
#       define uip_htons(n)UIP_HTONS(n)
#   endif
#   ifndef uip_htonl
#       define uip_htonl(n)UIP_HTONL(n)
#   endif

#   else /* UIP_BYTE_ORDER == UIP_BIG_ENDIAN */
#ifdef __GNUC__
#       define UIP_HTONS(n) __builtin_bswap16(n)
#       define UIP_HTONL(n) __builtin_bswap32(n)

#   ifndef uip_htons
#       define uip_htons(n)UIP_HTONS(n)
#   endif
#   ifndef uip_htonl
#       define uip_htonl(n)UIP_HTONL(n)
#   endif

#elif defined(__REV)
    // ARM-core provide this command. CMSIS provide this macro
#       define UIP_HTONS(n) __REV16(n)
#       define UIP_HTONL(n) __REV(n)

#   ifndef uip_htons
#       define uip_htons(n)UIP_HTONS(n)
#   endif
#   ifndef uip_htonl
#       define uip_htonl(n)UIP_HTONL(n)
#   endif

#else
#     define UIP_HTONS(n) (uint16_t)((((uint16_t) (n)) << 8) | (((uint16_t) (n)) >> 8))
#     define UIP_HTONL(n) (((uint32_t)UIP_HTONS(n) << 16) | UIP_HTONS((uint32_t)(n) >> 16))
#endif
#   endif /* UIP_BYTE_ORDER == UIP_BIG_ENDIAN */
#else
#error "UIP_HTONS already defined!"
#endif /* UIP_HTONS */

/**
 * Convert a 16-bit quantity from host byte order to network byte order.
 *
 * This function is primarily used for converting variables from host
 * byte order to network byte order. For converting constants to
 * network byte order, use the UIP_HTONS() macro instead.
 */
#ifndef uip_htons
uint16_t uip_htons(uint16_t val);
#endif /* uip_htons */
#ifndef uip_ntohs
#define uip_ntohs uip_htons
#endif

#ifndef uip_htonl
uint32_t uip_htonl(uint32_t val);
#endif /* uip_htonl */
#ifndef uip_ntohl
#define uip_ntohl uip_htonl
#endif

/** @} */

#endif /* OS_NET_UIP_HTON_H_ */
