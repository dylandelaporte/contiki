/*
 * uip_hton.h
 *
 *  Created on: 26 θών. 2020 γ.
 *      Author: netuser
 */

#ifndef OS_NET_IP_UIP_HTON_H_
#define OS_NET_IP_UIP_HTON_H_


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


#endif /* OS_NET_IP_UIP_HTON_H_ */
