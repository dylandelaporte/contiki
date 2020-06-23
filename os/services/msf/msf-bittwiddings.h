/*
 * msf-bittwiddings.h
 *
 *  Created on: 18/06//2020
 *      Author: alexrayne <alexraynepe196@gmail.com>
 */

#ifndef OS_SERVICES_MSF_MSF_BITTWIDDINGS_H_
#define OS_SERVICES_MSF_MSF_BITTWIDDINGS_H_



#if defined( __GNUC__ )

static inline
int ffz( unsigned int x ){  return __builtin_ctz(~x); }

#elif  _XOPEN_SOURCE >= 700 \
        || ! (_POSIX_C_SOURCE >= 200809L) \
        || /* Glibc since 2.19: */ _DEFAULT_SOURCE \
        || /* Glibc versions <= 2.19: */ _BSD_SOURCE \
        || _SVID_SOURCE

#include <strings.h>

static inline
int ffz( unsigned int x ){  return ffs(~x); }

#elif defined(__CTZ)

static inline
int ffz( unsigned int x ){  return __CTZ(~x); }

#endif



#endif /* OS_SERVICES_MSF_MSF_BITTWIDDINGS_H_ */
