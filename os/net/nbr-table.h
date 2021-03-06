/*
 * Copyright (c) 2013, Swedish Institute of Computer Science
 * Copyright (c) 2010, Vrije Universiteit Brussel
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
 *
 * Authors: Simon Duquennoy <simonduq@sics.se>
 *          Joris Borms <joris.borms@vub.ac.be>
 */

#ifndef NBR_TABLE_H_
#define NBR_TABLE_H_

#include "contiki.h"
#include "net/linkaddr.h"
#include "net/netstack.h"

/* Neighbor table size */
#ifdef NBR_TABLE_CONF_MAX_NEIGHBORS
#define NBR_TABLE_MAX_NEIGHBORS NBR_TABLE_CONF_MAX_NEIGHBORS
#else /* NBR_TABLE_CONF_MAX_NEIGHBORS */
#define NBR_TABLE_MAX_NEIGHBORS 8
#endif /* NBR_TABLE_CONF_MAX_NEIGHBORS */

/* An item in a neighbor table */
typedef void nbr_table_item_t;

/* Callback function, called when removing an item from a table */
typedef void(nbr_table_callback)(nbr_table_item_t *item);

/* A neighbor table */
typedef struct nbr_table {
  int index;
  int item_size;
  nbr_table_callback *callback;
  nbr_table_item_t *data;
} nbr_table_t;

/* index in nbr_tables */
typedef int nbr_idx_t;

/*this inlines give faster nbr tables index access */
#define NBR_TABLE_INLINES(type, name) \
    static inline type* name##_item_from_index(nbr_idx_t index) {\
        if (index >= 0) \
            return (type *)name->data + index; \
        return NULL; } \
    static inline nbr_idx_t name##_index_from_item(const type* item) {\
        if (item != NULL)\
            return (item - (const type *)(name->data));\
        return -1;} \
    static inline linkaddr_t* name##_lladr_item(const type* item) {\
        if (item != NULL)\
            return nbr_table_idx_lladdr( item - (const type *)(name->data) );\
        return NULL;} \
    static inline int name##_remove_item(nbr_table_item_t* item) {\
        if (item != NULL)\
            return nbr_table_idx_remove(name, (const type*)item - (const type *)(name->data) );\
        return -1;}

/** \brief A static neighbor table. To be initialized through nbr_table_register(name) */
#define NBR_TABLE(type, name) \
  static type _##name##_mem[NBR_TABLE_MAX_NEIGHBORS]; \
  static nbr_table_t name##_struct = { 0, sizeof(type), NULL, (nbr_table_item_t *)_##name##_mem }; \
  static nbr_table_t *name = &name##_struct; \
  NBR_TABLE_INLINES(type, name)

/** \brief A non-static neighbor table. To be initialized through nbr_table_register(name) */
#define NBR_TABLE_GLOBAL(type, name) \
  static type _##name##_mem[NBR_TABLE_MAX_NEIGHBORS]; \
  static nbr_table_t name##_struct = { 0, sizeof(type), NULL, (nbr_table_item_t *)_##name##_mem }; \
  nbr_table_t *name = &name##_struct;

/** \brief Declaration of non-static neighbor tables */
#define NBR_TABLE_DECLARE(type, name) extern nbr_table_t *name;\
                    NBR_TABLE_INLINES(type, name)



typedef enum {
        NBR_TABLE_REASON_UNDEFINED,
	NBR_TABLE_REASON_RPL_DIO,
	NBR_TABLE_REASON_RPL_DAO,
	NBR_TABLE_REASON_RPL_DIS,
	NBR_TABLE_REASON_ROUTE,
	NBR_TABLE_REASON_IPV6_ND,
  NBR_TABLE_REASON_IPV6_ND_AUTOFILL,
	NBR_TABLE_REASON_MAC,
	NBR_TABLE_REASON_LLSEC,
	NBR_TABLE_REASON_LINK_STATS,
  NBR_TABLE_REASON_SIXTOP,
} nbr_table_reason_t;

/** \name Neighbor tables: register and loop through table elements */
/** @{ */
int nbr_table_register(nbr_table_t *table, nbr_table_callback *callback);
int nbr_table_is_registered(nbr_table_t *table);
nbr_table_item_t *nbr_table_head(nbr_table_t *table);
nbr_table_item_t *nbr_table_next(nbr_table_t *table, nbr_table_item_t *item);
/** @} */

/** \name Neighbor tables: add and get data */
/** @{ */
nbr_table_item_t *nbr_table_add_lladdr(nbr_table_t *table, const linkaddr_t *lladdr, nbr_table_reason_t reason, void *data);
nbr_table_item_t *nbr_table_get_from_lladdr(nbr_table_t *table, const linkaddr_t *lladdr);
nbr_table_item_t *nbr_table_get_from_idx(nbr_table_t *table, nbr_idx_t idx);
/** @} */

/** \name Neighbor tables: set flags (unused, locked, unlocked) */
/** @{ */
int nbr_table_remove(nbr_table_t *table, nbr_table_item_t *item);
int nbr_table_idx_remove(nbr_table_t *table, nbr_idx_t idx);
int nbr_table_lock(nbr_table_t *table, nbr_table_item_t *item);
int nbr_table_unlock(nbr_table_t *table, nbr_table_item_t *item);
/** @} */

/** \name Neighbor tables: address manipulation */
/** @{ */
linkaddr_t *nbr_table_get_lladdr(nbr_table_t *table, const nbr_table_item_t *item);
/** @name take lladdr by nbr index.
      NBR_TABLE_xxx provide index acces for declared table by routines
      name##_index_from_item(const type* item) - use it for fast index on table item
  */
linkaddr_t *nbr_table_idx_lladdr(nbr_idx_t idx);
/** @} */

#endif /* NBR_TABLE_H_ */
