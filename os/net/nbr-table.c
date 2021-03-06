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

#include "contiki.h"

#include <stddef.h>
#include <string.h>
#include "lib/memb.h"
#include "lib/list.h"
#include "net/nbr-table.h"



/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "NetNbr"
#ifdef  LOG_CONF_LEVEL_NET
#define LOG_LEVEL LOG_CONF_LEVEL_NET
#else
#define LOG_LEVEL LOG_LEVEL_NONE
#endif

#if LOG_LEVEL >= LOG_LEVEL_DBG
#define DEBUG 1
#else
#define DEBUG 0
#endif

#if DEBUG
#include <stdio.h>
#include "sys/ctimer.h"
static void handle_periodic_timer(void *ptr);
static struct ctimer periodic_timer;
static uint8_t initialized = 0;
static void print_table();
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

// Provide Nbr index ops check that index in table bounds
#ifndef NBR_CHECK_BOUNDS
#define NBR_CHECK_BOUNDS DEBUG
#endif


/* This is the callback function that will be called when there is a
 *  nbr-policy active
 **/
#ifdef NBR_TABLE_FIND_REMOVABLE
const linkaddr_t *NBR_TABLE_FIND_REMOVABLE(nbr_table_reason_t reason, void *data);
#endif /* NBR_TABLE_FIND_REMOVABLE */


/* List of link-layer addresses of the neighbors, used as key in the tables */
typedef struct nbr_table_key {
  struct nbr_table_key *next;
  linkaddr_t lladdr;
} nbr_table_key_t;

/* For each neighbor, a map of the tables that use the neighbor.
 * As we are using uint8_t, we have a maximum of 8 tables in the system */
static uint8_t used_map[NBR_TABLE_MAX_NEIGHBORS];
/* For each neighbor, a map of the tables that lock the neighbor */
static uint8_t locked_map[NBR_TABLE_MAX_NEIGHBORS];
/* The maximum number of tables */
#define MAX_NUM_TABLES 8
/* A list of pointers to tables in use */
static struct nbr_table *all_tables[MAX_NUM_TABLES];
/* The current number of tables */
static unsigned num_tables;

/* The neighbor address table */
MEMB(neighbor_addr_mem, nbr_table_key_t, NBR_TABLE_MAX_NEIGHBORS);
LIST(nbr_table_keys);



#if NBR_CHECK_BOUNDS
#include <assert.h>
#define ASSERT_NBR( x ) assert( (x) )
#define IN_MEMB(m, index) (index < m.num)
#define IN_MAPS(m, index) (index < NBR_TABLE_MAX_NEIGHBORS)
#define IN_BITMAP(m, index) (index < (8*sizeof(used_map[0])))
#else
#define ASSERT_NBR( x )
#define IN_MEMB(m, index) 1
#define IN_MAPS(m, index) 1
#define IN_BITMAP(m, index) 1
#endif

/*---------------------------------------------------------------------------*/
/* Get a key from a neighbor index */
static nbr_table_key_t *
key_from_index(nbr_idx_t index)
{
  ASSERT_NBR(IN_MEMB(neighbor_addr_mem, index));
  if ((index >= 0) && IN_MEMB(neighbor_addr_mem, index))
      return &((nbr_table_key_t *)neighbor_addr_mem.mem)[index];
  return NULL;
}
/*---------------------------------------------------------------------------*/
/* Get an item from its neighbor index */
static nbr_table_item_t *
item_from_index(nbr_table_t *table, nbr_idx_t index)
{
  if ((table != NULL) && (index >= 0))
      return (char *)table->data + index * table->item_size;
  return NULL;
}
/*---------------------------------------------------------------------------*/
/* Get the neighbor index of an item */
static nbr_idx_t
index_from_key(nbr_table_key_t *key)
{
  if (key != NULL){
      int idx = key - (nbr_table_key_t *)neighbor_addr_mem.mem;
      ASSERT_NBR( IN_MEMB(neighbor_addr_mem, idx ));
      if (IN_MEMB(neighbor_addr_mem, idx))
          return idx;
  }
  return  -1;
}
/*---------------------------------------------------------------------------*/
/* Get the neighbor index of an item */
static nbr_idx_t
index_from_item(nbr_table_t *table, const nbr_table_item_t *item)
{
  if (table != NULL && item != NULL)
      return ((int)((char *)item - (char *)table->data)) / table->item_size;
  else
      return -1;
}
/*---------------------------------------------------------------------------*/
/* Get an item from its key */
static nbr_table_item_t *
item_from_key(nbr_table_t *table, nbr_table_key_t *key)
{
  return item_from_index(table, index_from_key(key));
}
/*---------------------------------------------------------------------------*/
/* Get the key af an item */
static nbr_table_key_t *
key_from_item(nbr_table_t *table, const nbr_table_item_t *item)
{
  return key_from_index(index_from_item(table, item));
}
/*---------------------------------------------------------------------------*/
/* Get the index of a neighbor from its link-layer address */
static nbr_idx_t
index_from_lladdr(const linkaddr_t *lladdr)
{
  nbr_table_key_t *key;
  /* Allow lladdr-free insertion, useful e.g. for IPv6 ND.
   * Only one such entry is possible at a time, indexed by linkaddr_null. */
  if(lladdr == NULL) {
    lladdr = &linkaddr_null;
  }
  key = list_head(nbr_table_keys);
  while(key != NULL) {
    if(lladdr && linkaddr_cmp(lladdr, &key->lladdr)) {
      return index_from_key(key);
    }
    key = list_item_next(key);
  }
  return -1;
}
/*---------------------------------------------------------------------------*/
/* Get bit from "used" or "locked" bitmap */
static int
nbr_get_bit_idx(uint8_t *bitmap, nbr_table_t *table, nbr_idx_t item_index)
{
  if(table != NULL && item_index >= 0) {
    ASSERT_NBR( IN_MAPS(bitmap, item_index) );
    ASSERT_NBR( IN_BITMAP(bitmap, table->index) );
    return (bitmap[item_index] & (1 << table->index)) != 0;
  } else {
    return 0;
  }
  return 0;
}

static int
nbr_get_bit(uint8_t *bitmap, nbr_table_t *table, nbr_table_item_t *item)
{
  int item_index = index_from_item(table, item);
  return nbr_get_bit_idx(bitmap, table, item_index);
}
/*---------------------------------------------------------------------------*/
/* Set bit in "used" or "locked" bitmap */
static int
nbr_set_bit_idx(uint8_t *bitmap, nbr_table_t *table, nbr_idx_t item_index, int value)
{
  if(table != NULL && item_index >= 0) {
    ASSERT_NBR( IN_MAPS(bitmap, item_index) );
    ASSERT_NBR( IN_BITMAP(bitmap, table->index) );
    if(value) {
      bitmap[item_index] |= 1 << table->index;
    } else {
      bitmap[item_index] &= ~(1 << table->index);
    }
    return 1;
  } else {
    return 0;
  }
  return 0;
}

static int
nbr_set_bit(uint8_t *bitmap, nbr_table_t *table, nbr_table_item_t *item, int value)
{
  int item_index = index_from_item(table, item);
  return nbr_set_bit_idx(bitmap, table, item_index, value);
}
/*---------------------------------------------------------------------------*/
static void
remove_key(nbr_table_key_t *least_used_key)
{
  int i;
  for(i = 0; i < MAX_NUM_TABLES; i++) {
    if(all_tables[i] != NULL && all_tables[i]->callback != NULL) {
      /* Call table callback for each table that uses this item */
      nbr_table_item_t *removed_item = item_from_key(all_tables[i], least_used_key);
      if(nbr_get_bit(used_map, all_tables[i], removed_item) == 1) {
        all_tables[i]->callback(removed_item);
      }
    }
  }
  /* Empty used map */
  used_map[index_from_key(least_used_key)] = 0;
  /* Remove neighbor from list */
  list_remove(nbr_table_keys, least_used_key);
}
/*---------------------------------------------------------------------------*/
#if defined( __GNUC__ ) && !defined(__MSP430__)
/* FIX: GCC for MSP430 have missed __builtin_popcount */
static inline
int popcount8( unsigned x ){  return __builtin_popcount(x); }
static inline
int popcount( unsigned x ){  return __builtin_popcount(x); }
#else
static
int popcount8(uint8_t n) {
    n = (n & 0x55u) + ((n >> 1) & 0x55u);
    n = (n & 0x33u) + ((n >> 2) & 0x33u);
    n = (n & 0x0fu) + ((n >> 4) & 0x0fu);
    return n;
}

static
int popcount(uint32_t n) {
    n = (n & 0x55555555u) + ((n >> 1) & 0x55555555u);
    n = (n & 0x33333333u) + ((n >> 2) & 0x33333333u);
    n = (n & 0x0f0f0f0fu) + ((n >> 4) & 0x0f0f0f0fu);
    n = (n & 0x00ff00ffu) + ((n >> 8) & 0x00ff00ffu);
    n = (n & 0x0000ffffu) + ((n >>16) & 0x0000ffffu);
    return n;
}
#endif

static nbr_table_key_t *
nbr_table_allocate(nbr_table_reason_t reason, void *data)
{
  nbr_table_key_t *key;
  int least_used_count = 0;
  nbr_table_key_t *least_used_key = NULL;

  key = memb_alloc(&neighbor_addr_mem);
  if(key != NULL) {
    return key;
  } else {
#ifdef NBR_TABLE_FIND_REMOVABLE
    const linkaddr_t *lladdr;
    lladdr = NBR_TABLE_FIND_REMOVABLE(reason, data);
    if(lladdr == NULL) {
      /* Nothing found that can be deleted - return NULL to indicate failure */
      LOG_INFO("*** Not removing entry to allocate new\n");
      return NULL;
    } else {
      /* used least_used_key to indicate what is the least useful entry */
      nbr_idx_t index;
      int locked = 0;
      if((index = index_from_lladdr(lladdr)) >= 0) {
        least_used_key = key_from_index(index);
        ASSERT_NBR( (index >= 0) && IN_MAPS(locked_map, index) );
        locked = locked_map[index];
      }
      /* Allow delete of locked item? */
      if(least_used_key != NULL && locked) {
        LOG_INFO("Deleting locked item!\n");
        ASSERT_NBR( (index >= 0) && IN_MAPS(locked_map, index) );
        locked_map[index] = 0;
      }
    }
#endif /* NBR_TABLE_FIND_REMOVABLE */

    if(least_used_key == NULL) {
      /* No more space, try to free a neighbor.
       * The replacement policy is the following: remove neighbor that is:
       * (1) not locked
       * (2) used by fewest tables
       * (3) oldest (the list is ordered by insertion time)
       * */
      /* Get item from first key */
      key = list_head(nbr_table_keys);
      while(key != NULL) {
        nbr_idx_t item_index = index_from_key(key);
        ASSERT_NBR( (item_index >= 0) && IN_MAPS(locked_map, item_index) );
        int locked = locked_map[item_index];
        /* Never delete a locked item */
        if(!locked) {
          int used = used_map[item_index];
          int used_count;
          if (sizeof(used_map[0]) == 1)
              used_count = popcount8(used);
          else
              used_count = popcount(used);

          /* Find least used item */
          if(least_used_key == NULL || used_count < least_used_count) {
            least_used_key = key;
            least_used_count = used_count;
            if(used_count == 0) { /* We won't find any least used item */
              break;
            }
          }
        }
        key = list_item_next(key);
      }
    }

    if(least_used_key == NULL) {
      /* We haven't found any unlocked item, allocation fails */
      return NULL;
    } else {
      /* Reuse least used item */
      remove_key(least_used_key);
      return least_used_key;
    }
  }
}
/*---------------------------------------------------------------------------*/
/* Register a new neighbor table. To be used at initialization by modules
 * using a neighbor table */
int
nbr_table_register(nbr_table_t *table, nbr_table_callback *callback)
{
#if DEBUG
  if(!initialized) {
    initialized = 1;
    /* schedule a debug printout per minute */
    ctimer_set(&periodic_timer, CLOCK_SECOND * 60, handle_periodic_timer, NULL);
  }
#endif

  if(nbr_table_is_registered(table)) {
    /* Table already registered, just update callback */
    table->callback = callback;
    return 1;
  }

  if(num_tables < MAX_NUM_TABLES) {
    table->index = num_tables++;
    table->callback = callback;
    all_tables[table->index] = table;
    LOG_DBG("register table %p ->[%u]\n", table, num_tables);
    return 1;
  } else {
    /* Maximum number of tables exceeded */
    return 0;
  }
}
/*---------------------------------------------------------------------------*/
/* Test whether a specified table has been registered or not */
int
nbr_table_is_registered(nbr_table_t *table)
{
  if(table != NULL && table->index >= 0 && table->index < MAX_NUM_TABLES)
  if(all_tables[table->index] == table) {
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Returns the first item of the current table */
nbr_table_item_t *
nbr_table_head(nbr_table_t *table)
{
  /* Get item from first key */
  nbr_idx_t idx = index_from_key(list_head(nbr_table_keys));
  nbr_table_item_t *item = item_from_index(table, idx);
  if (item == NULL)
      return NULL;
  /* Item is the first neighbor, now check is it is in the current table */
  if(nbr_get_bit_idx(used_map, table, idx)) {
    return item;
  } else {
    return nbr_table_next(table, item);
  }
}
/*---------------------------------------------------------------------------*/
/* Iterates over the current table */
nbr_table_item_t *
nbr_table_next(nbr_table_t *table, nbr_table_item_t *item)
{
  nbr_idx_t idx;
  do {
    void *key = key_from_item(table, item);
    key = list_item_next(key);
    /* Loop until the next item is in the current table */
    idx = index_from_key(key);
    item = item_from_index(table, idx);
  } while(item && !nbr_get_bit_idx(used_map, table, idx));
  return item;
}
/*---------------------------------------------------------------------------*/
/* Add a neighbor indexed with its link-layer address */
nbr_table_item_t *
nbr_table_add_lladdr(nbr_table_t *table, const linkaddr_t *lladdr, nbr_table_reason_t reason, void *data)
{
  nbr_idx_t index;
  nbr_table_item_t *item;
  nbr_table_key_t *key;

  if(table == NULL) {
    return NULL;
  }

  /* Allow lladdr-free insertion, useful e.g. for IPv6 ND.
   * Only one such entry is possible at a time, indexed by linkaddr_null. */
  if(lladdr == NULL) {
    lladdr = &linkaddr_null;
  }

  if((index = index_from_lladdr(lladdr)) == -1) {
     /* Neighbor not yet in table, let's try to allocate one */
    key = nbr_table_allocate(reason, data);

    /* No space available for new entry */
    if(key == NULL) {
      LOG_ERR("Not enough mem to alloc ");
      LOG_ERR_LLADDR(lladdr);
      LOG_ERR_("\n");
      return NULL;
    }

    /* Add neighbor to list */
    list_add(nbr_table_keys, key);

    /* Get index from newly allocated neighbor */
    index = index_from_key(key);

    /* Set link-layer address */
    linkaddr_copy(&key->lladdr, lladdr);
  }

  LOG_DBG("set nbr ");
  LOG_DBG_LLADDR(lladdr);
  LOG_DBG_(" ->[%d]\n", index);

  /* Get item in the current table */
  item = item_from_index(table, index);

  /* Initialize item data and set "used" bit */
  memset(item, 0, table->item_size);
  nbr_set_bit_idx(used_map, table, index, 1);

#if DEBUG
  print_table();
#endif
  return item;
}
/*---------------------------------------------------------------------------*/
/* Get an item from its link-layer address */
void *
nbr_table_get_from_lladdr(nbr_table_t *table, const linkaddr_t *lladdr)
{
  nbr_idx_t idx = index_from_lladdr(lladdr);
  return nbr_table_get_from_idx(table, idx);
}

void* nbr_table_get_from_idx(nbr_table_t *table, nbr_idx_t idx){
    if (idx < 0)
        return NULL;
  if (nbr_get_bit_idx(used_map, table, idx))
      return item_from_index(table, idx);
  return NULL;
}
/*---------------------------------------------------------------------------*/
/* Removes a neighbor from the current table (unset "used" bit) */
int nbr_table_idx_remove(nbr_table_t *table, nbr_idx_t item_index)
{
  int ret = nbr_set_bit_idx(used_map, table, item_index, 0);
  nbr_set_bit_idx(locked_map, table, item_index, 0);
  return ret;
}

int nbr_table_remove(nbr_table_t *table, void *item)
{
    return nbr_table_idx_remove(table, index_from_item(table, item));
}

/*---------------------------------------------------------------------------*/
/* Lock a neighbor for the current table (set "locked" bit) */
int
nbr_table_lock(nbr_table_t *table, void *item)
{
#if DEBUG
  int i = index_from_item(table, item);
  LOG_DBG("*** Lock %d\n", i);
#endif
  return nbr_set_bit(locked_map, table, item, 1);
}
/*---------------------------------------------------------------------------*/
/* Release the lock on a neighbor for the current table (unset "locked" bit) */
int
nbr_table_unlock(nbr_table_t *table, void *item)
{
#if DEBUG
  int i = index_from_item(table, item);
  LOG_DBG("*** Unlock %d\n", i);
#endif
  return nbr_set_bit(locked_map, table, item, 0);
}
/*---------------------------------------------------------------------------*/
/* Get link-layer address of an item */
linkaddr_t *
nbr_table_get_lladdr(nbr_table_t *table, const void *item)
{
  // TODO:need check nbr_get_bit( . ) ?
  nbr_table_key_t *key = key_from_item(table, item);
  return key != NULL ? &key->lladdr : NULL;
}

linkaddr_t *nbr_table_idx_lladdr(nbr_idx_t idx){
    // TODO:need check nbr_get_bit( . ) ?
    nbr_table_key_t *key = key_from_index(idx);
    return key != NULL ? &key->lladdr : NULL;
}
/*---------------------------------------------------------------------------*/
#if DEBUG
static void
print_table()
{
  int i, j;
  /* Printout all neighbors and which tables they are used in */
  PRINTF("NBR TABLE:\n");
  for(i = 0; i < NBR_TABLE_MAX_NEIGHBORS; i++) {
    if(used_map[i] > 0) {
      PRINTF(" %02d %03d",i , key_from_index(i)->lladdr.u8[LINKADDR_SIZE - 1]);
      for(j = 0; j < num_tables; j++) {
        PRINTF(" [%d:%d]", (used_map[i] & (1 << j)) != 0,
               (locked_map[i] & (1 << j)) != 0);
      }
      PRINTF("\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
handle_periodic_timer(void *ptr)
{
  print_table();
  ctimer_reset(&periodic_timer);
}
#endif
