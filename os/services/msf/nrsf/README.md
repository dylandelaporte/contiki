# 6TiSCH Net Resolution Scheduling Function (NRSF)

This SF implements spread `msf-avoid`s cells register over nbrs.

MSF use register of all busy/reserved local and nbr's cells. This register helps 
select new cell that have no conflicts with other, already allocated.

This strategy, gives big effort, if cells of far nbr's ( > 1hop) known - keep from interference with far nodes. 

To get register from far nodes, NRSF sends to all nbr's updates about allocated/deleted 
cells, and sends all register to new connected nodes.

## How to Use

Follow these steps to use MSF:

* add to `MODULES` in your project `Makefile` this:
    - `os/services/msf`
    - `os/services/msf/nrsf`
    - `os/net/mac/tsch/sixtop`

* install `msf` and `nrsf` in your code using `sixtop_add_sf()`

## Configuration

See `nrsf.h`, which has all the configuration parameters.

## Known issues

see `msf/TODO.md`