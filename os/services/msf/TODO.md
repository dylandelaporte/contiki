msf
========================================================================
msf-sixp
------------------------------------------------------------------------
1) *request_wait_timer* - need support for per-nbr timers.
    now it supports common timeout for all no-parent nbrs. this can lead into 
    bottle neck, when multiple nbrs broke their operations.

msf-reserved
------------------------------------------------------------------------
2) *relocate* - need adecvate algorithm to design with of concurent cells should
    relocate. Now relocates local cell always, if it can, not fixed.
    This algorithm leads to inefficient relocation of both cells of conflict, and 
    none of them leave owns cell. 

nrsf
========================================================================
1) protocol: need `loop resolution`, i.e. distinguish far cells vs echo of local
    cells. This now limits detecting conflicts to far cells, only close cells 
    conflicts are takes to action -> relocating.

6top
========================================================================
1) `sixp_cell` provide with sized to uint8_t slot/chanel fields. 
    Now it is unefficient structure, since we use less <256 slots/chanels

2) *packets coupling* - different SFs may produce thair packets at same time. 
    if couple them into one radio packet, it gives huge effort for msf/nrsf 
    speedup.