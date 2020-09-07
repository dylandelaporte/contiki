# 6TiSCH Minimal Scheduling Function (MSF)

This module implements
[draft-ietf-6tisch-msf-08](https://tools.ietf.org/html/draft-ietf-6tisch-msf-08).

## How to Use

Follow these steps to use MSF:

* add `os/services/msf` and `os/net/mac/tsch/sixtop` to `MODULES` in
  your project `Makefile`
* install `msf` in your code using `sixtop_add_sf()`

An example is provided under [examples/6tisch/msf](../../../examples/6tisch/msf/README.md).

## Shell commands
You can enable shell commands for MSF following these steps:

* add `os/services/shell` to `MODULES` in your project `Makefile`
* initialize the shell by `serial_shell_init()`

`msf help` gives you a list of available commands. As of writing, you
will get a list shown below:

```
#0012.4b00.060d.9edf> msf help
MSF commands
'> msf help': show this message
'> msf cnst': show constants (configuration parameters)
'> msf stat': show current status
'> msf nego': show negotiated cells
'> msf auto': show autonomous cells
```

## Configuration
See `msf.h`, which has all the configuration parameters.

## Known issues

* EBs and DIOs can be transmitted by a node which doesn't have a
   negotiated TX cell to its parent.

## improvements vs base Contiki-NG implementation

+ `msf-avoid` module - register of all used cells, local and at nbrs.

+ reserving cells take to account avoid cells. it now smarter - can allocate 
    cells in single slot on different chanels.

+ relocate slot action now provide variant - only migrate chanel, if no one slot
    avail.

+ detection of autonomous cells to parent conflicts with any other cells.
    This sutiation forces negotiate RX/TX cell to parent.

+ `msf-negotiated` gives hooks `MSF_AFTER_CELL_USE/RELESE`, `MSF_ON_NEW_NBR`

+ `nrsf` protocol service - spreads used/leaved cells over nbrs, and updates 
    local avoid register.

+ timeout for operations with parent now slit from other nbrs.
