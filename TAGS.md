List of tags/branches of [this Contiki fork](https://github.com/alexrayne/contiki)
========================================================================

[feature/make-extends] - set of make extends

[feature/make-exclude_source] - provide Makefile system with SOURCE_EXCLUDE feature


[feature/os-extends] - set of OS features
--------------------------------------------------------------------------------
- linkaddr.u16[] - unifyed u16[] as array for all addr sizes

[pr/printf-netaddr] - provides printf with dump internal structures:
	- print uipaddr by `%pI`
	- print lladdr by `%pL`

[feature/net-uipaddr-deploy] - moves uip adress defs to 'uipaddr.h'

DEBUG
========================================================================
[feature/debug-os-unstatic] - gives protothreads functions no statics. this allow 
debugger to know code where it run.

TSCH
========================================================================
[feature/tsch-extends] - set of TSCH features
	- fixed PANID handling for LINKADDR_SIZE=2, and control it at ACK

[pr/tsch-rx_relaxcpu-prefetch] - implements wait for receiveing radio frame by fast 
 polling. This releases CPU to APP between polls, so CPU about 90% free when receive frame.

[fix-tsch-activity_onoff] - 

[feature/tsch-fast-gettimesrc] - inline optimisations for tsch_queue_get_time_source

[fix/tsch-cb-joinleave] - TSCH_JOIN/LEAVE events hooks establish

[tsch-scan-flex] - provides TSCH association scan with more strategys:
	- single/loop scaning
	- random/sequental chanels scan

[alexrayne-tsch-slot_prognose_rfoff] - this takes to account that RF power-on takes 
	a time, and so try to prefetch poweron such it be ready at slot start.
	This allows not include poweron time in slot layout - gives more time 
	for radio activity.

[contrib/tsch-link-signaling] - establish event TSCH_CALLBACK_LINK_SIGNAL for links
	marked with LINK_OPTION_SIGNAL

[tsch/6p-extends]
------------------------------------------------------------------------
- introduced new API structures
- concurent transactions in/out and on differ nbrs
- fixed some bugs

MSF
------------------------------------------------------------------------
[msf/workplace] - inroduce MSF schdeler service for TSCH.

[nrsf] - protocol for sync scheduling tables berween nbrs. Allows MSF build 
	no-conflict scheduling.

[feature/cc13-extends] - set of cc13/26xx, most for RF prop mode driver
------------------------------------------------------------------------
- a number of init/powerup fixes, invalid frames handling ...

Optimisations
========================================================================
[speed/inlines] - core optimisation by inlining code

[speed/nbr-index-access] - nbr-table inline optimisations

[speed/ctimer-expireds] - ctimer faster handle expited events addition - they 
	are not arm timers, but passes through to execute

[feature/tsch-fast-gettimesrc] - inline optimisations for tsch_queue_get_time_source

[optimise/dev-slip] - dev/slip - code size optimisation

cooja.
========================================================================
[add-cooja-dbglog] - deploy debug output printing from rs232 output. This gives pure serial
   interconnection, that not fills with debug mesages

[contrib/cooja-coffeeecatch] - `coffeecatch` library suports exception/signal 
  capture in native code, when simulate motes. This provides crash stack-trace 
  when crashed in simulation.

[speed/cooja] - speed up printing in cooja - print direct into debugout buffer.

[feature/cooja-clock-resolution] - supports control on simulator rtimer clock resolution.
	default resolution = 1ms, for some cases need better resolution, and it 
	possible here.

[fix/cooja-radio-startoff] - cooja radion starts turned off. it is valid behaviour.

[feature/cooja-radio-emu] - allows use cc13/26x0 cpu radio configs with cooja mote.
	It allows simulate cooja mote close to original cc13

[fix-cooja-make-printredef] - fixes building cooja targets.
	Objcopy crushes when try to rename symbols in archives. Therefore later simulation
		crushes too.

Other stuff
========================================================================
[contrib/native-slipgate] - native platform now can use board `slipgate` to use serial as
 SLIP tonel for board-routing
