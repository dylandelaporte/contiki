<img src="https://github.com/contiki-ng/contiki-ng.github.io/blob/master/images/logo/Contiki_logo_2RGB.png" alt="Logo" width="256">

# Contiki-NG: The OS for Next Generation IoT Devices

[![Build Status](https://travis-ci.org/contiki-ng/contiki-ng.svg?branch=master)](https://travis-ci.org/contiki-ng/contiki-ng/branches)
[![Documentation Status](https://readthedocs.org/projects/contiki-ng/badge/?version=master)](https://contiki-ng.readthedocs.io/en/master/?badge=master)
[![license](https://img.shields.io/badge/license-3--clause%20bsd-brightgreen.svg)](https://github.com/contiki-ng/contiki-ng/blob/master/LICENSE.md)
[![Latest release](https://img.shields.io/github/release/contiki-ng/contiki-ng.svg)](https://github.com/contiki-ng/contiki-ng/releases/latest)
[![GitHub Release Date](https://img.shields.io/github/release-date/contiki-ng/contiki-ng.svg)](https://github.com/contiki-ng/contiki-ng/releases/latest)
[![Last commit](https://img.shields.io/github/last-commit/contiki-ng/contiki-ng.svg)](https://github.com/contiki-ng/contiki-ng/commit/HEAD)

Contiki-NG is an open-source, cross-platform operating system for Next-Generation IoT devices. It focuses on dependable (secure and reliable) low-power communication and standard protocols, such as IPv6/6LoWPAN, 6TiSCH, RPL, and CoAP. Contiki-NG comes with extensive documentation, tutorials, a roadmap, release cycle, and well-defined development flow for smooth integration of community contributions.

Unless explicitly stated otherwise, Contiki-NG sources are distributed under
the terms of the [3-clause BSD license](LICENSE.md). This license gives
everyone the right to use and distribute the code, either in binary or
source code format, as long as the copyright license is retained in
the source code.

Contiki-NG started as a fork of the Contiki OS and retains some of its original features.

Find out more:

* GitHub repository: https://github.com/contiki-ng/contiki-ng
* Documentation: https://github.com/contiki-ng/contiki-ng/wiki
* Web site: http://contiki-ng.org
* Nightly testbed runs: https://contiki-ng.github.io/testbed

Engage with the community:

* Gitter: https://gitter.im/contiki-ng
* Twitter: https://twitter.com/contiki_ng

This fork (brief):
============================
This fork is born in project with TSCH net, star topology.
 
Used frequent beacons - about 8/sec, for faster net association. Used multilevel frames - short, and long. This Frames interacts, so that activation short frame depends on long one.

Syncronising with net was optimised by listen beacons as rare as possible, depends on estimation of drift by adaptive sync algorithm. This gives much power consumption economy.
Therefore in adaptive sync was intoduced hooks on new estimation comes.  

For association - provided more control over chanels list, and chanel hoping style. 

Also multiple slots can be attached to work with single receiver, and with different protocols on different ports. 

Used AES128 encription, but there was myltiple ports - wityhout encription, and different keys. 
Such work demands some flexibility with encription of links and packets - reailised assigning key per recv/sender adress and port.   

Has imroved cc26xx RFcore-prop driver. 
Most important - provided power control, that use GLDO during receive. Linear LDO hav eless noise - and gives about 12db for SNR.

Also improved ake system - it now can build in separate dir, and more clever on dependencies, and cache it.

And other little work for core library, etimer and rtimer, cc26xx LPM, SPI ... 


This fork improves:
============================

contiki core improves
--------------------------------
+ rtime:
    - [rtimer\_multiple\_access] - PR #1290
    - rtime\_expired() / RTIME\_EXPIRED

+ process\_abort() [add-proc-abort]

+ [add-net-addrsize4] - allow use 4byte network adress.
+ [fix-rame802154-addrsize] - fixes MAC adress check when LINKADDR_SIZE != 8

+ CRC16\_STYLE [add-crc16-lookup8 ] - core crc16 code provided in variants - full soft calculate, or
    via faster table lookup 

+ [fix-assert-cpp ] - fix contiki code vs c++ <assert.h>

+ core/lib/ringbuf16index - 16bit ringbiffers

+ fixes on make system:
    - ![fix-make-deps] - fix make dependences
    - [alexrayne-make-cpp] - DEBUG symbol pass to compiler
    - [fix-make-loose-objects] - prevent delete project objects that have deps but not included in contiki lib
    - [fix-alexrayne-cc26-srec] - fix srec make routine when  build on windows platform
    - [bkozak/build_in_seperate_dir] - PR#1417 We can now set BUILDDIR to build in seperate dir
    - [ivan-alekhin/customrules-fix] - Allow to search "customrules" in additional directories.
    - [fix-make-dotpath] - ommits '.' dir in SOURCEDIRS if one alredy provided PROJECTDIRS
     

+ [optimise-list-inlines] - core lists ontimisation by inlining code

2) Imported PRs
-------------------------------
- PR #2347 [fix-make-dotpath]
- PR #2338 from tim-ist/bmp_fix
- [ivan-alekhin/customrules-fix]
- PR #1417 [bkozak/build\_in\_seperate\_dir] - We can now set BUILDDIR to build in seperate dir
- [pengi/apps\_from\_platform] - Load platform before apps
- PR #1265 [bkozak/optomize\_etimer\_implementation] - etimer\_expired\_at, timer\_expired
- PR #1290 [rtimer\_multiple\_access]
- PR #2599 [lavr18-add-cc26-i2c] - Add board-i2c library for the launchpad 

Work fo Launchpad cc26xx platform:
-------------------------------
- fix SPI driver for LPM system
- [fix-cc26xx-launchpad-spiclose] allowssome SPI pins maybe assigned as unused 
- fix UART driver for LPM system

- +[cc26xx-dev-crypto] - add AES128 core driver

- cc1310 RF core:
    * power mode controls allow linear DC for receive [cc26-rf-prop-gldo]
    * fix power on with bad quartz [add-cc26xx-rfosc-timeout]
    * fixes over RF RAT timer sync vs core clock
    * RF cores features TSCH\_HW\_FEATURE\_xxx:
        - RF\_CORE\_CONF\_PENDING - a bit styles over pending receiving frames. Introduced RF\_CORE\_PENDING\_READS style - for more relyable pending receiver.
    * surropt RF prop mode RADIO\_PARAM\_LAST\_RSSI

- RTC CH2 isr hander [cc26xx-add-rtc-isr-ch2]  

- RTIMER\_CONF\_ARCH\_SECOND - provide RTC resolution setup. now tick of RTC can be setup

- [bkozak/add\_rtimer\_multiple\_access] - import PR for rtimer safer work

3) just a bit fine code:
-------------------------------
+ [tsch-fine-alexraynes], [fine-tsch-sfhandle]

+ [cc26xx-lpm-fines]
    * improve LPM with  WAKE\_UP\_FROM\_DEEP\_SCHEDULE 
+ [fine-cc26-lpm]
    * RF\_CORE\_PENDING\_READSANY - provide protection vs buggy RFcore leaves slots pending state.
+ process\_post\_pause() [process-pause-codesize] - for less code size

+ [cc26xx-rtc-fastercode]

Work over TSCH net stack
============================= 
introduce configurations:
------------------------------------

- TSCH\_CONF\_POLLING\_STYLE - TSCH\_POLLING\_STRONG/RELAXED [tsch-pending-smart]

- TSCH\_RADIO\_ON\_DURING\_TIMESLOT when smarter RF core power off/on between slots, allows RF well be powered on close slots [tsch\_prognose\_rf\_off]

- TSCH\_CONF\_TIMESYNC\_EB\_LOOSES - allow denote how much EB beacons can be loose, before net loose sync

- TSCH\_CONF\_RTIMER\_GUARD - turning for RTIMER\_GUARD feature
 
- tsch\_timeslot\_timing\_elements:tsch\_ts\_netwide\_count - denotes timings that assigns for network by EB. Allow to fix local network timings, ignoring ones from network EB. (handy for debug)

- TSCH\_SECURITY\_CONF\_STRICT - better confiigurable net sequrity:

    * \_PAN\_RELAX - allow unsecured packets in secured PAN or secured packets in unsecured PAN

    * \_KEYID\_RELAX - allow use any key_id for secured frames, regards frame-type

- TSCH\_CONF\_WITH\_PHANTOM\_NBR [tsch-pantom\_addr] - phantom TSCH adress (if != eb_adress) binds width adress declared in links. This allows use different queues on same receiver adress, denoted to different links. i.e. - different links can have theirs own packet queue, even if them works on same receiver adress.


TSCH association control [tsch-scan-flex]:
------------------------------------------
- TSCH\_CONF\_ASSOCIATION\_SINGLE - single or cycling associating retryes
- TSCH\_JOIN\_STYLE - HOPPING\_RANDOM/STEPPING - style of frequences scaning when associating 
- TSCH\_JOIN\_HOPPING\_SEQUENCE - sequensce for frequency scaning
- TSCH\_CONF\_JOIN\_HOPPING\_START(action) macro - provides user specified action on net join
- [tsch-scan-handle-radiofail] - fix association hung when radio failes  
- TSCH\_CONF\_SEQ\_FROMRT - provides initialisation packet seq from RTC [tsch-seq-from-rt]
- TSCH\_ASSOCIATION\_POLL\_FREQUENCY - when use polling on THSC slot listen, this param set up frequency of rx checks
- fixes on assosianiton proc:
    + [alexrayne-tsch-scan-yelds]
- add events JOINING/LEAVING\_NETWORK callbacks invoke on coordinator start, disassociate [fix-tsch-cb-leave]     
- TSCH\_CONF\_SEQ\_FROMRT - provides initialisation packet seq from RTC [tsch-seq-from-rt]

more control on scheduler: [tsch-sched-disables]:
-------------------------------------------
- LINK\_OPTION\_PLANPOINT - allow rescheduling on link packet
- LINK\_OPTION\_DISABLE - allow turn off link softly, without rebuild frames structule
- LINK\_OPTION\_IDLED\_AUTOOFF - provide autoset disable option when link inactive. for TX slots/
- TSCH\_SCHEDULE\_OMMIT\_NOXFER - allows to skip slots that have no RX or TX links activity
- tsch\_slot\_operation\_break\_before/invalidate [tsch-sched-invalidate] - allows to force reschedule next slot by demant
- tsch\_slot\_operation\_stop -
- tsch\_schedule\_link\_change\_option/addr - link option change on fly< no need to debulf frames
- TSCH\_WITH\_LINK\_SELECTOR - allows to pass desired frame*timeslot in packet attributes PACKETBUF\_ATTR\_TSCH\_SLOTFRAME/TIMESLOT
- TSCH\_LINK\_SELECTOR\_ENABLED\[RX\] [tsch-linksel-app] - 
- TSCH\_ON\_SLOT\_TX/RX() [tsch-slot-hooks ] - user hooks on scheduler slots events - after slot completes 

introduce log system:
----------------------------------------
- TSCH\_LOGF configurable macro like PRINTF
- TSCH\_PUTS/PRINTF/ANNOTATE - provide instead global PRINTF

tsch\_queue improves:
----------------------------------------
- tsch\_queue\_free\_neighbors(tsch\_free\_UNLINKED) [tsch-add-clenupques] - allows to cleanup from loosed links
- tsch\_queue\_flush\_nbr\_queue/_remove_nbr [tsch-link-adr_change] - more control on neighbours  

more clever tsch\_activate vs turnoff/sleep/poweroff [fix-tsch-activity_onoff]
--------------------------------------- 

improve adaptive TSCH frame period estimation via sync drift [tsch-adaptive_sync-estimate]
---------------------------------------
- TSCH\_DRIFT\_SYNC\_ESTIMATE - allow rarely listen EB beacons for sync, for lower power consumption.
- TSCH\_TIMESYNC\_ON\_DRIFT(...) - provides user handle on drift sync extimation 
- [alexrayne-tsch-drift\_fail\_protect] - when RF RAT timer unsync with  core clock, measured sync drift may be disaster. this is detects it, and resync RAT. 

security keys:
---------------------------------------
- [tsch-log-sec-keyid]
- TSCH\_SEC\_KEY [tsch-sec-user] - provide user specified keys for packets encription
- better sequrity error handling via tsch\_security\_secure\_packet - it provides errors tschERR\_UNSECURED/NOKEY, mac provide error MAC\_TX\_ERR\_SEC
- [fix-tsch-sec-nonce]

fixes:
---------------------------------------
+! [tsch-fix-slots-stop] - a bit fixes over planing rtimer, when disassociate.

+! [fix-tsch-orchestra-linksel] - a bit fixes on orchestra

+! [fix-tsch-seq-ffff] - fixes for net packet numbers ffff

+ [alexrayne-tsch-slot\_prep\_time] - RTIMER\_GUARD now work on all slot ops, at tsch\_schedule\_slot\_operation. TSCH\_SCHEDULE\_AND\_YIELD now use for accurate positioninig in slot.


some code optimisations:
---------------------------------------
- [opt-tsch-frame802154\_has\_panid]
- [tsch-rtime-optimise]
- [tsch-scan-share\_eb\_parse ] - code size optimise
- +TSCH\_CONF\_H - introduced for lightweight code rebuild on tsch parameters change
- tsch\_queue\_get\_time\_source [tsch-fast-gettimesrc] - fast implementation, not use search over nbr list