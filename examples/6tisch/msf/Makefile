CONTIKI_PROJECT ?= msf-node msf-root
all: $(CONTIKI_PROJECT)

PLATFORMS_EXCLUDE = sky z1 nrf52dk simplelink
ifeq ("$(BOARD)","")
PLATFORMS_EXCLUDE += native
endif

ifdef APP_SEND_INTERVAL
CFLAGS += -DAPP_SEND_INTERVAL=$(APP_SEND_INTERVAL)
endif

CONTIKI = ../../..

MAKE_MAC = MAKE_MAC_TSCH
MODULES += os/net/mac/tsch/sixtop
MODULES += os/services/msf
MODULES += os/services/shell

ifdef MAKE_WITH_BR
MODULES += os/services/rpl-border-router
endif

include $(CONTIKI)/Makefile.include
