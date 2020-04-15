CONTIKI_PROJECT = msf-root
all: $(CONTIKI_PROJECT)

PLATFORMS_EXCLUDE = sky z1 nrf52dk native simplelink

ifdef APP_SEND_INTERVAL
CFLAGS += -DAPP_SEND_INTERVAL=$(APP_SEND_INTERVAL)
endif

CONTIKI = ../../..
MSF_APP_DIR = ../msf

PROJECTDIRS += ${MSF_APP_DIR}

MAKE_MAC = MAKE_MAC_TSCH

MODULES += $(CONTIKI_NG_SERVICES_DIR)/rpl-border-router
MODULES += os/net/mac/tsch/sixtop
MODULES += os/services/msf
# MODULES += $(MSF_APP_DIR)

include $(CONTIKI)/Makefile.include
