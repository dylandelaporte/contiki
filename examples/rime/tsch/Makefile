CONTIKI=../../..
CONTIKI_PROJECT = node
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

CONTIKI_WITH_RIME = 1

#MODULES += core/net/mac/tsch
MAKE_MAC=MAKE_MAC_TSCH

CONTIKI_TARGET_SOURCEFILES = trace.cpp

#default platform sources, if not found in target platform 
EXTERNALDIRS += platform

all: $(CONTIKI_PROJECT)

include $(CONTIKI)/Makefile.include
