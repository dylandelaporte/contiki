CONTIKI = ../..

CONTIKI_PROJECT = example-abc example-mesh example-collect example-trickle example-polite \
     example-runicast example-unicast example-neighbors

all: $(CONTIKI_PROJECT)

CONTIKI_WITH_RIME = 1
include $(CONTIKI)/Makefile.include

# platform may provide CFS storage, that have some more examples for that
ifeq ($HAVE_CFS,1)
CONTIKI_PROJECT += example-rudolph1 example-rudolph2 example-rucb
endif
