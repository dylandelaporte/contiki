#!/bin/bash

# Contiki directory

if  [ -z "$1" ]
then 
CONTIKI=$(dirname "$0")/../..
else
CONTIKI=$1
fi

# Simulation file
BASENAME=$(basename $0 .sh)

echo basename:$BASENAME

bash test-border-router.sh $CONTIKI $BASENAME fd00::204:4:4:4 60
