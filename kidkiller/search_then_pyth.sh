#!/bin/bash

echo "waiting for /dev/ttyUSB0..."

while [ ! -e /dev/ttyUSB0 ]; do
    sleep 1
done

echo "/dev/ttyUSB0 found! Running python..."
source # enviornment sett on the rapberry pi
python3 # insert path here after pulling from GitHub on raspberry pi

