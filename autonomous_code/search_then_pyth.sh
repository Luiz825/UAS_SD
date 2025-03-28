#!/bin/bash

STATUS="Activate drone detection! :0"
TIMESTAMP=$(date "+%c")
declare -i lim=5

    echo -e "$STATUS\n$TIMESTAMP"
    echo "$STATUS\n$TIMESTAMP" >> rp5_login_state.txt

for ((i=1; i<=$lim; i++))
do    
    sleep 1
    if [ ! -e /dev/ttyUSB0 ] && [ "$i" -eq $lim ]; then
        STATUS="Was not able to find /dev/ttyUSB0 sry :("          
    elif [ -e /dev/ttyUSB0 ] && [ "$i" -le $lim ]; then
        STATUS="Found /dev/ttyUSB0...! :>"
    else 
        STATUS="waiting for /dev/ttyUSB0..."
    fi 

    TIMESTAMP=$(date "+%c")

    echo -e "$STATUS\n$TIMESTAMP"

    echo "$STATUS\n$TIMESTAMP" >> rp5_login_state.txt                
done    

if [ "$STATUS" = "Was not able to find /dev/ttyUSB0 sry :(" ]; then  
    exit 0
fi
echo -e "/dev/ttyUSB0 found! Running python..."
echo "/dev/ttyUSB0 found! Running python..." >> rp5_login_state.txt        
source /media/cece/DuelData/academic/SDSU/SP2025/COMPE492/UAS_SD/kidkiller/myenv_pymavlink/bin/activate #change to local path
# enviornment sett on the rapberry pi
python3 /media/cece/DuelData/academic/SDSU/SP2025/COMPE492/UAS_SD/kidkiller/indep_thought.py #change for local path

# insert path here after pulling from GitHub on raspberry pi

#folllow steps to set up on rp5 so that the script begins when 
#rp on rebbot
