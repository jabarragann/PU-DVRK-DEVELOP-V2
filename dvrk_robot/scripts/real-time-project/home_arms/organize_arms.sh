#!/bin/bash

read -p "setup arms. DANGER! Check the coordinates before executing. Type to (Y) to execute " userInput

VAR1="Y"

if [ "$userInput" = "$VAR1" ]; then
    printf '%s\n' "executing.." 
    printf '%s\n' "psm1.." 
    python home_psm1.py
    printf '%s\n' "psm2.." 
    python home_psm2.py
    printf '%s\n' "psm3.." 
    python home_psm3.py
    printf '%s\n' "mtml.." 
    python home_mtml.py
    printf '%s\n' "mtmr.." 
    python home_mtmr.py

else
    printf '%s\n' "cancelling"
    exit 1
fi
