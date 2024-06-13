#!/bin/bash

bash gnome-terminal --tab -e python3 haptic_control.py
sleep 2
bash gnome-terminal --tab -e python3 tracking_control.py