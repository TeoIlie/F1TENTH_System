#!/bin/bash
cd /home/jetson/f1tenth_ws/src/F1TENTH_System/scripts
python3 check_steering_angle.py --ros-args --params-file ../f1tenth_stack/config/vesc.yaml
