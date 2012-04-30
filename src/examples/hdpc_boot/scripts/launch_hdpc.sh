#!/bin/bash

source $HOME/.bashrc.ros

# rosclean purge
rm -rf $HOME/.ros/log/*
screen -dmS hdpc -t hdpc -O sh -c "echo waiting...;sleep 10;roslaunch hdpc_drive hdpc_teleop.launch"


