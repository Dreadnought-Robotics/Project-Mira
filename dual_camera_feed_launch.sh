#!/bin/bash

source ~/Project-Mira/devel/setup.bash

# gnome-terminal -- bash -c "roslaunch mira_perception rov_perception.launch; exec bash"

# gnome-terminal -- bash -c "roslaunch mira_rov teleop.launch; exec bash"

screen_width=$(xdpyinfo | grep dimensions | sed -r 's/^[^0-9]*([0-9]+x[0-9]+).*$/\1/')
width=$(echo "$screen_width" | cut -d'x' -f1)
height=$(echo "$screen_width" | cut -d'x' -f2)

gnome-terminal --geometry "${width}x${height}+0+0 --window-with-profile=MIRA-Perception -- bash -c "roslaunch mira_perception rov_perception.launch; exec bash"

gnome-terminal --geometry "${width}x${height/2}+0+$((height/2)) --window-with-profile=MIRA-Teleop -- bash -c "roslaunch mira_rov teleop.launch; exec bash"
