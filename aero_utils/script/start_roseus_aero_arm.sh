gnome-terminal -x /opt/ros/`rosversion -d`/bin/roslaunch aero_startup aero_bringup.launch &
rlwrap roseus `rospack find aero_utils`/euslisp/start-roseus-aero-arm.l
