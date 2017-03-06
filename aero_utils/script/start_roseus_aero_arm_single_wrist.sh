gnome-terminal -x /opt/ros/`rosversion -d`/bin/roslaunch aero_startup aero_bringup.launch &
rosnode kill /aero_servo_controller_service
rosrun aero_utils AeroServoController.py &
# roslaunch aero_dgripper aero_dgripper.launch &
rlwrap roseus `rospack find aero_utils`/euslisp/start-roseus-aero-arm-single-wrist.l
