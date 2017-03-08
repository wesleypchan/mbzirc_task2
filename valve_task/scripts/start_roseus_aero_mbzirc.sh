gnome-terminal -x /opt/ros/`rosversion -d`/bin/roslaunch aero_startup aero_bringup.launch &

rosnode kill /aero_servo_controller_service


# aero_bringup.launch sometimes dies with 
# "run_id on parameter server does not match declared run_id" error
# if aero_utils is launched immediately after for some reason.
# Need to put in a pause to prevent the error 
sleep 5


## launch aero_utils.launch in the same terminal as roseus
# roslaunch aero_utils aero_utils.launch &
## launch aero_utils.launch in a separate terminal
gnome-terminal -x /opt/ros/`rosversion -d`/bin/roslaunch aero_utils aero_utils.launch &

## for some reason if this is included in aero_utils.launch it causes aero_bringup to stop when run through ssh. Need to put it in a separate launch file to avoid the problem
# gnome-terminal -x /opt/ros/`rosversion -d`/bin/roslaunch aero_utils ground_publisher.launch &

rlwrap roseus `rospack find valve_task`/euslisp/start-roseus-aero-mbzirc.l

