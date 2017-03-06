#!/usr/bin/env bash
PROGNAME=$(basename $0)

usage() {
  echo "Usage: $PROGNAME [option]"
  echo
  echo "options:"
  echo "  -h, --help"
  echo
  exit 1
}

for OPT in "$@"
do
  case "$OPT" in
    '-h'|'--help' )
      usage
      exit 1
      ;;
    '--'|'-' )
      shift 1
      param+=( "$@" )
      break
      ;;
    -*)
      echo "$PROGNAME: unknown option $(echo $1 | sed 's/^-*//'). Check '$PROGNAME -h'" 1>&2
      exit 1
      ;;
  esac
done

byobu-tmux start-server
byobu-tmux new-session -s aero-mbzirc -d -n roscore \; new-window -n bringup \; new-window -n utils \; new-window -n roseus
sleep 1

byobu-tmux send-keys -t roscore "roscore" C-m
byobu-tmux send-keys -t bringup "/opt/ros/`rosversion -d`/bin/roslaunch aero_startup aero_bringup.launch" C-m
sleep 7
byobu-tmux send-keys -t utils "/opt/ros/`rosversion -d`/bin/roslaunch aero_utils aero_utils.launch" C-m

byobu-tmux send-keys -t roseus "rosnode kill /aero_servo_controller_service" C-m
byobu-tmux send-keys -t roseus "rlwrap roseus `rospack find valve_task`/euslisp/start-roseus-aero-mbzirc.l" C-m

byobu-tmux attach-session -t aero-mbzirc
