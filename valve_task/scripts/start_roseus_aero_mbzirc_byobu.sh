#!/usr/bin/env bash
PROGNAME=$(basename $0)
WRENCH_TARGET=4

usage() {
  echo "Usage: $PROGNAME [option]"
  echo
  echo "options:"
  echo "  -h, --help"
  echo "  -w, --wrench          option for set wrench target. set 3 for 19mm, 4 for 22mm"
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
    '-w'|'--wrench' )
      if [[ -z "$2" ]] || [[ "$2" =~ ^-+ ]]; then                                                    
        echo "$PROGNAME : -w and --wrench option requires an argument!"                           
      fi
      WRENCH_TARGET="$2"
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

# aero core system ======
byobu-tmux send-keys -t roscore "roscore" C-m
byobu-tmux send-keys -t bringup "/opt/ros/`rosversion -d`/bin/roslaunch aero_startup aero_bringup.launch" C-m
sleep 7
byobu-tmux send-keys -t utils "/opt/ros/`rosversion -d`/bin/roslaunch aero_utils aero_utils.launch" C-m

byobu-tmux send-keys -t roseus "rosnode kill /aero_servo_controller_service" C-m
byobu-tmux send-keys -t roseus "rlwrap roseus `rospack find valve_task`/euslisp/start-roseus-aero-mbzirc.l" C-m
# ====== aero core system

# roseus nodes ======
byobu-tmux new-window -n picking
byobu-tmux send-keys -t picking "rlwrap roseus `rospack find mbzirc_task2_control`/euslisp/catch-wrench.l" C-m
byobu-tmux send-keys -t picking "full-pick $WRENCH_TARGET :auto t :ready? t" # for rehearsal

byobu-tmux new-window -n aligning
byobu-tmux send-keys -t aligning "rlwrap roseus `rospack find mbzirc_task2_control`/euslisp/align-with-panel.l" C-m
byobu-tmux send-keys -t aligning "align-with-panel" # for rehearsal
byobu-tmux new-window -n approach
byobu-tmux send-keys -t approach "rlwrap roseus `rospack find mbzirc_task2_control`/euslisp/approach-panel.l" C-m
byobu-tmux send-keys -t approach "long-approach" # for rehearsal
byobu-tmux new-window -n init-pos
byobu-tmux send-keys -t init-pos "rlwrap roseus `rospack find mbzirc_task2_control`/euslisp/init-position.l" C-m
byobu-tmux send-keys -t init-pos "go-to-init-position :init_x 40000 :init_y 20000" #for rehearsal

# ====== roseus nodes

byobu-tmux attach-session -t aero-mbzirc
