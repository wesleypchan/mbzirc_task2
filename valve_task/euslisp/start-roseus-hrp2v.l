#!/usr/bin/env roseus

(ros::load-ros-manifest "tf")

(load "package://hrp2v_utils/euslisp/start_roseus_hrp2.l")
(load "package://hrp2v_dynamixel_controller/euslisp/gripper_controller.l")

;; need to correct rarm end pos since we have a costomized gripper
(ros::roseus "valve_task_start_roseus_hrp2v")
(setq *tfl* (instance ros::transform-listener :init))
(if (not (send *tfl* :wait-for-transform "RARM_LINK6" "RARM_END_COORDS" (ros::time 0) 0.3))
        (progn
            (print "ERROR: can't tf can't find transformation between RARM_LINK6 and RAMR_END_COORDS")
            (exit)
            )
    )
(setq rarm-end-coords (send *tfl* :lookup-transform "RARM_LINK6" "RARM_END_COORDS" (ros::time 0)))
(send *hrp2w* :rarm :end-coords :replace-pos (send rarm-end-coords :pos))

(defun start-mbzirc-teleop ()
    (load "package://valve_task/euslisp/hrp2v-ps3joy-mbzirc-teleop.l")
    )

