#!/usr/bin/env roseus

(load "package://aero_utils/euslisp/start-roseus-aero-arm-single-wrist.l")
(setq *real* t)
;(resettask2pose)
;(moverobot)
(load "package://valve_task/euslisp/aero-arm-turn-wrench-torso.l")
(load "package://valve_task/euslisp/aero-arm-turn-wrench-arm.l")

(defun start-mbzirc-teleop ()
    (load "package://valve_task/euslisp/aero-ps3joy-mbzirc-teleop.l")
    )


(defun testFTSensorPose ()
    (send *aero* :reset-task2-pose)
    (send *aero* :rarm :shoulder-y :joint-angle -30)
    (moverobot)
    (movehand #f(100 0 -300))
    )

(defun calibFTSensorPose ()
    (send *aero* :reset-pose)
    (send *aero* :rarm :shoulder-p :joint-angle 0)
    (send *aero* :rarm :elbow-p :joint-angle 0)
    (moverobot)
    (movewaist #f( 0 0 -200)))

    
