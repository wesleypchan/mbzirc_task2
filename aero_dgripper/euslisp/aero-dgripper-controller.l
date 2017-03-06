#!/usr/bin/env roseusins

(ros::load-ros-manifest "aero_utils")
(ros::load-ros-manifest "aero_startup")
(ros::load-ros-manifest "aero_dgripper")
(ros::load-ros-manifest "geometry_msgs")

(load "package://aero_utils/euslisp/aero-servo-controller.l")

(defmethod aero-upper-interface
  (:gripper-servo-enable ()
      (send self :send-servo 7 1)
      )
  (:gripper-servo-disable ()
      (send self :send-servo 7 0)
      )
  (:get-force-vector ()
      (ros::spin-once)
      (setq force-vector-sensor-frame (float-vector rarm_FT_fx rarm_FT_fy rarm_FT_fz))
      (setq ft-sensor-to-end-coords-rot (make-coords :rpy #f(1.570796 3.1415926 0)))
      (send ft-sensor-to-end-coords-rot :rotate-vector force-vector-sensor-frame)
      )
  (:get-torque-vector ()
      (ros::spin-once)
      (float-vector rarm_FT_tx rarm_FT_ty rarm_FT_tz)
       )
  ;; calibrates the force sensor and sets the offset force
  (:get-force-offset ()
    (setq numSamples 100)
    (setq count 0)
    (setq force-sum #f(0 0 0))
    (while (< count numSamples)
        (setq force-sum (v+ force-sum (send self :get-force-vector)))
        (inc count))
    (setq force-offset (scale (/ 1.0 numSamples) force-sum))
    )
  (:get-offset-force-vector ()
    (v- (send self :get-force-vector) force-offset))
  )


  );; end of defmethod aero-upper-interface

(defun callback (data)
    (setq rarm_FT_fx (send (send data :force) :x))
    (setq rarm_FT_fy (send (send data :force) :y))
    (setq rarm_FT_fz (send (send data :force) :z))
    (setq rarm_FT_tx (send (send data :torque) :x))
    (setq rarm_FT_ty (send (send data :torque) :y))
    (setq ramr_FT_tz (send (send data :torque) :z))
    )


(ros::roseus "aero-dgripper-controller")

(ros::subscribe "aero_ft"
                geometry_msgs::Wrench
                #'callback 1)

