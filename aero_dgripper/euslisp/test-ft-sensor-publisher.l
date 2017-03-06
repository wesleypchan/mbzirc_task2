#!/usr/bin/env roseus

(ros::load-ros-manifest "aero_dgripper")
(ros::load-ros-manifest "geometry_msgs")

(ros::roseus "test-ft-sensor-publisher")

(setq fx 0)
(setq fy 0)
(setq fz 0)
(setq tx 0)
(setq ty 0)
(setq tz 0)

(defun callback (data)
    (print "in callback")
    (setq fx (send (send data :force) :x))
    (setq fy (send (send data :force) :y))
    (setq fz (send (send data :force) :z))
    (setq tx (send (send data :torque) :x))
    (setq ty (send (send data :torque) :y))
    (setq tz (send (send data :torque) :z))
    )

(ros::subscribe "aero_ft"
                geometry_msgs::Wrench
                #'callback 1)

(ros::rate 1000)
(while (ros::ok)
    (ros::spin-once)
    (ros::sleep)
    (format t "~a ~a ~a~%" fx fy fz)

