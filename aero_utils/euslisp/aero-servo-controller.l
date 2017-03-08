#!/usr/bin/env roseusins

(ros::load-ros-manifest "aero_startup")
(ros::load-ros-manifest "aero_utils")

(ros::roseus "aero_servo_controller")

;; (ros::wait-for-service "/aero_servo_controller")

(defmethod aero-upper-interface
  (:send-servo (ID data)
    (let (req
	  res
	  )
      (setq req (instance
		 aero_utils::AeroServoControllerRequest
		 :init))
      (send req :id ID)
      (send req :data data)
      (setq res (ros::service-call "/aero_servo_controller" req))
      )
      )
  (:wheels-servo-enable ()
    (send self :send-servo 8 1)
    )
  (:wheels-servo-disable ()
    (send self :send-servo 8 0)
    )
  (:torso-servo-enable ()
    (send self :send-servo 9 1)
    )
  (:torso-servo-disable ()
    (send self :send-servo 9 0)
    )
  (:servo-enable ()
    (send self :send-servo -1 1)
    )
  (:servo-disable ()
    (send self :send-servo -1 0)
    )
  );; end of defmethod aero-upper-interface
