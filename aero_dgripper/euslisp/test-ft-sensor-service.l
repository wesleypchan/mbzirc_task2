#!/usr/bin/env roseus

(ros::load-ros-manifest "aero_dgripper")
(ros::wait-for-service "/aero_ft_sensor_service")

(while t
    (setq req (instance aero_dgripper::AeroFTSensorRequest :init))
    (setq res (ros::service-call "/aero_ft_sensor_service" req) )
    (print (send (send (send res :wrench) :force) :x))
    )
