(load "package://aero_utils/euslisp/aero-arm-single-wrist-utils.l")
(load "package://valve_task/euslisp/aero-ps3joy-mbzirc-teleop.l")


(setq cartWidth 50)

(defun prompt (msg)
    (if debug
            (progn
                (print msg)
                (read-char)))
    )


(defun getSingleXDist (index) 
    (* (elt ranges index) (cos  (+ angle_min (* index angle_increment))))
    )

(defun getIndex ()
    )

(defun isThereObstacleWithinDist (distX)

    (setq angle_limit (atan (/ (/ cartWidth 2) distX)))
    (setq minIndex (/ (- (- angle_limit) angle_min) angle_increment))
    (setq maxIndex (/ (- angle_limit angle_min) angle_increment))

    (setq index minIndex)
    (setq pointsWithinDistXcount 0)
    (while (< index maxIndex)
        (if ( < (getSingleXDist index) distX)
                (setq pointsWithinDistXcount (+ pointsWithinDistXcount 1)))
        (setq index (+ index 1))
        )
    (format t "% points with in distx ~f : ~f~%" distx (/ pointsWithinDistXcount (- maxIndex minIndex)))
    )

(defun callbackScan (msg)
    (format t "in callbackScan")
    (format t "size ~d~%" (length (send msg :ranges)))

    (format t "range [0] ~f~%" (elt (send msg :ranges) 0))

    
    (setq angle_min (send msg :angle_min))
    (setq angle_max (send msg :angle_max))
    (setq angle_increment (send msg :angle_increment))
    (setq ranges (send msg :ranges))
 
    (format t "x front  ~f~%" (getsinglexdist 360))
    (isthereobstacleWithinDist 25.0)
      
    )

(ros::roseus "aero_ps3_joy_mbzirc_teleop")

(ros::subscribe "/scan"
                sensor_msgs::LaserScan
                #'callbackScan 1)

;; (ros::rate 10)
(while (ros::ok)
    (ros::spin-once)
    ;; (ros::sleep)
    )
