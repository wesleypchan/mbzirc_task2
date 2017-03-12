(ros::load-ros-manifest "trajectory_msgs")

(setq *base-time* 1)
(if (not (boundp `*real*))
    (setq *real* nil))

(defun moveRobot (&optional (time 4000))
  (if (not *real*)
    (progn
        (print "*real* is not set to t")
        (return-from moveRobot))
    (progn
        (if t
      ;; (if (not (send *aero* :self-collision-check))
          (progn
            ;;(send *ri* :wait-interpolation-smooth)
            (send *ri* :angle-vector (send *aero* :angle-vector) (* time *base-time*))
            )
        (print "collision detected !!")))))


(defun matchSimWithRobot ()
  (send *aero* :angle-vector (send *ri* :state :actual-vector))
  (if (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects)))

(defun resetPose (&optional (time 4000))
    (send *aero* :reset-pose)
    (if (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects))
  (moveRobot time))

(defun resetManipPose (&optional (time 4000))
    (send *aero* :reset-manip-pose)
    (if (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects))
    (moveRobot time))

(defun resetTask2Pose (&optional (time 4000))
    (send *aero* :reset-task2-pose)
    (if (boundp '*irtviewer*)
        (send *irtviewer* :draw-objects))
    (moveRobot time)
    (send *ri* :move-waist #f(150 0 -50) :world 7000))

(defun Task2RecogPose (&optional (time 4000))
        ;;(send *aero* :task2-recog-pose)
        (send *aero* :angle-vector #f(-86 0 67 0 0 90 0 0 0 0 0 0))
            (if (boundp '*irtviewer*)
                            (send *irtviewer* :draw-objects))
                (moveRobot time)
                    (send *ri* :move-waist #f(-150 0 -250) :world 7000))

(defun testFitWrenchPose (&optional (time 4000))
        ;;(send *aero* :task2-recog-pose)
        (send *aero* :angle-vector #f(0.012882 32.3977 88.5227 0.005602 -30.9825 0 0 0 0 0 0 0))
            (if (boundp '*irtviewer*)
                            (send *irtviewer* :draw-objects))
                (moveRobot time))

(defun resetDrivingPose (&optional (time 4000))
    ;;(send *robot* :angle-vector #f(0.0 60.0 120.0 0.01 -90.0 1.20858))
    (send *robot* :angle-vector #f(0.0 0.0 67.0 0.0 0.0 90.0 0 0 0 0 0 0))
    (if (boundp '*irtviewer*)
        (send *irtviewer* :draw-objects))
    (moveRobot time)
    (send *ri* :move-waist #f(0 0 -500) :world 7000))

(defun zeroPose (&optional (time 4000))
    (send *aero* :angle-vector #f(0 0 0 0 0 0))
    (if (boundp '*irtviewer*)
        (send *irtviewer* :draw-objects))
    (moveRobot time))


(defun moveHand (delta &optional (time 4000) (debug-view nil))
    (setq target (send *aero* :rarm :end-coords :copy-worldcoords))
    (send target :translate delta)

    (send *aero* :rarm :inverse-kinematics target
          :debug-view debug-view)
    (if (boundp '*irtviewer*)
        (send *irtviewer* :draw-objects))
    (moveRobot time))

(defun moveWaist (delta &optional (wrt :local) (time 2000))
    (if (checkTorsoKinematics delta wrt)
            (send *ri* :move-waist delta wrt time)
        )
    )

(defun gripper-servo-pos (pos &optional (time 2000))
    (setq gripper-offset 0)
    (send *aero* :rarm :dgripper :joint-angle (+ pos gripper-offset))
    (moverobot time))

(defun arm-servo-enable ()
    (send *ri* :arm-servo-enable)
    (matchsimwithrobot)
    )
(defun arm-servo-disable ()
    (send *ri* :arm-servo-disable)
    )
(defun gripper-servo-enable ()
    (send *ri* :gripper-servo-enable)
    (matchsimwithrobot)
    )
(defun gripper-servo-disable()
    (send *ri* :gripper-servo-disable)
    )
(defun lifter-servo-enable ()
    (send *ri* :lifter-servo-enable)
    (matchsimwithrobot)
    )
(defun lifter-servo-disable()
    (send *ri* :lifter-servo-disable)
    )
(defun wheels-servo-enable ()
    (send *ri* :wheels-servo-enable)
    (matchsimwithrobot)
    )
(defun wheels-servo-disable()
    (send *ri* :wheels-servo-disable)
    )


(defun testWheelCmd (vel1 vel2 vel3 vel4 dur_msec)
    (setq traj (instance trajectory_msgs::JointTrajectory :init))
    (send traj :joint_names
          (list "can_front_l_wheel" "can_front_r_wheel" "can_rear_l_wheel" "can_rear_r_wheel"))


    (setq traj_pt (instance trajectory_msgs::JointTrajectoryPoint :init))
    (send traj_pt :positions (list vel1 vel2 vel3 vel4))
    (send traj_pt :time_from_start (instance ros::time :init :nsec (* dur_msec 1000 1000)))
    (send traj :points (list traj_pt))
    (ros::publish "aero_controller/wheel_command" traj)
    )

;; (defun joymovebase (x y theta)
    ;; (setq joymsg (instance sensor_msgs::joy :init
                           ;; :

(ros::roseus "aero-arm-single-wrist-utils")

(ros::advertise "aero_controller/wheel_command" trajectory_msgs::JointTrajectory)
;; (ros::advertise "joy" sensor_msgs::Joy)


(defun checkTorsoKinematics (target_pos wrt)
    (let ((link1_length 360.0) (link2_length 360.0)
         (x_origin 0.0) (z_origin 720.0)
         (x_target (elt target_pos 0)) (z_target (elt target_pos 2))
         (epsilon (deg2rad 1)))

        (print "in checkTorsoKinematics")
        (setq now_pos (send *ri* :move-waist #f(0 0 0) :local 1))
        (setq x_now (+ (elt now_pos 0) x_origin))
        (setq z_now (+ (elt now_pos 1) z_origin))
        (cond
         ((equal wrt :world)
          (progn
              (setq goal_position_x (+ x_origin x_target))
              (setq goal_position_z (+ z_origin z_target))
              )
          )
         ((equal wrt :local)
          (progn
              (setq goal_position_x (+ x_now x_target))
              (setq goal_position_z (+ z_now z_target))
              )
          )
         (t
          (progn
              (print "unexepcted coordinate")
              (return-from checkTorsoKinematics nil)
              )
          )
         )       
        
        (setq to_end_position (+ (* goal_position_x goal_position_x) (* goal_position_z goal_position_z)))
        
        (setq theta
              (acos (/
                     (- to_end_position
                        (* link1_length link1_length)
                        (* link2_length link2_length))
                     (* 2 link1_length link2_length)))
              )
        (setq xi (acos (/
                        goal_position_x
                        (sqrt to_end_position))))
        (setq k1 (+ link1_length (* link2_length (cos theta))))
        (setq k2 (* link2_length (sin theta)))
        (setq gamma (atan (/ k2 k1)))
        (if (< k1 0)
                (setq gamma (+ gamma (deg2rad 180))))
        (setq phi (+ xi gamma (- (deg2rad 90))))
        
        (if (or
             (< phi (- 0 epsilon))
             (> phi (+ theta epsilon))
             (> phi (deg2rad 90)))
                (progn
                    (format t "invalid goal phi = ~a (~a deg)~%" phi (rad2deg phi))
                    (format t "(theta = ~a (~a deg)~%)" theta (rad2deg theta))
                    (return-from checkTorsoKinematics nil)
                    )
            )
        
        (setq nan (acos 2))
        (if (or (equal theta nan) (equal phi nan))
                (progn
                    (print "nan value error")
                    (return-from checkTorsoKinematics nil)
                    )
            )
        
        (format t "knee joint = theta = ~a (~a deg)~%" theta (rad2deg theta))
        (format t "phi = ~a (~a deg)~%" phi (rad2deg phi))
        (format t "hip joint = theta-phi= ~a (~a deg)~%" (- theta phi) (rad2deg (- theta phi)))

        (setq theta_max (deg2rad 180))
        (setq theta_min (deg2rad 0))
        (setq phi_max (deg2rad 90))
        (setq phi_min (deg2rad 0))
        (cond
         ((> theta (+ theta_max epsilon))
          (progn
              (format t "theta ~a exceeds theta max ~a~%" theta theta_max)
              (return-from checkTorsoKinematics nil)
              )
          )
         ((< theta (- theta_min epsilon))
          (progn
              (format t "theta ~a exceeds theta min ~a~%" theta theta_min)
              (return-from checkTorsoKinematics nil)
              )
          )
         ((> phi (+ phi_max epsilon))
          (progn
              (format t "phi ~a exceeds phi max ~a~%" phi phi_max)
              (return-from checkTorsoKinematics nil)
              )
          )
         ((< phi (- phi_min epsilon))
          (progn
              (format t "phi ~a exceeds phi min ~a~%" phi phi_min)
              (return-from checkTorsoKinematics nil)
              )
          )
         )
        t
        )
    )


(defun get-force-vector ()
    (send *ri* :get-force-vector)
    )

(defun get-force-offset ()
    (send *ri* :get-force-offset)
    )

(defun get-offset-force-vector ()
    (send *ri* :get-offset-force-vector)
    )
