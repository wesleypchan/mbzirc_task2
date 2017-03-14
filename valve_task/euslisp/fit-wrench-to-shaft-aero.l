(load "package://valve_task/models/panel.l")
(load "package://valve_task/models/wrench.l")
(load "package://aero_utils/euslisp/aero-arm-single-wrist-utils.l")
(load "package://aero_dgripper/euslisp/aero-dgripper-controller.l")
(load "package://valve_task/models/task2arena-scene.l")
(load "package://valve_task/euslisp/valve-task-params.l")
(load "package://valve_task/euslisp/aero-arm-turn-wrench-torso.l")

;;;;;;;;;; state flow ;;;;;;;;;;
;;   turn-shaft-full-turn
;;             |
;;    fit-wrench-to-shaft
;;             |
;; detect-shaft-center-state
;;             |
;;;;;      slide-to-fit-state
;;;;;             |
;;             |
;;     turn-shaft-state

(setq arm :rarm)

(setq turn-radius wrenchLength)

(setq debug t)

(defun prompt (msg)
    (if debug
            (progn
                (print msg)
                (read-char)))
    )

(defun turn-shaft-full-turn ()

    (format t "enter turn-shaft-full-turn")
    (setq turn-shaft-start-arm-pose (send *robot* arm :end-coords :copy-worldcoords))
    (setq start-pose-delta 0)
    (setq turn_count 1)
    (while (< turn_count 5)
        (setq start-pose-delta (fit-wrench-to-shaft start-pose-delta))
        (if start-pose-delta ;; if successfully turned without error
                (progn
                    (setq wrench-normal-vector ;; in the direction towards the shaft, in lifer frame
                          (float-vector
                           (sin (deg2rad (get-gripper-pos)))
                           0
                           (- (cos (deg2rad (get-gripper-pos)))))
                          )
                    (setq detach-wrench-move-waist-distance 40)
                    (movewaist (scale detach-wrench-move-waist-distance (v- wrench-normal-vector)))
                    ;; (send *robot* arm :inverse-kinematics arm-above-shaft-pos)
                    (send *robot* arm :inverse-kinematics turn-shaft-start-arm-pose)
                    (moverobot 5000)
                    (movewaist #f(150 0 -50) :world 5000) 
                    (unix:sleep 5)
                    (gripper-servo-enable)
                    (gripper-servo-pos 0 2000)
                    (unix:sleep 2)

                    (setq start-pose-delta (fit-wrench-to-shaft start-pose-delta))
                    (setq turn_count (+ turn_count 1))
                    )
            (progn ;; if failed to turn
                (print "need to implement recovery method")
                (return-from turn-shaft-full-turn nil)
                )
            )
        )
    )


;; Entry point to the entire state machine!
(defun fit-wrench-to-shaft (&optional prev-arm-pose-delta)
    (format t "Entered fit-wrench-to-shaft.~%")
    (gripper-servo-enable)
    (gripper-servo-pos 0 1)

    (setq approach-vector #f(0 0 -1)) ;; local frame wrt the hand
    (setq tangent-vector #f(0 1 0)) ;; local frame wrt the hand

    ;;;;;;;;;;;;; here's the state machine! ;;;;;;;;;;;;;;
    ;;;;;;; state : scan-for-contact-state ;;;;;;;;;;;
    (scan-for-contact-state 0)

    ;; remember state at initial contact
    (setq arm-end-coords-at-initial-contact (send *robot* arm :end-coords :copy-worldcoords))
    (setq tangent-force (v. (get-offset-force-vector) tangent-vector))
    (setq approach-force (v. (get-offset-force-vector) approach-vector))

    ;; (twist-to-fit-state)

    ;;;;;;; state : detect-shaft-center-state ;;;;;;;;;;;;;
    (setq detect-shaft-center-result (detect-shaft-center-state))
    ;; (setq detect-shaft-center-result 0)


    (format t "detect-shaft-center-result ~a~%" detect-shaft-center-result)

    (if (boundp `prev-arm-pose-delta)
            (setq start-arm-pose-delta prev-arm-pose-delta)
        (setq start-arm-pose-delta 0)
        )
    (while (not (eq detect-shaft-center-result 0))
        (unless detect-shaft-center-result
          (progn 
            (format t "detect-shaft-center-result returned nil. Wrench head is not touching shaft at all!")
            (return-from fit-wrench-to-shaft nil)
            )
          )
        (format t "detect-shaft-center-result did not return 0~%")
        (format t "scan-for-contact-state-start-pose ~a~%" scan-for-contact-state-start-pose) 
        (send *robot* arm :inverse-kinematics scan-for-contact-state-start-pose)
        (if (boundp `*irtviewer*)
                (send *irtviewer* :draw-objects))
        (moverobot 1000)
        (send *ri* :wait-interpolation)
        (unix:sleep 2) ;; wait for force sensor values to settle
        (if (< detect-shaft-center-result 0)
                (setq current-delta (* 3 (exp (exp (- (abs detect-shaft-center-result))))))
            (setq current-delta (- (* 3 (exp (exp (- (abs detect-shaft-center-result)))))))
            )
        (setq start-arm-pose-delta (+ start-arm-pose-delta current-delta))
        (format t "detect-shaft-center-result ~a. start-arm-pose-delta now ~a~%" detect-shaft-center-result start-arm-pose-delta)

        ;; here's the state!
        (scan-for-contact-state start-arm-pose-delta)
        (setq detect-shaft-center-result (detect-shaft-center-state))
        
        (format t "detect-shaft-center-result ~a~%" detect-shaft-center-result)
        
        )
    (print "entering turn-shaft-state from fit-wrench-to-shaft-state")

    ;;;;;;;; state : turn-shaft-state ;;;;;;;;;
    (turn-shaft-state)
            (send *irtviewer* :draw-objects))
    )

(defun lift-arm-up ()
    (format t "Entered lift-arm-up.~%")
    (setq arm-start-pose (make-coords :pos #f(370 -470 1055))) ;; BUG IN EUSLISP!!!! setq fails to set the :pos field of the new coordinates object 
    (send arm-start-pose :replace-pos #f(370 -470 1055)) ;; BUG IN EUSLISP!!!! need to use replace-pos to directly change the :pos field
    (format t "arm-start-pose ~a~%" arm-start-pose)
;    (cond
 ;    ((equal wrenchSize 15) (send arm-start-pose :translate #f(50 -90 506)))
  ;   ((equal wrenchSize 17) (send arm-start-pose :translate #f(50 -90 526)))
   ;  )
    (format t "arm-start-pose ~a~%" arm-start-pose)
    ;(send *robot* :reset-pose)
    (send *robot* :head :neck-y :joint-angle -9)
    (send *robot* arm :inverse-kinematics arm-start-pose)
    (if (boundp `*irtviewer*)
            (send *irtviewer* :draw-objects))
    (moverobot)
)


(defun move-arm-to-start-pose ()
    (format t "Entered move-arm-to-start-pose.~%")
    (setq arm-start-pose (make-coords :pos #f(417 -272 607))) ;; BUG IN EUSLISP!!!! setq fails to set the :pos field of the new coordinates object 
    (send arm-start-pose :replace-pos #f(417 -272 607)) ;; BUG IN EUSLISP!!!! need to use replace-pos to directly change the :pos field
    (cond
     ((equal wrenchSize 15) (send arm-start-pose :translate #f(80 -60 506)))
     ((equal wrenchSize 17) (send arm-start-pose :translate #f(80 -60 526)))
     )
    (format t "arm-start-pose ~a~%" arm-start-pose)
    (send *robot* arm :inverse-kinematics arm-start-pose)
    (if (boundp `*irtviewer*)
            (send *irtviewer* :draw-objects))
    (moverobot)

    (send *ri* :wait-interpolation)
    (send arm-start-pose :orient 0 #f(1 0 0) :world)
    (cond
     ((equal wrenchSize 15) (send arm-start-pose :translate #f(0 60 0)))
     ((equal wrenchSize 17) (send arm-start-pose :translate (float-vector 0 60 0)))
     )
    (send *robot* arm :inverse-kinematics arm-start-pose)
    (if (boundp `*irtviewer*)
            (send *irtviewer* :draw-objects))
    (moverobot)
    (send *ri* :wait-interpolation)
    (send arm-start-pose :translate #f(50 0 0))
    (send *robot* arm :inverse-kinematics arm-start-pose)
    (if (boundp `*irtviewer*)
            (send *irtviewer* :draw-objects))
    (moverobot)
    (send *ri* :wait-interpolation)
    )

;; move the hand directly above the detected shaft pos 
(defun align-wrench-with-shaft ()
    (print "entered align-wrench-with-shaft")
    (setq time_now (instance ros::time :now))
;    (if (send *tfl* :wait-for-transform "/RARM_END_COORDS" "valve" time_now 30)
    (if (send *tfl* :wait-for-transform "/rarm_end_coords" "valve" (instance ros::time :init) 30)
;    (if (send *tfl* :wait-for-transform "/rarm_end_coords" "cluster_decomposer_finaloutput00" time_now 20)
;    (if (send *tfl* :wait-for-transform "/rarm_end_coords" "dummy_shaft" time_now 10)
       (progn
         (print "Found shaft tf.")
;         (setq hand-to-shaft-coords (send *tfl* :lookup-transform "/rarm_end_coords" "valve" time_now))
         (setq hand-to-shaft-coords (send *tfl* :lookup-transform "/rarm_end_coords" "valve" (instance ros::time :init)))
         (setq ground-to-shaft-coords (send *tfl* :lookup-transform "/ground" "valve" time_now))
         (setq ground-to-end-coords (send *tfl* :lookup-transform "/ground" "rarm_end_coords" time_now))
         (format t "hand-to-shaft-coords ~a~%" hand-to-shaft-coords)
         (format t "ground-to-shaft-coords ~a~%" ground-to-shaft-coords)
         (format t "ground-to-end-coords ~a~%" ground-to-end-coords)
;         (setq hand-to-shaft-coords (send *tfl* :lookup-transform "/rarm_end_coords" "cluster_decomposer_finaloutput00" time_now))
;         (setq hand-to-shaft-coords (send *tfl* :lookup-transform "/rarm_end_coords" "dummy_shaft" (instance ros::time :init)))
         (setq shaft-world-coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (send hand-to-shaft-coords :pos)))
         (send shaft-world-coords :rotate (send (send *robot* :rarm :end-coords :copy-worldcoords) :rot))  ;; use hand orientation
         (setq arm-above-shaft-pos (send (send shaft-world-coords :copy-coords) :translate (float-vector 0 0 (+ wrenchLength 20))))

         (send *robot* arm :inverse-kinematics arm-above-shaft-pos)


         (if (boundp `*irtviewer*)
             (send *irtviewer* :draw-objects))
         (moverobot)
         (send *ri* :wait-interpolation)
         )
      (progn ;; else if shaft tf not found 
        (print "No shaft detected. Wait for shaft tf failed."))
       )
    )

;; confirm that the chaft has been fitted to the wrench head 
;; by moving the dynamixel motor left and right and check for force
(defun confirm-shaft-center-found ()

  (print "in confirm-shaft-center-found")
  (setq gripper-err-thres 0.5)
  (setq start-gripper-pos (get-gripper-pos))

  ;; turn right 
  (gripper-servo-pos (+ start-gripper-pos 10) 200)
  (unix::sleep 1)
  (format t "gripper err ~a~%" (get-gripper-err))
  (setq detected-force-right (> (abs (get-gripper-err)) gripper-err-thres))

  ;; turn left 
  (gripper-servo-pos (- start-gripper-pos 10) 200)
  (unix::sleep 1)
  (format t "gripper err ~a~%" (get-gripper-err))
  (setq detected-force-left (> (abs (get-gripper-err)) gripper-err-thres))

  ;; turn back to start pos
  (gripper-servo-pos start-gripper-pos 200)
  (unix::sleep 1)

  (if (and detected-force-right detected-force-left)
      (return-from confirm-shaft-center-found 0))
  (if (and detected-force-right (not detected-force-left))
      (return-from confirm-shaft-center-found 1))
  (if (and detected-force-left (not detected-force-right))
      (return-from confirm-shaft-center-found -1))
  (return-from confirm-shaft-center-found 1)
  )

(defun get-gripper-pos ()
    (elt (send *ri* :actual-vector) 5)
    )

(defun get-gripper-err ()
    (-
     (elt (send *robot* :angle-vector) 5)
     (elt (send *ri* :actual-vector) 5)
     )
    )

(defun move-arm-to-start-pose2 ()
    (resetpose)
    (setq arm-start-pose (send *robot* arm :end-coords :copy-worldcoords))
    (send arm-start-pose :orient 0 #f(1 0 0) :world)
    (send arm-start-pose :translate #f(130 60 200))
    (send *robot* arm :inverse-kinematics arm-start-pose)
    (if (boundp `*irtviewer*)
            (send *irtviewer* :draw-objects))
    (moverobot)
    (send *ri* :wait-interpolation)
    )

;; IK for moving arm smoothly in cartesian space.
;; This assumes that shoulder-y is at -90 deg, and use the lifter to
;; move in the y direction, while using the arm to move only in the
;; z-x directions, since the arm does not have enough DOFs to move smoothly
(defun slide-to-fit-move-arm (delta)
    (format t "delta ~a~%" delta)
    (if (and (eq (elt delta 0) 0.0) (eq (elt delta 1) 0.0)) ;; if moving in approach direction
            (progn ;; moving in approach direction, can move faster
                (format t "moving in approach direction, can move faster~%")
                (setq slide-to-fit-moverobot-time 300)
                (setq target-pose (send *robot* arm :end-coords :copy-worldcoords))
                (send target-pose :translate delta)
                (send *robot* arm :inverse-kinematics target-pose)
                (moverobot slide-to-fit-moverobot-time)
                ;; (send *ri* :wait-interpolation)
                (unix:usleep (+ (* slide-to-fit-moverobot-time 1000) 100000))
                )
        
            (progn
                (format t "moving in tangent direction, can move slower~%")
                (setq slide-to-fit-moverobot-time 300)
                (setq target-pose (send *robot* arm :end-coords :copy-worldcoords))
                (send target-pose :translate delta)
                (send *robot* arm :inverse-kinematics target-pose)
                (moverobot slide-to-fit-moverobot-time)
                ;; (send *ri* :wait-interpolation)
                (unix:usleep (+ (* slide-to-fit-moverobot-time 1000) 1000000))
                )
            )
        )
        
;;(setup)

(setq arm :rarm)
(setq debug t)

(setq *real* t)

(get-force-offset)
