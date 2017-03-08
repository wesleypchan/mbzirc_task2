(load "package://valve_task/euslisp/valve-task-params.l")
(load "package://aero_dgripper/euslisp/aero-dgripper-controller.l")
;; make aero arm turn wrench using lower body by drawing circle with waist

(if (not (boundp `*robot*))
        (setq *robot* *aero*))

(setq turn-radius wrenchLength)

(setq debug t)

(defun prompt (msg)
    (if debug
            (progn
                (print msg)
                (read-char)))
    )


;; Generates a circular cranking motion of the arm centerted at @rotation-center,
;; in the plane normal to the z axis of @rotation-center.
;; The starting point is at a distance @radius away from @rotation-center in the x-axis direction.
(defun turn-wrench-torso (radius turn-arc) 

    (setq waistPos (send *ri* :move-waist #f(0 0 0) :local))
    ;; make a cascoord frame of the rotation point
    (setq handle-frame
          (make-cascoords
           :pos (float-vector (elt waistPos 0) 0 (elt waistPos 1))
           )
          )

    ;; make a cascoord frame for the rotation center
    (setq rotation-center-frame
          (send
           (make-cascoords
            :pos (send (send handle-frame :copy-worldcoords) :pos)
            )
           :translate (float-vector 0 0 (- radius))
           )
          )

    ;; assoc handle frame to rotation center frame to be spun around it.
    (send rotation-center-frame :assoc handle-frame) 
    
    ;; (unix:usleep 1000000)
        
    ;; starting tracing circular trajectory
    (prompt "hit enter to start executing trajectory")
    (setq index 0)
    (setq turn-size 10)
    (setq step-time 300)
    (while (< index (+ (/ turn-arc turn-size) 1))
        
        (send rotation-center-frame :rotate (deg2rad turn-size) #f(0 -1 0))
        (setq target-pos (send handle-frame :worldpos))
        (format t "target-pos ~a~%" target-pos)
        (if (equal "nan value error" (send *ri* :move-waist target-pos :world step-time))
            (progn
                (format t "move-waist failed~%")
                (return-from turn-wrench nil)))

        (setq index (+ index 1))
        
        (unix:usleep (* (+ step-time 200) 1000))
        
        (prompt "hit enter to step")
        )
    t
    )
       

;; Generates a circular cranking motion of the arm centerted at @rotation-center,
;; in the plane normal to the z axis of @rotation-center.
;; The starting point is at a distance @radius away from @rotation-center in the x-axis direction.
;;;;;;;; send joint command (moverobot) continuously to prevent current down ;;;;;;;;;;;
(defun turn-wrench-torso2 (radius turn-arc) 

    (format t "enter turn-wrench-torso2~%")

    (gripper-servo-pos 0)
    (send *ri* :get-force-offset)
    (setq waistPos (send *ri* :move-waist #f(0 0 0) :local))
    ;; make a cascoord frame of the rotation point
    (setq handle-frame
          (make-cascoords
           :pos (float-vector (elt waistPos 0)
                              0
                              (- (elt waistPos 1) 10))
           )
          )

    ;; make a cascoord frame for the rotation center
    (setq rotation-center-frame
          (send
           (make-cascoords
            :pos (send (send handle-frame :copy-worldcoords) :pos)
            )
           :translate (float-vector 0 0 (- (+ radius 0)))
           )
          )

    (format t "handle-frame pos ~a~%" (send handle-frame :pos))
    (format t "rotation-center-frame pos ~a~%" (send rotation-center-frame :pos))
    ;; assoc handle frame to rotation center frame to be spun around it.
    (send rotation-center-frame :assoc handle-frame) 
    
    ;; (unix:usleep 1000000)
        
    ;; starting tracing circular trajectory
    (prompt "hit enter to start executing trajectory")
    (setq index 0)
    (setq turn-size 5)
    (setq step-time 300)

    ;; get the robot out of the current down state
    (setq start-time (instance ros::time :now))
    (setq current-time (instance ros::time :now))
    (while (< (- (send current-time :to-sec) (send start-time :to-sec))
              (* (+ step-time 200) 0.001))
        
        (moverobot 100) ;; send arm command continuously to prevent current down
        (setq current-time (instance ros::time :now))
        )
    
    ;; start turning 
    (while (< index (+ (/ turn-arc turn-size) 1))

        (setq force-vector (get-offset-force-vector))
        (format t "force: ~a magnitude: ~a~%" force-vector (norm force-vector))
        (setq force-limit 40)
        (if (> (norm force-vector) force-limit)
                (progn
                    (format t "force magnitude ~a exceeded limit ~a~%" (norm force-vector) force-limit)
                    (format t "exiting turn-wrench-torso2~%")
                    (return-from turn-wrench-torso2 nil)
                    )
            )
        (setq index (+ index 1))
        (send rotation-center-frame :rotate (deg2rad turn-size) #f(0 -1 0))
        (setq target-pos (send handle-frame :worldpos))
        (format t "~a deg~%" (* index turn-size))
        (format t "target-pos ~a~%" target-pos)
        (if (equal "nan value error" (send *ri* :move-waist target-pos :world step-time))
            (progn
                (format t "move-waist failed~%")
                (return-from turn-wrench nil)))

        (setq current-gripper-pos (send *aero* :rarm :dgripper :joint-angle))
        (send *aero* :rarm :dgripper :joint-angle (+ current-gripper-pos turn-size))
        (moverobot (+ step-time 1000))
        
        (prompt "hit enter to step")
        (setq start-time (instance ros::time :now))
        (setq current-time (instance ros::time :now))
        (while (< (- (send current-time :to-sec) (send start-time :to-sec))
                  (* (+ step-time 200) 0.001))
            
            (moverobot 100) ;; send arm command continuously to prevent current down
            (setq current-time (instance ros::time :now))
            )
        
        )
    
    (unix:sleep 1)
    (gripper-servo-disable)
    t
    )

(defun readyTurnWrenchTorso ()
    (resettask2pose)

    (setq target-pos (make-coords :pos #f(0 -430 420)))
    (send target-pos :rotate (deg2rad -90) #f(0 0 1))
    (send *aero* :rarm :inverse-kinematics target-pos)
    (if (boundp '*irtviewer*)
            (send *irtviewer* :draw-objects))
    (moverobot)
    ;; panel height : 55, wasit world coords (0 0 -120), 360 deg arc
    ;; panel height : 30, wasit world coords (-180 0 -280), 120 deg arc
    (send *ri* :move-waist #f(0 0 -120) :world)
    (gripper-servo-enable)
    (gripper-servo-pos 0 2000)
    (unix:sleep 4)
    (gripper-servo-disable)

    )

(defun turnWrenchTorso ()
    (gripper-servo-disable)
    (setq turn-arc 85)
    (turn-wrench-torso turn-radius turn-arc)
    )

(defun turnWrenchTorso2 ()
    (gripper-servo-disable)
    (setq turn-arc 85)
    (turn-wrench-torso2 turn-radius turn-arc)
    )

(defun retractWrench ()
    (matchsimwithrobot)
    (setq retract-direction (transform (send (send *aero* :rarm-end-coords) :worldrot) #f( 0 0 1)))
    (setq retract-vector (scale 30 retract-direction))
    (send *ri* :move- waist retract-vector :local 1000)
    )
