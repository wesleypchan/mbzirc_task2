(load "package://valve_task/euslisp/valve-task-params.l")

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
(defun turnWrenchArmImpl (rotation-center radius) 

    ;; make a cascoord frame of the rotation center 
    (setq rotation-center-frame
          (make-cascoords
           :pos (send (send rotation-center :copy-worldcoords) :pos)
           :rot (send (send rotation-center :copy-worldcoords) :rot)
           )
          )

    ;; make a cascoord frame for the crank handle
    (setq handle-frame
          (send
           (make-cascoords
            :pos (send (send rotation-center :copy-worldcoords) :pos)
            :rot (send (send rotation-center :copy-worldcoords) :rot)
            )
           :translate (float-vector 0 0 (+ (- radius 20))) :world ;; make the radius a bit smaller to give a inwards force
           )
          )

    ;; assoc handle frame to rotation center frame to be spun around it.
    (send rotation-center-frame :assoc handle-frame) 
    
    ;; (unix:usleep 1000000)

    ;; starting tracing circular trajectory
    (prompt "hit enter to start executing trajectory")
    (setq index 0)
    (setq turn-step-size 10)
    (setq step-time 500)
    (setq turn-wrench-arc 100)
    (while (< index (/ turn-wrench-arc turn-step-size))
        
        (prompt "In turn wrench loop. Hit enter to step....")
        (send rotation-center-frame :rotate (deg2rad turn-step-size) #f(1 0 0))
        (setq target-hand-pos (send handle-frame :copy-worldcoords))

        (unless (send *robot* :rarm :inverse-kinematics target-hand-pos
                      ;; :rotation-axis :x
                      )
            (progn
                (format t "IK failed~%")
                (return-from turnWrenchArmImpl nil)))
                
        (if (boundp '*irtviewer*)
                (send *irtviewer* :draw-objects))
        (moverobot 200)
        ;;(send *ri* :wait-interpolation)
        (unix:usleep 50000)
        
        (setq index (+ index 1))

        )
    t
    )

;; put arm in a start position where it can turn the wrench by 90 deg.
(defun readyTurnWrenchArm ()
    (setq turn-center-pos #f(50 -435 300))
    (setq target-coords (make-coords :pos (v+ turn-center-pos (float-vector 0 0 turn-radius))))
    (send target-coords :rotate (deg2rad -90) #f(0 0 1))
    (send *aero* :reset-task2-pose)
    (send *aero* :rarm :inverse-kinematics target-coords)
    (if (boundp '*irtviewer*)
            (send *irtviewer* :draw-objects))
    (moverobot)
    (send *ri* :gripper-servo-disable)
    )

;; turn wrench using the current arm point as the starting point
(defun turnWrenchArm ()
    (setq turn-center (send (send *aero* :rarm :end-coords :copy-worldcoords) :translate (float-vector 0 0 (- turn-radius)) :world))
    (turnWrenchArmImpl turn-center turn-radius)
    )

;; turn wrench using the given point as the turn center pos
(defun testTurnWrenchArmPoint (turn-center-pos)
    (setq turn-center (send (make-coords :pos turn-center-pos) :rotate (deg2rad -90) #f(0 0 1)))
    (turnWrenchArmImpl turn-center turn-radius)
    )


;; test a bunch of points to see if IK is solvable for turning wrench
(defun testTurnWrenchArm ()
    ;; resettask2pose pos: 0.0 -475.0 230.0
    (setq xmax 300.0)
    (setq xmin -300.0)
    (setq ymax -375.0)
    (setq ymin -675.0)
    (setq zmax 530.0)
    (setq zmin 30.0)
    (setq step-size 5.0)
    
    
    (setq succeeded-points (list))
    
    (setq xindex 0)
    (while (<= xindex (/ (- xmax xmin) step-size))
        (setq yindex 0)
        (while (<= yindex (/ (- ymax ymin) step-size))
            (setq zindex 0)
            (while (<= zindex (/ (- zmax zmin) step-size))

                (setq start-pos (float-vector (+ xmin (* step-size xindex)) (+ ymin (* step-size yindex)) (+ zmin (* step-size zindex))))
                (setq start-coords (send (make-coords :pos start-pos) :rotate (deg2rad -90) #f(0 0 1)))
                (setq turn-center (send (send start-coords :copy-worldcoords) :translate (float-vector 0 0 (- turn-radius)) :world))
                
                
                (if (send *robot* :rarm :inverse-kinematics start-coords)
                        (progn
                            (if (boundp '*irtviewer*)
                                    (send *irtviewer* :draw-objects))
                            
                            (format t "t~%~%urn-center ~a~%" turn-center)
                            (prompt "hit enter to continue")
                            ;; (unix:usleep 1000000)
                            (if (turnWrenchArmImpl turn-center turn-radius)
                                    (progn
                                        (format t "~a succeeded~%~%~%~%" (send turn-center :pos))
                                        (setq succeeded-points (list succeeded-points (send turn-center :pos)))
                                        )
                                (format t "~a turning failed~%~%~%~%" (send turn-center :pos))
                                )
                            )
                    (progn
                        (format t "~%~%turn-center ~a~%" turn-center)
                        (format t "failed to reach initial position~%")
                        (prompt "hit enter to continue")
                        )
                    )
                (setq zindex (+ zindex 1))
                )
            (setq yindex (+ yindex 1))
            )
        (setq xindex (+ xindex 1))
        )
    
    (print "succeeded-points")
    (print succeeded-points)

    )
