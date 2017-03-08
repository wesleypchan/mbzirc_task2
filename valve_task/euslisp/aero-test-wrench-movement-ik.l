;; This script tries to make the robot do a circular cranking motion
;; at various locations to see if the IK is solvable.
;; Run this scirpt after initializing the robot to do the test.
;; The successful center of rotations will be stored in the variable succeeded-points
;; when the script completes. 

;; NOTE:
;; With turn-radius=100, an offset of approximately #f(75 20 20) from restpose seem to be
;; the center of the aero's right hand workspace.
;;
;; With turn-radius=130, an offset of approximately #f(85 25 15) from restpose seem to be
;; the center of the aero's right hand workspace.
;;
;; Turn-radius=130 seems to be max when using only arm movement.


(load "package://valve_task/models/panel.l")
(load "package://valve_task/models/wrench.l")
(load "package://valve_task/models/task2arena-scene.l")

(if (not (boundp `*robot*))
        (setq *robot* *aero*))

;; parameters to be tested 
;; (setq xmax 100)
;; (setq xmin 0)
;; (setq ymax 50)
;; (setq ymin -10)
;; (setq zmax 50)
;; (setq zmin -10)
;; (setq step-size 10)

(setq xmax 85)
(setq xmin 65)
(setq ymax 30)
(setq ymin 10)
(setq zmax 30)
(setq zmin 10)
(setq step-size 5)



(setq turn-radius 120)





;; Generates a circular cranking motion of the arm centerted at @rotation-center,
;; in the plane normal to the z axis of @rotation-center.
;; The starting point is at a distance @radius away from @rotation-center in the x-axis direction.
(defun turn-wrench (rotation-center radius) 

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
           :translate (float-vector (- radius) 0 0)
           )
          )

    ;; assoc handle frame to rotation center frame to be spun around it.
    (send rotation-center-frame :assoc handle-frame) 

    ;; move hand to starting pose
    (setq target-hand-pos (send handle-frame :copy-worldcoords))
    (unless (send *robot* :rarm :inverse-kinematics target-hand-pos
          :rotation-axis :z)
        (progn
            (format t "IK failed to place hand at start pose~%")
            (return-from turn-wrench nil)))
            

    ;; (unix:usleep 1000000)

    ;; starting tracing circular trajectory
    (setq index 0)
    (setq turn-size 10)
    (while (< index (/ 360 turn-size))
        
        (send rotation-center-frame :rotate (deg2rad turn-size) #f(0 0 1))
        (setq target-hand-pos (send handle-frame :copy-worldcoords))

        (unless (send *robot* :rarm :inverse-kinematics target-hand-pos
                      :rotation-axis :z)
            (progn
                (format t "IK failed~%")
                (return-from turn-wrench nil)))
                
        (if (boundp '*irtviewer*)
                            (send *irtviewer* :draw-objects))
        
        (setq index (+ index 1))
        ;; (unix:usleep 10000)
        
        )
    t
    )

;; setup
(defun setup ()
    (task2arena)
    (objects (list *robot* *task2arena*))
    (resetpose)
    )


(setup)

;; (defun runtest ()
;; position the hand to start pos
(resetpose)
(setq rarm-end-coords (send *robot* :rarm :end-coords :copy-worldcoords))
(send rarm-end-coords :rotate (/ pi 2) #f(1 0 0) :local)

(setq start-pose (send rarm-end-coords :copy-worldcoords))
(send *robot* :rarm :inverse-kinematics start-pose)




(setq succeeded-points (list))

(setq xindex 0)
(while (<= xindex (/ (- xmax xmin) step-size))
    (setq yindex 0)
    (while (<= yindex (/ (- ymax ymin) step-size))
        (setq zindex 0)
        (while (<= zindex (/ (- zmax zmin) step-size))

            (resetpose) 
            (setq offset (float-vector (+ xmin (* step-size xindex)) (+ ymin (* step-size yindex)) (+ zmin (* step-size zindex))))
            (setq rotation-center (send start-pose :copy-worldcoords))
            (send rotation-center :translate offset :local)

            (format t "offset ~a~%" offset)
            (format t " rotation-center ~a~%" rotation-center)
            ;; (unix:usleep 1000000)
            (if (turn-wrench rotation-center turn-radius)
                    (progn
                        (format t "~a succeeded~%~%~%~%" offset)
                        (setq succeeded-points (list succeeded-points offset))
                        )
                (format t "~a failed~%~%~%~%" offset)
                )
            (setq zindex (+ zindex 1))
            )
        (setq yindex (+ yindex 1))
        )
    (setq xindex (+ xindex 1))
    )
     
;; )
