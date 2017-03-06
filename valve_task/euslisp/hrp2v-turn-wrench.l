(load "package://valve_task/models/panel.l")
(load "package://valve_task/models/wrench.l")
(load "package://valve_task/models/task2arena-scene.l")


(setq turn-wrench-arc 360)
;(setq turn-wrench-arc 115)

(setq gripper-force-limit 45)
(setq wrench-normal-force-lower-thres 5)
(setq wrench-normal-force-upper-thres 15)
(setq wrench-normal-force-servo-gain 3.0)

;; Generates a circular turning motion of the arm centerted at @rotation-center,
;; in the plane normal to the x axis of @rotation-center.
;; The starting point is at a displacement @radius away from @rotation-center in the z-axis direction.
(defun turn-wrench (rotation-center radius) 
    (send *ri* :start-impedance :rarm :k-p 800)
    (setq original-orientation (send rotation-center :rot))
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
           :translate (float-vector 0 0 (+ (- radius 0))) :world ;; make the radius a bit smaller to give a inwards force
           )
          )

    ;; assoc handle frame to rotation center frame to be spun around it.
    (send rotation-center-frame :assoc handle-frame) 

    ;; move hand to starting pose
    (setq target-hand-pos (send handle-frame :copy-worldcoords))
    (unless (turn-wrench-ik-helper target-hand-pos)
        (progn
            (format t "IK failed to place hand at start pose~%")
            (return-from turn-wrench nil)))
            

    (moverobot 2000)
    (send *ri* :wait-interpolation)

    ;; starting tracing circular trajectory
    (setq index 0)
    (setq turn-size 3)
    (while (<= index (/ turn-wrench-arc turn-size))
;;        (prompt "In turn wrench loop. Hit enter to step....")
        (send rotation-center-frame :rotate (deg2rad turn-size) #f(1 0 0))
        (setq target-hand-pos (send handle-frame :copy-worldcoords))
        (setq target-hand-pos (make-coords :pos  (send handle-frame :worldpos) :rot original-orientation))
        (format t "turn wrench at ~a deg~%" (* index turn-size))
        (format t "target-hand-pos ~a~%" target-hand-pos)

        (unless (turn-wrench-ik-helper target-hand-pos)
            (progn
                (format t "IK failed~%")
                (return-from turn-wrench nil)))

        (if (boundp '*irtviewer*)
            (send *irtviewer* :draw-objects))
        (moverobot 200)

;        (send *ri* :wait-interpolation)
        (unix:usleep 100000)

        ;; check force limit
        (setq offset-force-vector (get-offset-force-vector arm))
        (setq offset-force-wrench-normal (get-offset-force-wrench-normal))
        (setq offset-force-wrench-tangent (get-offset-force-wrench-tangent))
        (format t "offset-force-vector ~a norm ~a~%" offset-force-vector (norm offset-force-vector))
        (format t "normal ~a tangent ~a reject ~a~%" offset-force-wrench-normal offset-force-wrench-tangent (norm (get-offset-force-wrench-tangent-reject)))
        ;; if the force magnitude is too high, terminate
        (if (> (norm offset-force-vector) gripper-force-limit)
            (progn
              (format t "gripper force excceded limit. Terminating turn-wrench~%")
              (return-from turn-wrench)
              )
          )
        (if (< (abs offset-force-wrench-normal) wrench-normal-force-lower-thres)
            (progn
              (format t "----- offset-force-normal too low. Adjust turn center -----~%")
              (send rotation-center-frame :translate (scale wrench-normal-force-servo-gain #f(0 0 -1)))
              (format t "new center ~a~%" rotation-center-frame)
              )
            )
        (if (> (abs offset-force-wrench-normal) wrench-normal-force-upper-thres)
            (progn
              (format t "+++++ offset-force-normal too high. Adjust turn center +++++~%")
              (send rotation-center-frame :translate (scale wrench-normal-force-servo-gain #f(0 0 1)))
              (format t "new center ~a~%" rotation-center-frame)
              )
            )

        (setq index (+ index 1))

        (format t "~%")
        (format t "~%")
        )
    (send *ri* :wait-interpolation)
    (send *ri* :stop-impedance :rarm)
    t
    )

(defun turn-wrench-test (rotation-center radius)

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
            :pos (send (send rotation-center :copy-worldcget-offset-force-wrench-tangentoords) :pos)
            :rot (send (send rotation-center :copy-worldcoords) :rot)
            )
           :translate (float-vector 0 0 (+ (- radius 0))) :world ;; make the radius a bit smaller to give a inwards force
           )
          )

    ;; assoc handle frame to rotation center frame to be spun around it.
    (send rotation-center-frame :assoc handle-frame) 

    ;; move hand to starting pose
    (setq target-hand-pos (send handle-frame :copy-worldcoords))

    (unless (turn-wrench-ik-helper target-hand-pos)
      (progn
        (format t "IK failed to place hand at start pose~%")
        (return-from turn-wrench-test nil))
      )
    

;;    (moverobot 2000)
;;    (send *ri* :wait-interpolation)

    ;; starting tracing circular trajectory
    (setq index 0)
    (setq turn-size 3)
    (while (<= index (/ turn-wrench-arc turn-size))
        
        ;; (prompt "In turn wrench loop. Hit enter to step....")
        (send rotation-center-frame :rotate (deg2rad turn-size) #f(1 0 0))
        (setq target-hand-pos (send handle-frame :copy-worldcoords))

;        (unless (send *robot* :rarm :inverse-kinematics target-hand-pos
 ;                     :rotation-axis :x)
        (unless (turn-wrench-ik-helper target-hand-pos)
            (progn
                (format t "IK failed at ~a deg~%" (* index turn-size))
                (return-from turn-wrench-test nil)))
                
        (if (boundp '*irtviewer*)
                            (send *irtviewer* :draw-objects))
;;        (moverobot 200)
        ;;(send *ri* :wait-interpolation)
;;        (unix:usleep 50000)
        
        (setq index (+ index 1))

        )
    (send *ri* :wait-interpolation)
    t
    )



;; tries to maintain a constant force towards the shaft
(defun turn-wrench-with-force-servo (rotation-center radius) 

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
           :translate (float-vector 0 0 (+ radius)) :world
           )
          )

    ;; assoc handle frame to rotation center frame to be spun around it.
    (send rotation-center-frame :assoc handle-frame) 

    ;; move hand to starting pose
    (setq target-hand-pos (send handle-frame :copy-worldcoords))
    (unless (turn-wrench-ik-helper target-hand-pos)
        (progn
            (format t "IK failed to place hand at start pose~%")
            (return-from turn-wrench nil)))
            

    (moverobot 2000)
    (send *ri* :wait-interpolation)

    ;; starting tracing circular trajectory
    (setq index 0)
    (setq turn-size 3)
    (while (=< index (/ turn-wrench-arc turn-size))
;;        (prompt "In turn wrench loop. Hit enter to step....")
        (send rotation-center-frame :rotate (deg2rad turn-size) #f(1 0 0))
        (setq target-hand-pos (send handle-frame :copy-worldcoords))

        (unless (turn-wrench-ik-helper target-hand-pos)
            (progn
                (format t "IK failed~%")
                (return-from turn-wrench nil)))
                
        (if (boundp '*irtviewer*)
                            (send *irtviewer* :draw-objects))
        (moverobot 500)
        (send *ri* :wait-interpolation)
        
        (setq index (+ index 1))

        )
    t
    )


(defun turn-wrench-ik-helper (target-pos)
  (setq use-torso-for-ik t)
  (If use-torso-for-ik
      (progn
        (send *robot* :inverse-kinematics
              (list (send target-pos :copy-worldcoords)
                    (send (send *robot* :larm :end-coords) :copy-worldcoords))
              :link-list (list (send *robot* :link-list (send *robot* :rarm :end-coords :parent))
                               (send *robot* :link-list (send *robot* :larm :end-coords :parent)))
              :move-target (list (send *robot* :rarm :end-coords)
                                 (send *robot* :larm :end-coords))
              :rotation-axis (list :x t)
              )
        )
    (send *robot* :rarm :inverse-kinematics target-pos
          :rotation-axis :x)
    )
  )
;; (setq turn-radius 180)
;; (turn-wrench (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (float-vector 0 0 (- turn-radius)) :world) turn-radius)

;; (move-arm :rarm #f(0 -50 0))
;; (send *ri* :wait-interpolation)
;; (move-arm :rarm #f(0 0 200))
;; (send *ri* :wait-interpolation)


(defun turnWrench ()
    (turn-wrench (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (float-vector 0 0 (- turn-radius)) :world) (- turn-radius 0))
    )

;; Returns force in normal direction of the wrench,
;; i.e. towards the shaft
(defun get-offset-force-wrench-normal ()
  (setq wrench-unit-vector 
        (float-vector 
         0
         (cos (deg2rad (- (get-gripper-pos) 270)))
         (sin (deg2rad (- (get-gripper-pos) 270)))
         )
        )
  (v. (get-offset-force-vector arm) wrench-unit-vector)
  )

;; Returns force tangent to the wrench,
;; i.e. in the turning direction 
(defun get-offset-force-wrench-tangent ()
  (setq wrench-tangent-unit-vector 
        (float-vector 
         0
         (cos (deg2rad (+ 90 (- (get-gripper-pos) 270))))
         (sin (deg2rad (+ 90 (- (get-gripper-pos) 270))))
         )
        )
  (v. (get-offset-force-vector arm) wrench-tangent-unit-vector)
  )

;; Returns the force that is the reject of the tangent to the wrench,
;; i.e. the 3D force vector minus the force in the turning direction
(defun get-offset-force-wrench-tangent-reject ()
  (setq wrench-tangent-unit-vector 
        (float-vector 
         0
         (cos (deg2rad (+ 90 (- (get-gripper-pos) 270))))
         (sin (deg2rad (+ 90 (- (get-gripper-pos) 270))))
         )
        )
  (setq tangent-force-vector 
        (scale (v. (get-offset-force-vector arm) wrench-tangent-unit-vector)
               wrench-tangent-unit-vector))
  (v- (get-offset-force-vector arm) tangent-force-vector)
  )
