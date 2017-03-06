;; makes aero turn the wrench using lower body as well.
(load "package://valve_task/models/panel.l")
(load "package://valve_task/models/wrench.l")
(load "package://valve_task/models/task2arena-scene.l")

(if (not (boundp `*robot*))
        (setq *robot* *aero*))

;; specify shaft location relative to hand starting position
(setq shaft-location #f(105 125 15)) ;; works without using lower body
;; (setq shaft-location #f(50 0 15)) ;; uses lower body
(setq turn-radius 150)
(setq debug t)

(defun prompt (msg)
    (if debug
            (progn
                (print msg)
                (read-char)))
    )

;; Do ik with lower-body 
(defun ik-without-lower-body (target-hand-pos)
    
    (if (send *robot* :rarm :inverse-kinematics target-hand-pos
              :rotation-axis :z) ;; try using arm only first 
            (return-from ik-with-lower-body t)
        (return-from ik-with-lower-body nil)
        )
    )

;; Do ik with lower-body 
(defun ik-with-lower-body (target-hand-pos)

    (if (send *robot* :rarm :inverse-kinematics target-hand-pos
              :rotation-axis :z) ;; try using arm only first 
            (return-from ik-with-lower-body t)
        (progn
            ;; if using only arm didn't work, move lower body
            (setq current-hand-pos (send *robot* :rarm :end-coords :copy-worldcoords))
            (setq delta (v- (send target-hand-pos :pos) (send current-hand-pos :pos)))
            ;; adjust height using move-waist
            (send *ri* :move-waist (float-vector 0 0 (elt delta 2))) ;; local
            (send *robot* :translate (float-vector 0 0 (elt delta 2)))
            ;; adjust x-y direction using go-pos
            ;; convert from world frame to local frame
            (setq delta-local (send (make-coords :rot (inverse-matrix (send *robot* :rot))) :rotate-vector delta))
            ;; send to go-pos command
            (send *ri* :go-pos (elt delta-local 0) (elt delta-local 1) 0)
            (send *robot* :translate (float-vector (elt delta-local 0) (elt delta-local 1) 0))
            )
        )
    )

   

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

    ;; visualize the turn trajectory
    (setq traj-disk-frame (send rotation-center-frame :copy-coords))
    (setq thickness 10)
    (setq traj-disk (make-torus (list (float-vector (+ radius thickness) 0 0)
                                      (float-vector radius 0 (- thickness))
                                      (float-vector (- radius thickness) 0 0)
                                      (float-vector radius 0 thickness))))
    (send traj-disk :locate (send traj-disk-frame :pos))
    (send traj-disk :orient
          (car (send traj-disk-frame :rotation-angle))
          (cadr (send traj-disk-frame :rotation-angle)) :world)
    (objects traj-disk)
    
    ;; move hand to starting pose
    (prompt "hit enter to move hand to start pos")
    (setq target-hand-pos (send handle-frame :copy-worldcoords))
    (unless (ik-with-lower-body target-hand-pos)
        (progn
            (format t "IK failed to place hand at start pose~%")
            (return-from turn-wrench nil)))

        (if (boundp '*irtviewer*)
                (send *irtviewer* :draw-objects))
            

    ;; (unix:usleep 1000000)

    ;; starting tracing circular trajectory
    (prompt "hit enter to start executing trajectory")
    (setq index 0)
    (setq turn-size 10)
    (while (< index (/ 360 turn-size))
        
        (send rotation-center-frame :rotate (deg2rad turn-size) #f(0 0 1))
        (setq target-hand-pos (send handle-frame :copy-worldcoords))

        (unless (ik-with-lower-body target-hand-pos)
            (progn
                (format t "IK failed~%")
                (return-from turn-wrench nil)))

        (if (boundp '*irtviewer*)
                (send *irtviewer* :draw-objects))
        
        (setq index (+ index 1))
        (unix:usleep 50000)
        
        )
    t
    )

;; setup
(defun setup ()
    (task2arena)
    (objects (list *robot* *task2arena*))
    (resetpose)
    (send *robot* :reset-coords)
    (send *robot* :rotate (/ pi 2) #f(0 0 1)) ;; rotate robot 90 deg left
    (send *robot* :translate #f(0 0 -30)) ;; lower robot a bit first so it can go up later on.
    (send *ri* :move-waist (float-vector 0 0 -30)) ;; local
    (if (boundp '*irtviewer*)
            (send *irtviewer* :draw-objects))
    (unix:sleep 1)
    )


(setup)

;; position the hand to start pos
(resetpose)
(setq rarm-end-coords (send *robot* :rarm :end-coords :copy-worldcoords))
(send rarm-end-coords :rotate (/ pi 2) #f(1 0 0) :local)
;; (send rarm-end-coords :translate  #f(20 200 0) :local)

(setq start-pose (send rarm-end-coords :copy-worldcoords))
(send *robot* :rarm :inverse-kinematics start-pose)

(setq rotation-center (send start-pose :copy-worldcoords))
(send rotation-center :translate shaft-location :local)


;; (unix:usleep 1000000)
(turn-wrench rotation-center turn-radius)


