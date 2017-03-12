(load "package://valve_task/models/panel.l")
(load "package://valve_task/models/wrench.l")
(load "package://aero_utils/euslisp/aero-arm-single-wrist-utils.l")
(load "package://aero_dgripper/euslisp/aero-dgripper-controller.l")
(load "package://valve_task/models/task2arena-scene.l")
(load "package://valve_task/euslisp/valve-task-params.l")

;;;;;;;;;; state flow ;;;;;;;;;;
;;    fit-wrench-to-shaft
;;             |
;; detect-shaft-center-state
;;             |
;;     turn-shaft-state


(setq turn-radius wrenchLength)

(setq debug t)

(defun prompt (msg)
    (if debug
            (progn
                (print msg)
                (read-char)))
    )

;; move in shaft direction until force is detected 
(defun detect-shaft-contact ()
  
  (setq contact-force-thres 1.0)
  (get-force-offset)
  (setq offset-force (get-offset-force-vector))
  (setq approach-force (v. offset-force approach-vector))

  ;; move in approach direction until detected force
  (while (< (abs approach-force) contact-force-thres)
    (move-arm (scale 1 approach-vector))
    (unix::sleep 1)
    (setq approach-force (v. (get-offset-force-vector) approach-vector))
    (format t "approach-force ~a~%" approach-force)
         
    )

  ;; move back in the opposite direction to maintain minimum force 
  ;; this is to avoid pressing too hard on the shaft so the wrench head can't slide on it
  (unix::sleep 2) ;; wait for force reading to settle
  (setq approach-force (v. (get-offset-force-vector) approach-vector))
  (while (> (abs approach-force) contact-force-thres)
    (move-arm (scale -0.5 approach-vector)) ;; move in opposite direction 
    (unix::sleep 2)
    (setq approach-force (v. (get-offset-force-vector) approach-vector))
    (format t "approach-force ~a~%" approach-force)
    
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
        (moverobot)
        (send *ri* :wait-interpolation)
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

    (print "exited from turn-shaft-state back to fit-wrench-to-shaft-state")
    start-arm-pose-delta ;; return the arm-pose delta of where the shaft was detected to be used for next call to fit-wrench-to-shaft
    )

;; STATE: scan for contact
;; move in approach-vector direction until touched something 
(defun scan-for-contact-state (start-pos-delta)
       (format t "Entered scan-for-contact-state with start-pos-delta ~a~%" start-pos-delta)
       (setq force-offset (get-force-offset))
       (setq touch-lower-thres 1.0)
       (setq touch-upper-thres 3.5)
       
       (setq target-pose (send (send *robot* arm :end-coords :copy-worldcoords) :translate (float-vector 0 start-pos-delta 0)))
       (setq scan-for-contact-state-start-pose (send target-pose :copy-worldcoords))

       (setq approach-scan-distance 150)
       (setq tangent-scan-distance 50)
       (setq approach-inc-dist 1.0)
       (setq tangent-inc-dist 1.0)       
       
       (setq tangent-index 0) ;; index for scanning in the tangent direction
       (setq target-pose (send *robot* arm :end-coords :copy-worldcoords))
       (while (< tangent-index (* 2 (/ tangent-scan-distance tangent-inc-dist)))
           (format t "tangent-index ~a~%" tangent-index)
           (setq target-pose (send scan-for-contact-state-start-pose :copy-coords))
           (send target-pose :translate (float-vector 0 start-pos-delta 0))
           (if (equal 0 (mod tangent-index 2)) ;; do a widening zigzag scan in the tangent direction 
                   (send target-pose :translate (scale (* (/ tangent-index 2) tangent-inc-dist) tangent-vector))
               (send target-pose :translate (scale (- (* (/ tangent-index 2) tangent-inc-dist)) tangent-vector)))
           (send *robot* arm :inverse-kinematics target-pose)
           (moverobot 300)
           (send *ri* :wait-interpolation)
           (setq approach-index 0) ;; index for scanning in the approach direction 
           (while (< approach-index (/ approach-scan-distance approach-inc-dist))
                   ;; (prompt "scan-for-contact-state: hit enter to step through")
               (send target-pose :translate (scale approach-inc-dist approach-vector))
               (send *robot* arm :inverse-kinematics target-pose)
               (moverobot 500)
               ;; (send *ri* :wait-interpolation)
                   (if (boundp '*irtviewer*)
                           (send *irtviewer* :draw-objects))
                   (unix:usleep 250000)
                   (format t "offset force ~a~%" (get-offset-force-vector))
                   (if (> (abs (v. (v- (get-force-vector) force-offset) approach-vector)) touch-lower-thres) ;; if touch something 
                       (progn
                         (unix:usleep 1000000) ;; wait for two seconds to account for possible slip of wrench on shaft
                         (if (> (abs (v. (v- (get-force-vector) force-offset) approach-vector)) touch-lower-thres) ;; check again to see if really touching something
                             (progn
                               (print "touch-lower-thres reached.")
                               (print "now moving back in opposite direction to achieve minimum touch force.")
                               ;; move back in the opposite direction to maintain minimum force 
                               ;; this is to avoid pressing too hard on the shaft so the wrench head can't slide on it
                               (unix::usleep 300000) ;; wait for force reading to settle
                               (setq approach-force (v. (get-offset-force-vector) approach-vector))
                               (while (> (abs approach-force) touch-lower-thres)
                                 (send target-pose :translate (scale (- approach-inc-dist) approach-vector))
                                 (send *robot* arm :inverse-kinematics target-pose)
                                 (moverobot 500)
                                 (unix::usleep 300000)
                                 (setq approach-force (v. (get-offset-force-vector) approach-vector))
                                 (format t "approach-force ~a~%" approach-force)

                                 )
                               (print "Minimum touch force reached. Exit scan-for-contact-state")

                               ;; (prompt "hit enter to continue...")
                               (return-from scan-for-contact-state))
                           )
                         )
                     )
                   (inc approach-index)
                   )
           (inc tangent-index)
           )
       ) ;; end STATE: scan for contact

(defun detect-shaft-center-state ()
    (setq tangent-force-thres 1.5)
    (setq force-vector (get-offset-force-vector))
    (setq tangent-force (v. force-vector tangent-vector)) 
    (format t "offset force: ~a~%" (get-offset-force-vector))
    (format t "Entered detect-shaft-center-state.~%")
    (slide-to-fit-state)
    )

;; STATE: detect shaft center
;; Move the hand slightly left and right to determine which side the shaft is on
(defun detect-shaft-center-state-old ()
    (setq tangent-force-thres 1.5)
    (setq force-vector (get-offset-force-vector))
    (setq tangent-force (v. force-vector tangent-vector)) 
    (format t "offset force: ~a~%" (get-offset-force-vector))
    (format t "Entered detect-shaft-center-state.~%")
    ;; (prompt "hit enter to continue...")
    (if (< (abs tangent-force) tangent-force-thres)
            (progn ;; if tangent force too small, exit
                (format t "Tangent force too small. Enter slide-to-fit-state~%")
                (slide-to-fit-state)
                )
        (progn ;; else if tangent force above threshold    

            (if (> tangent-force 0) ;; determine which direction to try move
                    (progn
                        (print "detected force from right. Will try to move left.")
                        (setq tangent-inc-dist 1.0) ;; if force pushing to the left, move left
                        )
                (progn
                    (print "detected force from left. Will try to move right.")
                    (setq tangent-inc-dist -1.0)
                    ) ;; else if force pushing to the right, move right
                )

            (format t "tangent-force ~a. inc-dist ~a~%" tangent-force tangent-inc-dist)
            ;; (prompt "hit enter to continue")
            
            (setq tangent-scan-dist 18.0)
            (setq tangent-scan-index 0)
            ;; move hand in the direction of force to see if it can move freely (on one side of the shaft)
            ;; or if a force will be detected (shaft fitted to wrench)
            (while (< tangent-scan-index (/ tangent-scan-dist (abs tangent-inc-dist)))
                (format t "offset force: ~a~%" (get-offset-force-vector))
                (send target-pose :translate (scale tangent-inc-dist tangent-vector) :world) ;; move hand over a bit
                (format t "target-pose ~a~%" (send target-pose :pos))
                (if (> tangent-inc-dist 0)
                        (format t "detecting shaft center. Moving left.~%")
                    (format t "detecting shaft center. Moving right.~%")
                    )
                ;; (prompt "hit enter to continue....")
                (send *robot* arm :inverse-kinematics target-pose)
                (moverobot 300)
                (send *ri* :wait-interpolation)

                (unix:usleep 5000) ;; wait for force sensors to settle
                (setq current-tangent-force (v. (get-offset-force-vector) tangent-vector))
                (format t "current-tangent-force ~a~%" current-tangent-force)
                (if (or   ;; detect if there is force in the other direction
                     (and (> tangent-force 0) (< current-tangent-force (- tangent-force-thres)))
                     (and (< tangent-force 0) (> current-tangent-force tangent-force-thres)))
                        (progn ;; if detected force in the other direction
                            (while (< current-tangent-force tangent-force-thres) ;; move back to acheive near zero tangent force
                                (format t "offset force: ~a~%" (get-offset-force-vector))
                                (format t "moving back to shaft center.~%")
                                ;; (prompt "Hit enter to continue.")
                                (send target-pose :translate (scale (- tangent-inc-dist) tangent-vector) :world) ;; move hand over a bit
                                (send *robot* arm :inverse-kinematics target-pose)

                                (if (< (abs approach-force) touch-lower-thres) ;; force servoing in the approach direction
                                        (progn
                                            (format t "approach-force below touch-lower-thres. Adjust target-pos.~%")
                                            (send target-pose :translate (scale approach-inc-dist approach-vector) :world)
                                            (send loop-start-pose :translate (scale approach-inc-dist approach-vector) :world)
                                            (format t "target pos ~a~%" (send target-pose :pos))
                                            )
                                    )
                                
                                (if (> (abs approach-force) touch-upper-thres) ;; force servoing in the approach direction
                                            (progn
                                                (format t "approach-force above touch-upper-thres. Adjust target-pos.~%")
                                                (print "here 4")
                                                (print "loop-start-pose")
                                                (print loop-start-pose)
    

                                                (send target-pose :translate (scale (- approach-inc-dist) approach-vector) :world)
                                                (send loop-start-pose :translate (scale (- approach-inc-dist) approach-vector) :world)
                                                (format t "target pos ~a~%" (send target-pose :pos))
                                                )
                                )
                                
                                (send *robot* arm :inverse-kinematics target-pose)
                                (moverobot 300)
                                (send *ri* :wait-interpolation)
                                
                                (unix:usleep 5000)
                                (setq current-tangent-force (v. (get-offset-force-vector) tangent-vector))
                                (setq apporach-force (v. (get-offset-force-vector) approach-vector))
                                )
                            (print "Shaft center found. Exit detect-shaft-side")
                            (return-from detect-shaft-center-state 0) ;; wrench is fitted to shaft
                            )
                    )
                (inc tangent-scan-index)
                )
            (format t "Shaft center not caught in wrench. Last tangent force was ~a." tangent-force)
            (print "Exit detect shaft center state.")
            (return-from detect-shaft-center-state tangent-force) ;; also return shaft center pos here ????
            )
        )
    ) ;; end STATE: detect shaft center state

;; STATE: slide to fit
;; Once the wrench has detected contact with force in the approach direciton, but no force in the tangent direction,
;; slide hand left and right while maintaining a downward force to try to ensure/get a better fitting of the wrench to the shaft
(defun slide-to-fit-state ()
    (setq shaft-width 13)
    (format t "Entered slide-to-fit-state~%")
    (setq start-pose (send *robot* arm :end-coords :copy-worldcoords))    
    (setq target-pose (send *robot* arm :end-coords :copy-worldcoords))
    
    (setq force-sensor-wait-time 1000)

    (setq tangent-scan-distance 20.0)
    (setq tangent-inc-dist 2.0)       
    (setq approach-inc-dist 1.0)
    
    (setq detected-force-left nil)
    (setq detected-force-right nil)
    (setq tangent-index 0) ;; index for scanning in the tangent direction
    (setq loop-start-pose (send start-pose :copy-coords))
    (print "loop-start-pose")
    (print loop-start-pose)
    (print "")
    (print "")
    (setq last-loop-state nil)
    (while (< tangent-index (* 2 (/ tangent-scan-distance tangent-inc-dist)))
        (setq target-pose (send loop-start-pose :copy-coords))

        (setq force-vector (get-offset-force-vector))
        (setq tangent-force (v. force-vector tangent-vector)) 
        (setq approach-force (v. force-vector approach-vector)) 

        (setq had-to-servo-force nil)
        (format t "******** while loop begin *********~%")
        (format t "force-vector ~a~%" force-vector)
        (format t "last-loop-state ~a~%" last-loop-state)
        (format t "had-to-servo-force ~a~%" had-to-servo-force)
        (format t "~%")
        (format t "~%")

        ;; first mantain a contact force in the approach direction

        ;; servo force in approach direction

        (format t "had-to-servo-force should be nil: ~a~%" had-to-servo-force)        
        (if (and (< (abs approach-force) touch-lower-thres) (not (equal last-loop-state "force-servo-dec"))) ;; force servoing in the approach direction
                (progn ;; if force below threshold
                  (while (< (abs approach-force) touch-lower-thres)
                    (format t "------ force servoing ------~%")
                    (format t "approach-force below touch-lower-thres. Adjust target-pos.~%")
                    (send target-pose :translate (scale approach-inc-dist approach-vector))
                    (send loop-start-pose :translate (scale approach-inc-dist approach-vector))
                    (format t "target pos ~a~%" (send target-pose :pos))
                    (format t "loop start pos ~a~%" (send loop-start-pose :pos))
                    (format t "~%")

                    ;; check if hand moved too far in approach direction
                    (if (> (abs (v. (v- (send start-pose :pos) (send target-pose :pos)) approach-vector))
                           (* 2 shaft-width))
                        (progn
                          (format t "----- moved to far. Must have missed shaft completely -----~%")
                          (format t "~%")
                          (format t "~%")
                          (return-from slide-to-fit-state (slide-to-fit-state-return-helper))
                          )
                      )

                    
                    (send *robot* arm :inverse-kinematics target-pose)
                    (moverobot 200)
                    (send *ri* :wait-interpolation)
                    
                    (unix:usleep force-sensor-wait-time)
                    (setq force-vector (get-offset-force-vector))
                    (setq current-tangent-force (v. force-vector tangent-vector))
                    (setq approach-force (v. force-vector approach-vector))
                    (format t "force-vector ~a~%" force-vector)
                    (format t "approach-force ~a~%" approach-force)
                    (format t "~%")
                    (format t "~%")
                    )
                  (format t "servoed force, set had-to-servo-force to t~%")
                  (setq had-to-servo-force t)
                  (setq last-loop-state "force-servo-inc")
                  )  ;; end if force below threshold
          (progn  ;; else if force above threshold
            (if (and (> (abs approach-force) touch-upper-thres) (not (equal last-loop-state "force-servo-inc"))) ;; force servoing in the approach direction
              (progn
                  (while (> (abs approach-force) touch-upper-thres)
                    (format t "+++++++ force servoing ++++++~%")
                    (format t "approach-force above touch-upper-thres. Adjust target-pos.~%")
                    (send target-pose :translate (scale (- approach-inc-dist) approach-vector))
                    (send loop-start-pose :translate (scale (- approach-inc-dist) approach-vector))
                    (format t "target pos ~a~%" (send target-pose :pos))
                    (format t "loop start pos ~a~%" (send loop-start-pose :pos))
                    (format t "~%")

                    ;; check if hand moved too far in approach direction
                    (if (> (abs (v. (v- (send start-pose :pos) (send target-pose :pos)) approach-vector))
                           (* 2 shaft-width))
                        (progn
                          (format t "----- moved to far. Must have missed shaft completely -----~%")
                          (format t "~%")
                          (format t "~%")
                          (return-from slide-to-fit-state (slide-to-fit-state-return-helper))
                          )
                      )

                    (send *robot* arm :inverse-kinematics target-pose)
                    (moverobot 200)
                    (send *ri* :wait-interpolation)

                    (unix:usleep force-sensor-wait-time)
                    (setq force-vector (get-offset-force-vector))
                    (setq current-tangent-force (v. force-vector tangent-vector))
                    (setq approach-force (v. force-vector approach-vector))
                    (format t "force-vector ~a~%" force-vector)
                    (format t "approach-force ~a~%" approach-force)
                    (format t "~%")
                    (format t "~%")
                    )
                  (format t "servoed force, set had-to-servo-force to t~%")
                  (setq had-to-servo-force t)
                  (setq last-loop-state "force-servo-dec")
                  )
              )
            ) ;; end else if force above threshold
          ) ;; end if force below threshold

        ;; after force has been served to the right amount
        ;; check if detected shaft left or right
        (format t "check had-to-servo-force: ~a~%" had-to-servo-force)
        (unless had-to-servo-force
          (if (< tangent-force (- tangent-force-thres)) ;; if detected force when moving left (force negative value)
              (progn
                (format t "detected force when moving left~%")
                (format t "~%")
                (if detected-force-right ;; if has already detected force when moving right (force positive value)
                    (progn
                      (format t "----- moving back right to achieve near zero tangent force ------~%")
                      (setq current-tangent-force tangent-force)
                      (format t "current-tangent-force ~a~%" current-tangent-force)
                      (while (< current-tangent-force (- tangent-force-thres)) ;; move back right to achieve near zero tangent force
                        (format t "offset force: ~a~%" (get-offset-force-vector))
                        ;; (prompt "moving back to shaft center....")
                        (send target-pose :translate (scale (- tangent-inc-dist) tangent-vector)) ;; move hand right a bit

                        (if (< (abs approach-force) touch-lower-thres) ;; force servoing in the approach direction
                            (progn
                              (format t "approach-force below touch-lower-thres. Adjust target-pos.~%")
                              (send target-pose :translate (scale approach-inc-dist approach-vector))
                              (send loop-start-pose :translate (scale approach-inc-dist approach-vector))
                              (format t "target pos ~a~%" (send target-pose :pos))
                              )
                          )
                        ;; check if hand moved too far
                        (if (> (v. (v- (send start-pose :pos) (send target-pose :pos)) approach-vector)
                               (* 2 shaft-width))
                            (progn
                              (format t "----- moved to far while centering. Must have missed shaft completely -----~%")
                              (format t "~%")
                              (format t "~%")
                              (return-from slide-to-fit-state (slide-to-fit-state-return-helper))
                              )
                          )
                        (send *robot* arm :inverse-kinematics target-pose)
                        (moverobot 200)
                        (send *ri* :wait-interpolation)
                        
                        (unix:usleep force-sensor-wait-time)
                        (setq current-tangent-force (v. (get-offset-force-vector) tangent-vector))
                        (format t "current-tangent-force ~a~%" current-tangent-force)
                        (setq apporach-force (v. (get-offset-force-vector) approach-vector))
                        )
                      (return-from slide-to-fit-state (confirm-shaft-center-found))
                      )
                  (setq detected-force-left t)) ;; else if has not yet detected force when moving right
                )
            )
          (if (> tangent-force tangent-force-thres) ;; if detected force when moving right 
              (progn
                (format t "detected force when moving right~%") ;; force positive value
                (format t "~%")
                (if detected-force-left ;; if has already detected force when moving left (negative value)
                    (progn
                      (format t "------ moving back left to achieve near zero tangent force ------~%")
                      (setq current-tangent-force tangent-force)
                      (format t "current-tangent-force ~a~%" current-tangent-force)
                      (while (> current-tangent-force tangent-force-thres) ;; move back left to achieve near zero tangent force
                        (format t "offset force: ~a~%" (get-offset-force-vector))
                        ;; (prompt "moving back to shaft center....")
                        (send target-pose :translate (scale tangent-inc-dist tangent-vector)) ;; move hand left a bit
                        
                        (if (< (abs approach-force) touch-lower-thres) ;; force servoing in the approach direction
                            (progn
                              (format t "approach-force below touch-lower-thres. Adjust target-pos.~%")
                              (send target-pose :translate (scale approach-inc-dist approach-vector))
                              (send loop-start-pose :translate (scale approach-inc-dist approach-vector))
                              (format t "target pos ~a~%" (send target-pose :pos))
                              )
                          )

                        ;; check if hand moved too far
                        (if (> (v. (v- (send start-pose :pos) (send target-pose :pos)) approach-vector)
                               (* 2 shaft-width))
                            (progn
                              (return-from slide-to-fit-state (slide-to-fit-state-return-helper))
                              )
                          )
                        
                        (send *robot* arm :inverse-kinematics target-pose)
                        (moverobot 200)
                        (send *ri* :wait-interpolation)
                        
                        (unix:usleep force-sensor-wait-time)
                        (setq current-tangent-force (v. (get-offset-force-vector) tangent-vector))
                        (format t "current-tangent-force ~a~%" current-tangent-force)
                        (setq apporach-force (v. (get-offset-force-vector) approach-vector))
                        )
                      (return-from slide-to-fit-state (confirm-shaft-center-found))
                      )
                  (setq detected-force-right t)) ;; else if has not yet detected force when moving right
                )
            )
          
          ;; check if hand moved too far in approach direction
          (if (> (v. (v- (send start-pose :pos) (send target-pose :pos)) approach-vector)
                 (* 2 shaft-width))
              (progn
                (format t "----- moved to far. Must have missed shaft completely -----~%")
                (format t "~%")
                (format t "~%")
                (return-from slide-to-fit-state (slide-to-fit-state-return-helper))
                )
            )
          
          (format t "------ zig zaging ------~%")
          (if (equal 0 (mod tangent-index 2)) ;; do a widening zigzag scan in the tangent direction 
              (progn
                (format t "move left~%")
                (send target-pose :translate (scale (* (/ tangent-index 2) tangent-inc-dist) tangent-vector))
                (setq last-loop-state "zig-zag-left")
                )
            (progn
              (format t "move right~%")
              (send target-pose :translate (scale (- (* (/ tangent-index 2) tangent-inc-dist)) tangent-vector))
              (setq last-loop-state "zig-zag-right")
              )
            )
          
          (format t "target pos ~a~%" (send target-pose :pos))
          (format t "loop start pos ~a~%" (send loop-start-pose :pos))
          (format t "~%")
          (format t "~%")
          
          (send *robot* arm :inverse-kinematics target-pose)
          (moverobot 200)
          (send *ri* :wait-interpolation)
          
          (unix:usleep force-sensor-wait-time)
          (setq current-tangent-force (v. (get-offset-force-vector) tangent-vector))
          (setq apporach-force (v. (get-offset-force-vector) approach-vector))
          
          
          (inc tangent-index) 
          )
        
        ) ;; end doing zig zag scan 

    ;; done zig zag scan, but still can't find shaft center, so return to start pose
    (send *robot* arm :inverse-kinematics start-pose)
    (moverobot 2000)
    (send *ri* :wait-interpolation)
    
    (slide-to-fit-state-return-helper)

    ) ;; end STATE: slide to fit

(defun slide-to-fit-state-return-helper ()

;;    (return-from slide-to-fit-state-return-helper (confirm-shaft-center-found))
    (cond
     (detected-force-right 1)
     (detected-force-left -1)
     ((equal 0 (mod tangent-index 2)) -1) ;; was moving left
     ((equal 0 (mod tangent-index 2)) 1) ;; was moving right
     )
    )


;; STATE: twist to fit 
;; Twist the wrench left and right while pushing down to get it.
(defun twist-to-fit-state ()
    
    (return-from twist-to-fit-state)
    
    (setq consecutive-peak-increase 3.0) ;; how much the peak increase each time
    (setq next-peak-amplitude consecutive-peak-increase)
    (setq max-peak-amplitude 30.0) ;; maximum amount of yaw to twist
    (setq yaw-angle-inc 3.0) ;; amount of yaw to twist in each step of the loop
    (setq twist-axis #f(0 0 1)) ;; local frame wrt the hand
    (setq current-twist-angle 0) ;; just for keeping track of the current twist angle
    (setq target-pose (send *robot* arm :end-coords :copy-worldcoords))
    (while (< current-twist-angle max-peak-amplitude)
        (prompt "twist-to-fit-state: hit enter to step through")
        (if (> next-peak-amplitude 0) ;; yaw angle increasing 
                (progn
                    (print "(> next-peak-amplitude 0)")
                    (if (< current-twist-angle next-peak-amplitude)
                            (progn ;; current angle hasn't reached next peak yet
                                (send target-pose :rotate (deg2rad yaw-angle-inc) twist-axis) ;; continue increasing
                                (setq current-twist-angle (+ current-twist-angle yaw-angle-inc))
                                )
                        (progn ;; current angle has reached next peak                                   
                            (setq next-peak-amplitude (- next-peak-amplitude))
                            )
                        )
                    )
            (progn ;; yaw angle is decreasing
                (if (> current-twist-angle next-peak-amplitude) 
                        (progn ;; current angle hasn't reached next peak yet
                            (send target-pose :rotate (- (deg2rad yaw-angle-inc)) twist-axis) ;; continue decreasing
                            (setq current-twist-angle (- current-twist-angle yaw-angle-inc))
                            )
                    (progn ;; current angle has reached next peak                                   
                        (setq next-peak-amplitude (+ (- next-peak-amplitude) consecutive-peak-increase))
                        )
                    )
                )
            )
        (if (< (abs (v. (get-offset-force-vector) approach-vector)) touch-lower-thres) ;; try to maintain a downwards force
                (send target-pose :translate (scale approach-inc-dist approach-vector))
            )
        
        (send *robot* arm :inverse-kinematics target-pose)
        (moverobot 300)
        (send *ri* :wait-interpolation)
        (if (boundp '*irtviewer*)
                (send *irtviewer* :draw-objects))
        (unix:usleep 5000)
        
        (setq offset-force-vector (get-offset-force-vector))
               (setq approach-force-vector (scale (v. (v- (get-force-vector) force-offset) approach-vector) approach-vector))
               (setq tangent-force (v- offset-force-vector approach-force-vector))
               (format t "offset force ~a~%" offset-force-vector)
               
               (if (> (norm tangent-force) touch-lower-thres) ;; if touch something 
                       (progn
                           (print "tangent force exceeded threshold. Exit twist-to-fit-state")
                           (return-from twist-to-fit-state)
                           )
                   )
               )
    )  ;; end STATE: twist to fit 

;; STATE: turn shaft state
;; Turn the wrech in a clockwise trajectory after the wrench has fitted to the shaft
(defun turn-shaft-state ()
    (format t "Entered turn-shaft-state.~%")
    (gripper-servo-disable)
    (unless (turn-wrench (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (float-vector 0 0 (- turn-radius)) :world) (- turn-radius 0))
      (progn
        (format t "Turn shaft failed.~%")
        (return-from turn-shaft-state)
        )
      )

    (format t "Turn shaft done.~%")
    ;; (prompt "Hit enter to move hand right...")
;    (send *robot* arm :inverse-kinematics (send (send *robot* arm :end-coords :copy-worldcoords) :translate #f(0 -50 0) :world))
;    (moverobot 2000)
;    (send *ri* :wait-interpolation)
    ;; (prompt "Hit enter to move hand up...")
;    (send *robot* arm :inverse-kinematics (send (send *robot* arm :end-coords :copy-worldcoords) :translate #f(0 0 200) :world))
;    (moverobot 2000)
;    (send *ri* :wait-interpolation)
)

;; setup
(defun setup ()
    (task2arena)
    (objects (list *robot* *task2arena*))
    (if (boundp '*irtviewer*)
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
    (format t "in align-wrench-with-shaft hrp2g rarm end-coords ~a~%" (send *hrp2g* :rarm :end-coords))
    (setq time_now (instance ros::time :now))
;    (if (send *tfl* :wait-for-transform "/RARM_END_COORDS" "valve" time_now 30)
    (if (send *tfl* :wait-for-transform "/RARM_END_COORDS" "valve" (instance ros::time :init) 30)
;    (if (send *tfl* :wait-for-transform "/RARM_END_COORDS" "cluster_decomposer_finaloutput00" time_now 20)
;    (if (send *tfl* :wait-for-transform "/RARM_END_COORDS" "dummy_shaft" time_now 10)
       (progn
         (print "Found shaft tf.")
;         (setq hand-to-shaft-coords (send *tfl* :lookup-transform "/RARM_END_COORDS" "valve" time_now))
         (setq hand-to-shaft-coords (send *tfl* :lookup-transform "/RARM_END_COORDS" "valve" (instance ros::time :init)))
         (setq ground-to-shaft-coords (send *tfl* :lookup-transform "/ground" "valve" time_now))
         (setq link6-to-end-coords (send *tfl* :lookup-transform "/RARM_LINK6" "RARM_END_COORDS" time_now))
         (setq ground-to-link6-coords (send *tfl* :lookup-transform "/ground" "RARM_LINK6" time_now))
         (setq ground-to-end-coords (send *tfl* :lookup-transform "/ground" "RARM_END_COORDS" time_now))
         (format t "hand-to-shaft-coords ~a~%" hand-to-shaft-coords)
         (format t "ground-to-shaft-coords ~a~%" ground-to-shaft-coords)
         (format t "link6-to-end-coords ~a~%" link6-to-end-coords)
         (format t "ground-to-link6-coords ~a~%" ground-to-link6-coords)
         (format t "ground-to-end-coords ~a~%" ground-to-end-coords)
;         (setq hand-to-shaft-coords (send *tfl* :lookup-transform "/RARM_END_COORDS" "cluster_decomposer_finaloutput00" time_now))
;         (setq hand-to-shaft-coords (send *tfl* :lookup-transform "/RARM_END_COORDS" "dummy_shaft" (instance ros::time :init)))
         (setq shaft-world-coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (send hand-to-shaft-coords :pos)))
         (send shaft-world-coords :rotate (send hand-to-shaft-coords :rot))  ;; use shaft orientation
         (cond
           ((equal wrenchSize 15) (setq arm-above-shaft-pos (send (send shaft-world-coords :copy-coords) :translate (float-vector 0 0 240))))
           ((equal wrenchSize 17) (setq arm-above-shaft-pos (send (send shaft-world-coords :copy-coords) :translate (float-vector 0 0 265))))
           )
         ;(setq arm-above-shaft-pos (make-coords :pos (send arm-above-shaft-pos :pos)))

         (if (turn-wrench-test shaft-world-coords turn-radius)
             (format t "turn wrench possible.~%")
           (progn 
             (format t "cannot solve ik for turning-wrench~%")
             (return-from align-wrench-with-shaft nil)
             )
           )

         (matchsimwithrobot)
;         (send *robot* arm :inverse-kinematics arm-above-shaft-pos)
         (send *robot* :inverse-kinematics
               (list (send arm-above-shaft-pos :copy-worldcoords)
                     (send (send *robot* :larm :end-coords) :copy-worldcoords))
               :link-list (list (send *robot* :link-list (send *robot* :rarm :end-coords :parent))
                                (send *robot* :link-list (send *robot* :larm :end-coords :parent)))
               :move-target (list (send *robot* :rarm :end-coords)
                                  (send *robot* :larm :end-coords))
               :rotation-axis (list t t)
               )



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
  (gripper-servo-pos (+ start-gripper-pos 10) 1)
  (unix::sleep 1)
  (format t "gripper err ~a~%" (get-gripper-err))
  (setq detected-force-right (> (abs (get-gripper-err)) gripper-err-thres))

  ;; turn left 
  (gripper-servo-pos (- start-gripper-pos 10) 1)
  (unix::sleep 1)
  (format t "gripper err ~a~%" (get-gripper-err))
  (setq detected-force-left (> (abs (get-gripper-err)) gripper-err-thres))

  ;; turn back to start pos
  (gripper-servo-pos start-gripper-pos 1)
  (unix::sleep 1)

  (if (and detected-force-right detected-force-left)
      (return-from confirm-shaft-center-found 0))
  (if (and detected-force-right (not detected-force-left))
      (return-from confirm-shaft-center-found 1))
  (if (and detected-force-left (not detected-force-right))
      (return-from confirm-shaft-center-found -1))
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
    
;;(setup)

(setq arm :rarm)
(setq debug t)

(setq *real* t)

(get-force-offset)
