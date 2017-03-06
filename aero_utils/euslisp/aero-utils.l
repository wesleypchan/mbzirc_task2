(setq *base-time* 1)
(if (not (boundp `*real*))
    (setq *real* nil))

(defun moveRobot (&optional (time 4000))
  (if (not *real*)
      (progn
        (print "*real* is not set to t")
        (return-from moveRobot))
    (progn
      (if (not (send *aero* :self-collision-check))
          (progn
            ;;(send *ri* :wait-interpolation-smooth)
            (send *ri* :angle-vector (send *aero* :angle-vector) (* time *base-time*))
            )
        (print "collision detected !!")))))


(defun matchSimWithRobot ()
  (send *aero* :angle-vector (send *ri* :state :potentio-vector))
  (if (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects)))

(defun resetPose (&optional (time 4000))
  (send *aero* :reset-pose)
    (if (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects))
  ;; (send *aero* :head :neck-p :joint-angle 5)
  (moveRobot time))

