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
  (moveRobot time))

(defun resetManipPose (&optional (time 4000))
    (send *aero* :reset-pose)
    (send *aero* :rarm :shoulder-p :joint-angle 30)
    (send *aero* :rarm :elbow-p :joint-angle 60)
    (if (boundp '*irtviewer*)
        (send *irtviewer* :draw-objects))
    (moveRobot time))

(defun moveArm (delta &optional (debug-view nil))
    (setq target (send *aero* :rarm :end-coords :copy-worldcoords))
    (send target :translate delta)
    (send *aero* :rarm :inverse-kinematics target
          :debug-view debug-view)
    (if (boundp '*irtviewer*)
        (send *irtviewer* :draw-objects))
    (moveRobot time))
