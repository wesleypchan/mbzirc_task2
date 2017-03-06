(load "package://valve_task/models/panel.l")
(load "package://valve_task/models/wrench.l")
(load "models/tray-object.l")

(defclass task2arena-scene
    :super scene-model
    :slots ())
(defmethod task2arena-scene
    (:init
     (&rest args &key (name "task2arena"))
     (setq *panel* (panel))
     (send *panel* :transform (make-coords :pos (float-vector 600 0 900) :rot (rpy-matrix pi 0 0)))
     (setq *wrench0* (wrench))
     (setq wrench0-pos (send
                        (send
                         (send *panel* :wrench-hook0 :copy-worldcoords)
                         :translate (float-vector 0 0 (/ (send *wrench0* :get-val `wrench-length) -2)) )
                        :pos))
     (send *wrench0* :transform (make-coords :pos wrench0-pos :rot (rpy-matrix 0 (/ pi 2) pi)))
     (setq *wrench1* (wrench 160))
     (setq wrench1-pos (send
                        (send
                         (send *panel* :wrench-hook1 :copy-worldcoords)
                         :translate (float-vector 0 0 (/ (send *wrench1* :get-val `wrench-length) -2)) )
                        :pos))
     (send *wrench1* :transform (make-coords :pos wrench1-pos :rot (rpy-matrix 0 (/ pi 2) pi)))
     (setq *wrench2* (wrench 170))
     (setq wrench2-pos (send
                        (send
                         (send *panel* :wrench-hook2 :copy-worldcoords)
                         :translate (float-vector 0 0 (/ (send *wrench2* :get-val `wrench-length) -2)) )
                        :pos))
     (send *wrench2* :transform (make-coords :pos wrench2-pos :rot (rpy-matrix 0 (/ pi 2) pi)))
     (setq *wrench3* (wrench 220))
     (setq wrench3-pos (send
                        (send
                         (send *panel* :wrench-hook3 :copy-worldcoords)
                         :translate (float-vector 0 0 (/ (send *wrench3* :get-val `wrench-length) -2)) )
                        :pos))
     (send *wrench3* :transform (make-coords :pos wrench3-pos :rot (rpy-matrix 0 (/ pi 2) pi)))
     (setq *wrench4* (wrench 250))
     (setq wrench4-pos (send
                        (send
                         (send *panel* :wrench-hook4 :copy-worldcoords)
                         :translate (float-vector 0 0 (/ (send *wrench4* :get-val `wrench-length) -2)) )
                        :pos))
     (send *wrench4* :transform (make-coords :pos wrench4-pos :rot (rpy-matrix 0 (/ pi 2) pi)))
     (setq *wrench5* (wrench 185))
     (setq wrench5-pos (send
                        (send
                         (send *panel* :wrench-hook5 :copy-worldcoords)
                         :translate (float-vector 0 0 (/ (send *wrench5* :get-val `wrench-length) -2)) )
                        :pos))
     (send *wrench5* :transform (make-coords :pos wrench5-pos :rot (rpy-matrix 0 (/ pi 2) pi)))

     (send-super
      :init :name name :objects
      (list
       *panel*
       *wrench0*
       *wrench1*
       *wrench2*
       *wrench3*
       *wrench4*
       *wrench5*
       ))
     )
    )

(defun task2arena () (setq *task2arena* (instance task2arena-scene :init)))
