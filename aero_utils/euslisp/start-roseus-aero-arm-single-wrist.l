(load "package://aeroeus/aero-arm-single-wrist-interface.l")
(load "package://aero_utils/euslisp/aero-arm-single-wrist-utils.l")
(aero-init)
(load-controllers)
(load "package://aero_utils/euslisp/aero-servo-controller.l")
(load "package://aero_dgripper/euslisp/aero-dgripper-controller.l")
;;(objects (list *aero*))
(matchSimWithRobot)
;(send *ri* :go-pos 0 0 0) ;; to lock wheels
