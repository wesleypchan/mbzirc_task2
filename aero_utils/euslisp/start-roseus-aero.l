(load "package://aeroeus/aero-interface.l")
(load "package://aero_utils/euslisp/aero-utils.l")
(aero-init)
(load-controllers)
(objects (list *aero*))
(resetpose)
