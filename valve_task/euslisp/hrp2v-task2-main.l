(load "package://valve_task/euslisp/fit-wrench-to-shaft.l")


;;(setup)

;; (prompt "hit enter to reset pose")
;; (resetpose)
;; (moverobot 4000)


;; move arm to start position
(prompt "hit enter to move hand to start position")
(setq start-arm-pose-delta 0)
(move-arm-to-start-pose start-arm-pose-delta)
;; open hand
;; (send *ri* :stop-grasp arm)
;; (if (equal arm :rarm)
        ;; (send *robot* arm :thumb-r :joint-angle 80)
        ;; (send *robot* arm :thumb-r :joint-angle -80))
;; (moverobot)

;; close hand to grasp wrench
;; (prompt "hit enter to close hand")
;; (unix:sleep 2)
;; (send *ri* :start-grasp arm)

(fit-wrench-to-shaft)
