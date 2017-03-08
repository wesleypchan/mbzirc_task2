(setq wrenchSize 22) 

(cond
 ((equal wrenchSize 15) (setq wrenchLength 180))
 ((equal wrenchSize 16) (setq wrenchLength 188))
 ((equal wrenchSize 17) (setq wrenchLength 205))
 ((equal wrenchSize 18) (setq wrenchLength 214))
 ((equal wrenchSize 19) (setq wrenchLength 225))
 ;; ((equal wrenchSize 22) (setq wrenchLength 265)) ;; longer version
 ((equal wrenchSize 22) (setq wrenchLength 260)) ;; longer version
 ;; ((equal wrenchSize 22) (setq wrenchLength 227)) ;; shorter version
 ((equal wrenchSize 24) (setq wrenchLength 283))
 )   
