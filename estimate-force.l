(defmethod gait-generator
  (:solve-angle-vector
    (support-leg support-leg-coords swing-leg-coords cog
     &key (solve-angle-vector :solve-av-by-move-centroid-on-foot)
          (solve-angle-vector-args))
    (let ((ik-ret
           (cond
            ((functionp solve-angle-vector)
             (apply solve-angle-vector
                    support-leg support-leg-coords
                    swing-leg-coords cog robot solve-angle-vector-args))
            ((and (symbolp solve-angle-vector) (find-method self solve-angle-vector))
             (send* self solve-angle-vector
                    support-leg support-leg-coords
                    swing-leg-coords cog robot solve-angle-vector-args))
            (t (error ";; in :solve-angle-vector, invalid function or method~%!")))))
      (list ik-ret (send (car (send robot :links)) :copy-worldcoords))))
  (:solve-av-by-move-centroid-on-foot
   (support-leg support-leg-coords swing-leg-coords cog robot
   &rest args &key (cog-gain 3.5) (stop 100) (additional-nspace-list) &allow-other-keys)
   (let* ((legs (append (send self :get-counter-footstep-limbs support-leg)
                        support-leg))
          (leg-order (mapcar #'(lambda (l) (position l legs)) all-limbs))
          (fix-coords (append swing-leg-coords support-leg-coords)))
     (unless (memq :thre args)
       (append args (list :thre (mapcar #'(lambda (x) (if (memq x '(:rleg :lleg)) ik-thre (* 5 ik-thre))) all-limbs))))
     (unless (memq :rthre args)
       (append args (list :rthre (mapcar #'(lambda (x) (deg2rad (if (memq x '(:rleg :lleg)) ik-rthre (* 5 ik-rthre)))) all-limbs))))
     (send* robot :move-centroid-on-foot
            :both all-limbs :target-centroid-pos cog
            :fix-limbs-target-coords (mapcar #'(lambda (idx) (elt fix-coords idx)) leg-order)
            :cog-gain cog-gain :stop stop
            ;;:additional-nspace-list
            ;;(append
            ;; additional-nspace-list
            ;; (list (list (car (send robot :links))
            ;;             #'(lambda ()
            ;;                 (let* ((fcoords (apply #'midcoords 0.5
            ;;                                        (mapcar #'(lambda (x) (elt fix-coords (position x legs))) '(:rleg :lleg))))
            ;;                        (xvr (send robot :rotate-vector (case (length legs) (2 #f(1 0 0)) (4 #f(0 0 1)))))
            ;;                        (xvf (send fcoords :rotate-vector #f(1 0 0))))
            ;;                   (if (> (elt (send fcoords :inverse-rotate-vector xvr) 0) 0) (setq xvr (scale -1 xvr)))
            ;;                   (dolist (xv (list xvr xvf)) (setf (elt xv 2) 0.0))
            ;;                   (let ((dth (if (and (eps= (norm xvf) 0.0) (eps= (norm xvr) 0.0))
            ;;                                  0.0 (asin (elt (v* (normalize-vector xvf) (normalize-vector xvr)) 2)))))
            ;;                     (float-vector 0 0 0 0 0 dth)
            ;;                     ))))))
            args)))
  )

(defun jaxoninit ()
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
  ;; (load "package://control_tools/log-tools.l")
  (load "package://control_tools/play-hrpsysfile.l")
  (unless (boundp '*robot*) (setq *robot* (jaxon_red)))
  (send *robot* :fix-leg-to-coords (make-coords))
  (objects (list *robot*))
  (when (y-or-n-p "make *ri*?") (jaxon_red-init))
  )


(defun calc-zmp-from-imu ()
  
  )


(defun calc-zmp-from-force-sensor (&key (d 30))
  ;;(send *ri* :force-vector :larm) (send *ri* :moment-vector :larm) to get value from 6 axis sensor
  ;;leave about the arms' sensor, make this by legs' sensor only for chukan's experiment
  ;; d = height from sensor to touch point
  
  (let*
      ((rleg-fv (send *ri* :state :force-vector :rleg))
       (lleg-fv (send *ri* :state :force-vector :lleg))
       (rarm-fv (send *ri* :state :force-vector :rarm))
       (larm-fv (send *ri* :state :force-vector :larm))
       (rleg-mv (send *ri* :state :moment-vector :rleg))
       (lleg-mv (send *ri* :state :moment-vector :lleg))
       (rarm-mv (send *ri* :state :moment-vector :rarm))
       (larm-mv (send *ri* :state :moment-vector :larm));; :state is needed because it renews the values
       ;; (send *ri* :state) to renew all the values

       (p-rleg-x (/ (- (- (elt rleg-mv 1)) (* (elt rleg-fv 0) d)) (* (elt rleg-fv 2) 1.0)))
       (p-rleg-y (/ (- (elt rleg-mv 0) (* (elt rleg-fv 1) d)) (* (elt rleg-fv 2) 1.0)))
       (p-lleg-x (/ (- (- (elt lleg-mv 1)) (* (elt lleg-fv 0) d)) (* (elt lleg-fv 2) 1.0)))
       (p-lleg-y (/ (- (elt lleg-mv 0) (* (elt lleg-fv 1) d)) (* (elt lleg-fv 2) 1.0)))

       (p-x (/ (+ (* p-rleg-x (elt rleg-fv 2)) (* p-lleg-x (elt lleg-fv 2))) (* 1.0 (+ (elt rleg-fv 2) (elt lleg-fv 2)))))
       (p-y (/ (+ (* p-rleg-y (elt rleg-fv 2)) (* p-lleg-y (elt lleg-fv 2))) (* 1.0 (+ (elt rleg-fv 2) (elt lleg-fv 2)))))
       )

    (setq zmp-from-force-sensor (list p-x p-y))
  )
  )

(defun calc-zmp-fs ()
  
  (send *ri* :state)
  (setq zmp-fs (send *ri* :zmp-vector))
  )

(defun calc-zmp-imu ()
  (send *ri* :state)
  (setq zmp-imu (send *ri* :act-capture-point-vector))
  )

(defun calc-zmp-diff ()
  (send *ri* :state)
  (setq zmp-diff (v- (send *ri* :act-capture-point-vector) (send *ri* :zmp-vector)))
  )



(defun check-z (&key (jaxon-mass 130))
  (send *ri* :state)
  (let*
      ((rleg-fv (send *ri* :force-vector :rleg))
       (lleg-fv (send *ri* :force-vector :lleg))
       (rarm-fv (send *ri* :force-vector :rarm))
       (larm-fv (send *ri* :force-vector :larm))
       (rleg-mv (send *ri* :moment-vector :rleg))
       (lleg-mv (send *ri* :moment-vector :lleg))
       (rarm-mv (send *ri* :moment-vector :rarm))
       (larm-mv (send *ri* :moment-vector :larm))
       )

    (setq sum-of-fv-z (+ (elt rleg-fv 2) (elt lleg-fv 2)));;when using arms, add rarm-fv and larm-fv
    (setq diff-mass (- sum-of-fv-z jaxon-mass))
    
    (print 'lleg-force-vector)
    (print lleg-fv)
    (print 'rleg-force-vector)
    (print rleg-fv)
    (print 'lleg-moment-vector)
    (print lleg-mv)
    (print 'rleg-moment-vector)
    (print rleg-mv)
    (print '(sum of force-vector-z is))
    (print sum-of-fv-z)
    (print '(force-vector(z) - mass is))
    (print diff-mass)
    ))


(defun initial-pose (&key (box-height 850) (step-width 700) (hand-dis 400) (com-move 100) (arm-push 0))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (let* (
	 (centroid-coords
	  (send *robot* :centroid :copy-worldcoords))
	 (rleg-pos
	  (spend *robot* :rleg :end-coords :worldpos))
	 (lleg-pos
	  (send *robot* :lleg :end-coords :worldpos))
	 (target-coords
	  (list
	   (make-coords :pos (float-vector (+ hand-dis (elt centroid-coords 0)) (* -0.5 step-width) (- box-height arm-push)) :rpy (float-vector 0 (deg2rad 90) 0));rarm
	   (make-coords :pos (float-vector (+ hand-dis (elt centroid-coords 0)) (* 0.5 step-width) (- box-height arm-push)) :rpy (float-vector 0 (deg2rad 90) 0));larm
	   (make-coords :pos (float-vector (elt centroid-coords 0) (elt rleg-pos 1) (elt rleg-pos 2)) :rpy (float-vector 0 0 0))
	   (make-coords :pos (float-vector (elt centroid-coords 0) (elt lleg-pos 1) (elt lleg-pos 2)) :rpy (float-vector 0 0 0))
	  ))
	 (limbs
	  (list
	   (send *robot* :rarm :end-coords)
	   (send *robot* :larm :end-coords)
	   (send *robot* :rleg :end-coords)
	   (send *robot* :lleg :end-coords)
	   )))
    (send *robot*
	  :fullbody-inverse-kinematics target-coords
	  :move-target limbs
	  :link-list (mapcar #' (lambda (limb) (send *robot* :link-list (send limb :parent))) limbs)
	  :target-centroid-pos (float-vector (+ com-move (elt centroid-coords 0)) (elt centroid-coords 1) (elt centroid-coords 2))
	  )

    (when (y-or-n-p "Move to initial pose?")
      (send *ri* :angle-vector (send *robot* :angle-vector))
      (send *ri* :wait-interpolation))
    ))

(defun stop-auto-control ()
  (send *ri* :stop-st)
  (send *ri* :stop-impedance :arms)
  (send *ri* :stop-auto-balancer))


(defun jikken-shimon ()
  (warning-message 4 "log clear~%")
  (send *ri* :start-log)
  (send *robot* :reset-pose)
  (when (y-or-n-p "move to reset-pose?")
    (send *ri* :angle-vector (send *robot* :angle-vector))
    )
  
  (let ((i 0))
    (setq sum-diff-mass 0)
    (while (< i 10)
      (check-z)
      (setq sum-diff-mass (+ diff-mass sum-diff-mass))
      (setq i (+ i 1))
      )
    (print '(average of diff-mass is))
    (print (/ sum-diff-mass 10.0))
    )

  (when (y-or-n-p "pushing robot?")
    (let ((i 0))
      (while (< i 10)
        (send *ri* :state)
          (let*
              ((rleg-fv (send *ri* :force-vector :rleg))
               (lleg-fv (send *ri* :force-vector :lleg))
               (rarm-fv (send *ri* :force-vector :rarm))
               (larm-fv (send *ri* :force-vector :larm))
               (rleg-mv (send *ri* :moment-vector :rleg))
               (lleg-mv (send *ri* :moment-vector :lleg))
               (rarm-mv (send *ri* :moment-vector :rarm))
               (larm-mv (send *ri* :moment-vector :larm))
               )

            (print 'lleg-force-vector)
            (print lleg-fv)
            (print 'rleg-force-vector)
            (print rleg-fv)
            (print 'lleg-moment-vector)
            (print lleg-mv)
            (print 'rleg-moment-vector)
            (print rleg-mv)
            )
          (setq i (+ i 1))
          )
      )
    )

  (initial-pose)
  (when (y-or-n-p "put bar, box, or something around chest? ready to stop abc, st, and ic(arms)?")
    (stop-auto-control)
    )

  (let ((i 0))
    (setq sum-diff-mass 0)
    (while (< i 10)
      (check-z)
      (setq sum-diff-mass (+ diff-mass sum-diff-mass))
      (setq i (+ i 1))
      )
    (print '(average of diff-mass is))
    (print (/ sum-diff-mass 10.0))
    )
  (when (y-or-n-p "Save log?")
    (save-log :fname "jikken for chukan-shimon")
    )

  )
    

