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


(defun calc-zmp-from-force-sensor (&key (d ~~~~~))
  ;;(send *ri* :force-vector :larm) (send *ri* :moment-vector :larm) to get value from 6 axis sensor
  ;;leave about the arms' sensor, make this by legs' sensor only for chukan's experiment
  (let*
      ((rleg-fv (send *ri* :force-vector :rleg))
       (lleg-fv (send *ri* :force-vector :lleg))
       (rarm-fv (send *ri* :force-vector :rarm))
       (larm-fv (send *ri* :force-vector :larm))
       (rleg-mv (send *ri* :moment-vector :rleg))
       (lleg-mv (send *ri* :moment-vector :lleg))
       (rarm-mv (send *ri* :moment-vector :rarm))
       (larm-mv (send *ri* :moment-vector :larm))

       (p-rleg-x (/ (- (- (cadr rleg-mv)) (* (car rleg-fv) d)) (* (caddr rleg-fv) 1.0)))
       (p-rleg-y (/ (- (car rleg-mv) (* (cadr rleg-fv) d)) (* (caddr rleg-fv) 1.0)))
       (p-lleg-x (/ (- (- (cadr lleg-mv)) (* (car lleg-fv) d)) (* (caddr lleg-fv) 1.0)))
       (p-lleg-y (/ (- (car lleg-mv) (* (cadr lleg-fv) d)) (* (caddr lleg-fv) 1.0)))

       (p-x (/ (+ (* p-rleg-x (caddr rleg-fv)) (* p-lleg-x (caddr lleg-fv))) (* 1.0 (+ (caddr rleg-fv) (caddr lleg-fv)))))
       (p-y (/ (+ (* p-rleg-y (caddr rleg-fv)) (* p-lleg-y (caddr lleg-fv))) (* 1.0 (+ (caddr rleg-fv) (caddr lleg-fv)))))
       )

    (setq zmp-from-force-sensor (list p-x p-y))
  )


