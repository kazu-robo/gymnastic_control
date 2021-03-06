;; How to use

;; 1. execute roseus
;; 2. (load "box-climb.l")
;; 3. (jaxoninit) 
;; 4. make *ri* -> y




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


;;TODO change genereate~~~ to walk up 40cm box like stairs.  Maybe initial-pose function is not needed. but do not forget to be reset-pose at first.




(defun generate-hand-walk-pattern (&key (step-time 0.5) (stride 50) (real t) (dt 0.002) (step-height 50) (debug-view :no-message))
  (let* ((fs-list)
         (rarm-end-coords (send *robot* :rarm :end-coords :copy-worldcoords))
         (larm-end-coords (send *robot* :larm :end-coords :copy-worldcoords))
         )
    (setq fs-list (list (send (make-coords :coords (send rarm-end-coords :copy-worldcoords) :name :rarm) :translate (float-vector 0 0 0) :world)
                        (send (make-coords :coords (send larm-end-coords :copy-worldcoords) :name :larm) :translate (float-vector stride 0 0) :world)
                        (send (make-coords :coords (send rarm-end-coords :copy-worldcoords) :name :rarm) :translate (float-vector (* 2 stride) 0 0) :world)
                        (send (make-coords :coords (send larm-end-coords :copy-worldcoords) :name :larm) :translate (float-vector (* 3 stride) 0 0) :world)
                        (send (make-coords :coords (send rarm-end-coords :copy-worldcoords) :name :rarm) :translate (float-vector (* 4 stride) 0 0) :world)
                        (send (make-coords :coords (send larm-end-coords :copy-worldcoords) :name :larm) :translate (float-vector (* 5 stride) 0 0) :world)
                        (send (make-coords :coords (send rarm-end-coords :copy-worldcoords) :name :rarm) :translate (float-vector (* 6 stride) 0 0) :world)
                        (send (make-coords :coords (send larm-end-coords :copy-worldcoords) :name :larm) :translate (float-vector (* 7 stride) 0 0) :world)
                        (send (make-coords :coords (send rarm-end-coords :copy-worldcoords) :name :rarm) :translate (float-vector (* 8 stride) 0 0) :world)
                        (send (make-coords :coords (send larm-end-coords :copy-worldcoords) :name :larm) :translate (float-vector (* 8 stride) 0 0) :world)
                        ))
    (warning-message 3 "generating walking pattern ...~%")
    (setq walk-pattern
	  (send *robot* :calc-walk-pattern-from-footstep-list fs-list
		:all-limbs '(:rarm :larm)
		:init-pose-function #'(lambda ())
		:debug-view debug-view
		:default-step-time step-time
        :default-step-height step-height
        :dt dt
		))

    (setq av-list nil)
    (setq tm-list nil)
    (dolist (a walk-pattern)
      ;; a is like (:angle-vector #f(0 ...) :middle middle :time 0 :other other)
      ;; then, (memq :time a) extracts (:time 0 :other other)
      (push (concatenate float-vector (cadr (memq :angle-vector a)) #f(0 0 0 0)) av-list)
      ;; (push (* 1000 (cadr (memq :time a))) tm-list) ;; convert from sec to msec
      (push (* 1000 dt) tm-list) ;; convert from sec to msec
      )
    (nreverse av-list)
    (nreverse tm-list)
    (when real
      (warn "real machine move")
      (when (y-or-n-p "Walk by hand?")
      (send *ri* :angle-vector-sequence-full av-list tm-list)
      (send *ri* :wait-interpolation))
    )
    )
  
  )





(defun initial-pose (&key (box-height 850) (step-width 700) (hand-dis 400) (com-move 100) (arm-push 0))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (let* (
	 (centroid-coords
	  (send *robot* :centroid :copy-worldcoords))
	 (rleg-pos
	  (send *robot* :rleg :end-coords :worldpos))
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

(defun walk-up (&key (box-height 400) (stride 200))
  (send *robot* :reset-pose)
  (send *robot* :legs :move-end-pos #f(0 0 50)) ;;to curve the knees
  (send *ri* :angle-vector (send *robot* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :set-gait-generator-param
        :default-orbit-type :stair
        :default-step-time 2.5);; default-orbit-type :stair -> foot-moving-orbit is cycroid(default), which causes hitting to the stair, so it changes orbit to up and move forward
  (send *ri* :set-foot-steps
        (list (make-coords :coords (send *robot* :lleg :end-coords :copy-worldcoords) :name :lleg)
              (make-coords :coords (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector stride 0 box-height)) :rotate (deg2rad 0) :y) :name :rleg)
              (make-coords :coords (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector stride 0 box-height)) :rotate (deg2rad 0) :y) :name :lleg)
              ))
  )
