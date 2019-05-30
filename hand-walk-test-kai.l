;; How to use

;; 1. execute roseus
;; 2. (load "hand-walk-test-kai.l")
;; 3. (jaxoninit)  ;; (jaxon_red-init) do this in real machine
;;(send *ri* :stop-st)
;;(send *ri* :stop-auto-balancer)
;;(send *ri* :stop-impedance :arms)
;; 4. (initial-pose2)
;;(send *ri* :angle-vector (send *robot* :angle-vector))
;; 5. (hand-stand)
;;(send *ri* :angle-vector (send *robot* :angle-vector))
;; 6. (generate-hand-walk-pattern)
;;
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

(defun initial-pose ()
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :legs :move-end-pos (float-vector 0 0 100) :world);500->100
  (let* ((rtc (make-coords :pos #f(0 -300 800) :rpy (float-vector 0 (deg2rad 90) 0)))
         (ltc (make-coords :pos #f(0 300 800) :rpy (float-vector 0 (deg2rad 90) 0)))
         );700->800 *2 ;;800が棒の高さ
    (send *robot* :rarm :inverse-kinematics rtc)
    (send *robot* :larm :inverse-kinematics ltc)
    )
  (send *robot* :move-centroid-on-foot :both '(:rarm :larm) :target-centroid-pos (midpoint 0.5 (send *robot* :rarm :end-coords :worldpos) (send *robot* :larm :end-coords :worldpos)))
  (send *robot* :translate (v- (float-vector (elt (send *robot* :rarm :end-coords :worldpos) 0) 0 (elt (send *robot* :rarm :end-coords :worldpos) 2))) :world)
  (send *robot* :translate (float-vector 0 0 800) :world)

  (send *irtviewer* :draw-floor t)
  (setq *pole1* (make-cylinder 20 1000))
  (setq *pole2* (make-cylinder 20 1000))
  (send *pole1* :newcoords (send *robot* :rarm :end-coords :copy-worldcoords))
  (send *pole2* :newcoords (send *robot* :larm :end-coords :copy-worldcoords))
  (send *pole1* :translate (float-vector 0 0 -300))
  (send *pole2* :translate (float-vector 0 0 -300))
  (objects (list *robot* *pole1* *pole2*))
  )

(defun initial-pose2 (&key (step-width 700))
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
	   (make-coords :pos (float-vector (elt centroid-coords 0) (* -0.5 step-width) 650) :rpy (float-vector 0 (deg2rad 90) 0));rarm
	   (make-coords :pos (float-vector (elt centroid-coords 0) (* 0.5 step-width) 650) :rpy (float-vector 0 (deg2rad 90) 0));larm
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
	  :target-centroid-pos (float-vector (elt centroid-coords 0) (elt centroid-coords 1) (elt centroid-coords 2))
	  )

    (when (y-or-n-p "Move to initial pose?")
      (send *ri* :angle-vector (send *robot* :angle-vector))
      (send *ri* :wait-interpolation))
    ))


(defun hand-stand (&key (com-offset -10))
  (let* (
	 (centroid-coords (send *robot* :centroid))
	 (rleg-pos (send *robot* :rleg :end-coords :worldpos))
	 (lleg-pos (send *robot* :lleg :end-coords :worldpos))
	 (larm-coords (send *robot* :larm :end-coords :copy-worldcoords))
	 (rarm-coords (send *robot* :rarm :end-coords :copy-worldcoords))
	 (target-coords
	  (list
	   ;; (make-coords :pos (float-vector (elt centroid-coords 0) -350 700) :rpy (float-vector 0 (deg2rad 90) 0));rarm
	   ;; (make-coords :pos (float-vector (elt centroid-coords 0) 350 700) :rpy (float-vector 0 (deg2rad 90) 0));larm
	   (make-coords :pos (concatenate float-vector (subseq centroid-coords 0 1) (subseq (send rarm-coords :worldpos) 1 3)) :rpy (car (send rarm-coords :rpy-angle)))
	   (make-coords :pos (concatenate float-vector (subseq centroid-coords 0 1) (subseq (send larm-coords :worldpos) 1 3)) :rpy (car (send larm-coords :rpy-angle)))
	   (make-coords :pos (float-vector (elt centroid-coords 0) (elt rleg-pos 1) (+ 100 (elt rleg-pos 2))) :rpy (float-vector 0 0 0))
	   (make-coords :pos (float-vector (elt centroid-coords 0) (elt lleg-pos 1) (+ 100 (elt lleg-pos 2))) :rpy (float-vector 0 0 0))
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
	  :target-centroid-pos (v+ (float-vector com-offset 0 0) (send (midcoords 0.5 larm-coords rarm-coords) :worldpos))
	  )

    (when (y-or-n-p "Ready to stand by hand?")
      (send *ri* :angle-vector (send *robot* :angle-vector))
      (send *ri* :wait-interpolation))
    ))

(defun kyakuzenkyo-pose (&key (hip-angle -90) (com-offset -40));;(&key (leg-xpos-target 800) (leg-zpos-target 50))
  (let* (
	 (centroid-coords
	  (send *robot* :centroid))
	 (rleg-pos
	  (send *robot* :rleg :end-coords :worldpos))
	 (lleg-pos
	  (send *robot* :lleg :end-coords :worldpos))
	 (rarm-pos
	  (send *robot* :rarm :end-coords :copy-worldcoords))
	 (larm-pos
	  (send *robot* :larm :end-coords :copy-worldcoords))
	 (target-coords
	  (list
	   rarm-pos
	   larm-pos
	  ;; (make-coords :pos (float-vector (+ leg-xpos-target (elt rleg-pos 0)) (elt rleg-pos 1) (+ leg-zpos-target (elt rarm-pos 2))) :rpy (float-vector 0 0 0))
	  ;; (make-coords :pos (float-vector (+ leg-xpos-target (elt lleg-pos 0)) (elt lleg-pos 1) (+ leg-zpos-target (elt larm-pos 2))) :rpy (float-vector 0 0 0))
	  ))
	 (limbs
	  (list
	   (send *robot* :rarm :end-coords)
	   (send *robot* :larm :end-coords)
	   ;;(send *robot* :rleg :end-coords)
	  ;; (send *robot* :lleg :end-coords)
	   )))
    (send *robot* :lleg :crotch-y :joint-angle 0)
    (send *robot* :lleg :crotch-r :joint-angle 0)
    (send *robot* :lleg :crotch-p :joint-angle hip-angle)
    (send *robot* :lleg :knee-p   :joint-angle 0)
    (send *robot* :lleg :ankle-p  :joint-angle 84)
    (send *robot* :rleg :crotch-y :joint-angle 0)
    (send *robot* :rleg :crotch-r :joint-angle 0)
    (send *robot* :rleg :crotch-p :joint-angle hip-angle)
    (send *robot* :rleg :knee-p   :joint-angle 0)
    (send *robot* :rleg :ankle-p  :joint-angle 84)

    (send *robot*
	  :fullbody-inverse-kinematics target-coords
	  :move-target limbs
	  :link-list (mapcar #' (lambda (limb) (send *robot* :link-list (send limb :parent))) limbs)
	  ;; :target-centroid-pos (float-vector (+ -40 (elt centroid-coords 0)) (elt centroid-coords 1) (elt centroid-coords 2))
	  :target-centroid-pos (v+ (float-vector com-offset 0 0) (send (midcoords 0.5 larm-pos rarm-pos) :worldpos))
	  )))

(defun kyakuzenkyo ()
  (setq av-list nil)
  (setq tm-list nil)

  (when (not (is-simulation));; for real
    (kyakuzenkyo-pose :hip-angle -45 :com-offset -30)
    (push (concatenate float-vector (send *robot* :angle-vector) #f(0 0 0 0)) av-list)
    (push 10000 tm-list)
    )

  (kyakuzenkyo-pose)
  (push (concatenate float-vector (send *robot* :angle-vector) #f(0 0 0 0)) av-list)
  (push 20000 tm-list)

  (nreverse av-list)
  (nreverse tm-list)

  (when (y-or-n-p "Do kyakuzenkyo?")
    (send *ri* :angle-vector-sequence-full av-list tm-list)
    (send *ri* :wait-interpolation))
  )


(defun generate-hand-walk-pattern (&key (step-time 0.5) (stride 50) (real t))
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
                        (send (make-coords :coords (send larm-end-coords :copy-worldcoords) :name :larm) :translate (float-vector (* 9 stride) 0 0) :world)
                        ))
    (setq walk-pattern
	  (send *robot* :calc-walk-pattern-from-footstep-list fs-list
		:all-limbs '(:rarm :larm)
		:init-pose-function #'(lambda ())
		:debug-view :no-message
		:default-step-time step-time
		))

    (setq av-list nil)
    (setq tm-list nil)
    (dolist (a walk-pattern)
      ;; a is like (:angle-vector #f(0 ...) :middle middle :time 0 :other other)
      ;; then, (memq :time a) extracts (:time 0 :other other)
      (push (concatenate float-vector (cadr (memq :angle-vector a)) #f(0 0 0 0)) av-list)
      (push (cadr (memq :time a)) tm-list)
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

(defun play-kyakuzenkyo ()
  (warning-message 4 "log clear~%")
  (send *ri* :start-log)

  (initial-pose2)

  (warning-message 4 "stop st, abc and ic~%")
  (send *ri* :stop-st)
  (send *ri* :stop-auto-balancer)
  (send *ri* :stop-impedance :arms)

  (hand-stand)

  (kyakuzenkyo)
  ;; (send *ri* :save-log "/tmp/kyakuzenkyo")

  (when (y-or-n-p "Save log?")
    (save-log :fname "kyakuzenkyo")
    )
  )

(defun pb-walk-demo ()
  (warning-message 4 "log clear~%")
  (send *ri* :start-log)

  (initial-pose2)

  (warning-message 4 "stop st, abc and ic~%")
  (send *ri* :stop-st)
  (send *ri* :stop-auto-balancer)
  (send *ri* :stop-impedance :arms)

  (hand-stand)

  (generate-hand-walk-pattern)
)
