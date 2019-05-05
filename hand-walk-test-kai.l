;; How to use

;; 1. execute roseus
;; 2. (load "hand-walk-test-kai.l")
;; 3. (jaxoninit)  ;; (jaxon_red-init) do this in real machine
;;(send *ri* :start-auto-balancer)
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
  (setq *robot* (jaxon_red))
  (send *robot* :fix-leg-to-coords (make-coords))
  (objects (list *robot*))
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

(defun initial-pose2 ()
  (send *robot* :reset-pose)
  (let* (
	 (centroid-coords
	  (send *robot* :centroid :copy-worldcoords))
	 (rleg-pos
	  (send *robot* :rleg :end-coords :worldpos))
	 (lleg-pos
	  (send *robot* :lleg :end-coords :worldpos))
	 (target-coords 
	  (list
	   (make-coords :pos (float-vector (elt centroid-coords 0) -350 700) :rpy (float-vector 0 (deg2rad 90) 0));rarm
	   (make-coords :pos (float-vector (elt centroid-coords 0) 350 700) :rpy (float-vector 0 (deg2rad 90) 0));larm
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
    ))


(defun hand-stand ()
  (let* (
	 (centroid-coords
	  (send *robot* :centroid :copy-worldcoords))
	 (rleg-pos
	  (send *robot* :rleg :end-coords :worldpos))
	 (lleg-pos
	  (send *robot* :lleg :end-coords :worldpos))
	 (target-coords 
	  (list
	   (make-coords :pos (float-vector (elt centroid-coords 0) -350 700) :rpy (float-vector 0 (deg2rad 90) 0));rarm
	   (make-coords :pos (float-vector (elt centroid-coords 0) 350 700) :rpy (float-vector 0 (deg2rad 90) 0));larm
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
	  :target-centroid-pos centroid-coords
	  )))
    
  

(defun generate-hand-walk-pattern ()
  (let* ((fs-list)
         (rarm-end-coords (send *robot* :rarm :end-coords :copy-worldcoords))
         (larm-end-coords (send *robot* :larm :end-coords :copy-worldcoords))
         )
    (setq fs-list (list (send (make-coords :coords (send rarm-end-coords :copy-worldcoords) :name :rarm) :translate (float-vector 0 0 0) :world)
                        (send (make-coords :coords (send larm-end-coords :copy-worldcoords) :name :larm) :translate (float-vector 50 0 0) :world)
                        (send (make-coords :coords (send rarm-end-coords :copy-worldcoords) :name :rarm) :translate (float-vector 100 0 0) :world)
                        (send (make-coords :coords (send larm-end-coords :copy-worldcoords) :name :larm) :translate (float-vector 150 0 0) :world)
                        (send (make-coords :coords (send rarm-end-coords :copy-worldcoords) :name :rarm) :translate (float-vector 200 0 0) :world)
                        (send (make-coords :coords (send larm-end-coords :copy-worldcoords) :name :larm) :translate (float-vector 250 0 0) :world)
                        (send (make-coords :coords (send rarm-end-coords :copy-worldcoords) :name :rarm) :translate (float-vector 300 0 0) :world)
                        (send (make-coords :coords (send larm-end-coords :copy-worldcoords) :name :larm) :translate (float-vector 350 0 0) :world)
                        (send (make-coords :coords (send rarm-end-coords :copy-worldcoords) :name :rarm) :translate (float-vector 400 0 0) :world)
                        (send (make-coords :coords (send larm-end-coords :copy-worldcoords) :name :larm) :translate (float-vector 400 0 0) :world)
                        ))
    (setq walk-pattern
    (send *robot* :calc-walk-pattern-from-footstep-list fs-list
          :all-limbs '(:rarm :larm)
          :init-pose-function #'(lambda ())
          :debug-view :no-message
          :default-step-time 0.5
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
    (send *ri* :angle-vector-sequence-full av-list tm-list)
    )
  )

