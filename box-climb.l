;; How to use

;; 1. execute roseus
;; 2. (load "box-climb.l")
;; 3. (jaxoninit) 
;; 4. make *ri* -> y
;; 5. (box-climb)
;; 6. move to initial pose? -> y y y y y y y~~~~~~~~~




;;Comment

;;start choreonoid -> rtmlaunch hrpsys_choreonoid_tutorials jaxon_red_choreonoid.launch TASK:=BOX_CLIMB
;;modify environment -> change ~/catkin_ws/jaxon_tutorial/rtm_ros_robotics/rtmros_choreonoid/jvrc_models/models/climb_box.wrl
;;                          and ~/catkin_ws/jaxon_tutorial/rtm_ros_robotics/rtmros_choreonoid/hrpsys_choreonoid_tutorials/config/JAXON_RED_BOX_CLIMB.cnoid

;; to change size, location, or mass of the box, change climb_box.wrl
;;when change size(z), move z/2 of location(z) to attach the box to the floor
;;initial-pose (key box-height==real-box-height) -150mm in climb_box.wrl's Trasnform -> translate(z)



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

(defun hand-balance (&key (box-height 850) (step-width 700) (hand-dis 400) (com-move 100) (arm-push 0))

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
;;	   (make-coords :pos (float-vector (elt centroid-coords 0) (elt lleg-pos 1) (elt lleg-pos 2)) :rpy (float-vector 0 0 0))
	  ))
	 (limbs
	  (list
	   (send *robot* :rarm :end-coords)
	   (send *robot* :larm :end-coords)
	   (send *robot* :rleg :end-coords)
;;	   (send *robot* :lleg :end-coords)
	   )))
    (send *robot*
	  :fullbody-inverse-kinematics target-coords
	  :move-target limbs
	  :link-list (mapcar #' (lambda (limb) (send *robot* :link-list (send limb :parent))) limbs)
	  :target-centroid-pos (float-vector (+ com-move (elt centroid-coords 0)) (elt centroid-coords 1) (elt centroid-coords 2))
	  )

    (when (y-or-n-p "Move to hand balance pose?")
      (send *ri* :angle-vector (send *robot* :angle-vector))
      (send *ri* :wait-interpolation))
    ))


;;6/14kokomade

(defun stop-auto-control ()
  (send *ri* :stop-st)
  (send *ri* :stop-impedance :arms)
  (send *ri* :stop-auto-balancer))

(defun turiage-pose (&key (box-height 850) (step-width 700) (hand-dis 400) (com-move 100) (arm-push 0) (torso-x 0) (torso-z 0) (torso-angle 50) (leg-x 0) (leg-z 0))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (let* (
	 (centroid-coords
	  (send *robot* :centroid :copy-worldcoords))
	 (rleg-pos
	  (send *robot* :rleg :end-coords :worldpos))
	 (lleg-pos
	  (send *robot* :lleg :end-coords :worldpos))
     (torso-pos
      (send (send *robot* :torso-end-coords) :worldpos))
	 (target-coords
	  (list
	   (make-coords :pos (float-vector (+ hand-dis (elt centroid-coords 0)) (* -0.5 step-width) (- box-height arm-push)) :rpy (float-vector 0 (deg2rad 90) 0));rarm
	   (make-coords :pos (float-vector (+ hand-dis (elt centroid-coords 0)) (* 0.5 step-width) (- box-height arm-push)) :rpy (float-vector 0 (deg2rad 90) 0));larm
	   (send (make-coords :pos rleg-pos :rpy (float-vector 0 0 0)) :translate (float-vector leg-x 0 leg-z))
	   (send (make-coords :pos lleg-pos :rpy (float-vector 0 0 0)) :translate (float-vector leg-x 0 leg-z))
       (send (make-coords :pos torso-pos :rpy (float-vector 0 (deg2rad torso-angle) 0)) :translate (float-vector torso-x 0 torso-z) :world)
       ))
	 (limbs
	  (list
	   (send *robot* :rarm :end-coords)
	   (send *robot* :larm :end-coords)
	   (send *robot* :rleg :end-coords)
	   (send *robot* :lleg :end-coords)
       (send *robot* :torso-end-coords)
	   )))
    (send *robot*
	  :fullbody-inverse-kinematics target-coords
	  :move-target limbs
	  :link-list (mapcar #' (lambda (limb) (send *robot* :link-list (send limb :parent))) limbs)
	  :target-centroid-pos (float-vector (+ com-move (elt centroid-coords 0)) (elt centroid-coords 1) (elt centroid-coords 2))
	  )

    (when (y-or-n-p "Move to turiage pose?")
      (send *ri* :angle-vector (send *robot* :angle-vector))
      (send *ri* :wait-interpolation))
    ))

(defun paratune (&key (shoulder 0) (elbow 0) (wrist 0))
  (send *robot* :arms :shoulder-p :joint-angle shoulder)
  (send *robot* :arms :elbow-p :joint-angle elbow)
  (send *robot* :arms :wrist-p :joint-angle wrist)
  (when (y-or-n-p "Ready to move?")
    (send *ri* :angle-vector (send *robot* :angle-vector))
    (send *ri* :wait-interpolation))
  )


(defun right-foot-step (&key (com-move 390) (box-height 850) (step-width 700) (hand-dis 400) (arm-push 390) (foot-step-height 750))
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
	   (make-coords :pos (float-vector (+ hand-dis (elt centroid-coords 0)) (* -0.8 step-width) (- box-height arm-push)) :rpy (float-vector 0 0 0))
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
;;	  :target-centroid-pos (float-vector (+ com-move (elt centroid-coords 0)) (elt centroid-coords 1) (elt centroid-coords 2))
	  )

    (when (y-or-n-p "ready to put right foot up?")
      (send *ri* :angle-vector (send *robot* :angle-vector))
      (send *ri* :wait-interpolation))
    ))

(defun foot-on-step (&key (com-move 390) (box-height 850) (step-width 700) (hand-dis 400) (arm-push 390))
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
	   (make-coords :pos (float-vector (+ hand-dis (elt centroid-coords 0)) (* -0.8 step-width) (- box-height arm-push)) :rpy (float-vector 0 0 0))
	   (make-coords :pos (float-vector (+ hand-dis (elt centroid-coords 0)) (* 0.8 step-width) (- box-height arm-push)) :rpy (float-vector 0 0 0))
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
;;	  :target-centroid-pos (float-vector (+ com-move (elt centroid-coords 0)) (elt centroid-coords 1) (elt centroid-coords 2))
	  )

    (when (y-or-n-p "ready to put both foot up?")
      (send *ri* :angle-vector (send *robot* :angle-vector))
      (send *ri* :wait-interpolation))
    ))

(defun generate-right-foot-step (&key (step-time 3) (real t) (dt 0.002) (step-height 50) (debug-view :no-message) (com-move 390) (box-height 850) (step-width 700) (hand-dis 400) (arm-push 390))
  (let* ((fs-list)
         (centroid-coords (send *robot* :centroid :copy-worldcoords))
         (rarm-end-coords (send *robot* :rarm :end-coords :copy-worldcoords))
         (larm-end-coords (send *robot* :larm :end-coords :copy-worldcoords))
         (rleg-end-coords (send *robot* :rleg :end-coords :copy-worldcoords))
         )
    (setq fs-list (list (send (make-coords :coords rleg-end-coords :name :rleg) :translate (float-vector 0 0 0) :world)
                        (send (make-coords :pos (float-vector (+ hand-dis (elt centroid-coords 0)) (* -0.8 step-width) (- box-height arm-push)) :rpy (float-vector 0 0 0) :name :rleg) :translate (float-vector 0 0 0) :world)
                        ))

    (warning-message 3 "generating walking pattern ...~%")    
    (setq walk-pattern
	  (send *robot* :calc-walk-pattern-from-footstep-list fs-list
		:all-limbs '(:rarm :larm :rleg)
		:init-pose-function #'(lambda ())
		:debug-view debug-view
		:default-step-time step-time
        :default-step-height step-height
        :dt dt
;;        :solve-angle-vector-args (list :collision-avoidance-link-pair (list (list (send *robot* :rarm :wrist-p :child-link) (send *robot* :rleg_joint2 :child-link )) (list (send *robot* :larm :wrist-p :child-link) (send *robot* :lleg_joint2 :child-link)))
  ;;                                     :avoid-collision-null-gain 10
;;                                   :avoid-collision-joint-gain 0
;;                                       :avoid-collision-distance 200 
;;                                       :stop 200)
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


(defun fk-right-step-outside ()
  (send *robot* :rleg :crotch-p :joint-angle 20) 
  (send *robot* :rleg :crotch-y :joint-angle -58)
  (send *robot* :rleg :knee-p :joint-angle 70) 
  (send *robot* :arms :elbow-p :joint-angle -45)
  (send *ri* :angle-vector (send *robot* :angle-vector))
  (send *ri* :wait-interpolation)
  
  (send *robot* :torso :waist-y :joint-angle 35)
  (send *robot* :rleg :crotch-p :joint-angle -90)
  (send *robot* :rleg :knee-p :joint-angle 90)
  (send *robot* :lleg :crotch-p :joint-angle 0)
  (send *robot* :lleg :knee-p :joint-angle 0)
  (send *ri* :angle-vector (send *robot* :angle-vector))
  (send *ri* :wait-interpolation)


  (send *robot* :rleg :knee-p :joint-angle 90)
  (send *robot* :rleg :crotch-r :joint-angle 30)
  (send *robot* :rleg :crotch-y :joint-angle -58)
  (send *ri* :angle-vector (send *robot* :angle-vector))
  (send *ri* :wait-interpolation)
  
  (send *robot* :rleg :ankle-p :joint-angle 10)
  (send *robot* :rleg :crotch-y :joint-angle -50)
  (send *ri* :angle-vector (send *robot* :angle-vector))
  (send *ri* :wait-interpolation)
  )

  
 
(defun box-climb (&key (foot-on-com-move 390) (box-height 850) (step-width 700) (hand-dis 400) (foot-on-arm-push 390))

  (warning-message 4 "log clear~%")
  (send *ri* :start-log)
  
  (initial-pose :box-height box-height :step-width step-width :hand-dis hand-dis)
;;  (send *ri* :set-st-param :is-ik-enable (list t t t t))
;;  (send *ri* :stop-impedance :arms)
  (stop-auto-control)
  (initial-pose :com-move 350 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 380 :arm-push 20 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 400 :arm-push 30 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 420 :arm-push 100 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 420 :arm-push 150 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 400 :arm-push 170 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 400 :arm-push 200 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 230 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 250 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 270 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 290 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 310 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 330 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 350 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 370 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 390 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (fk-right-step-outside)
  (when (y-or-n-p "save log?")
    (save-log :fname "box-climb")
    )
  )

(defun generate-box-climb (&key (foot-on-com-move 390) (box-height 850) (step-width 700) (hand-dis 400) (foot-on-arm-push 390) (step-height 50) (step-time 3))
  (initial-pose :box-height box-height :step-width step-width :hand-dis hand-dis)
  (stop-auto-control)
  (initial-pose :com-move 350 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 380 :arm-push 20 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 400 :arm-push 30 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 420 :arm-push 100 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 420 :arm-push 150 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 400 :arm-push 170 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 400 :arm-push 200 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 230 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 250 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 270 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 290 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 310 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 330 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 350 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 370 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 390 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (generate-right-foot-step :com-move foot-on-com-move :arm-push foot-on-arm-push :box-height box-height :step-width step-width :hand-dis hand-dis :step-time step-time :step-height step-height)
  )



(defun box-climb-both-foot (&key (foot-on-com-move 390) (box-height 850) (step-width 700) (hand-dis 400) (foot-on-arm-push 390))
  (initial-pose :box-height box-height :step-width step-width :hand-dis hand-dis)
  (stop-auto-control)
  (initial-pose :com-move 350 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 380 :arm-push 20 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 400 :arm-push 30 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 420 :arm-push 100 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 420 :arm-push 150 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 400 :arm-push 170 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 400 :arm-push 200 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 230 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 250 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 270 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 290 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 310 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 330 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 350 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 370 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 390 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (foot-on-step :com-move foot-on-com-move :arm-push foot-on-arm-push :box-height box-height :step-width step-width :hand-dis hand-dis)
  )


(defun box-climb-knee (&key (foot-on-com-move 390) (box-height 850) (step-width 700) (hand-dis 400) (foot-on-arm-push 390))
  (initial-pose :box-height box-height :step-width step-width :hand-dis hand-dis)
  (stop-auto-control)
  (initial-pose :com-move 350 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 380 :arm-push 20 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 400 :arm-push 30 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 420 :arm-push 100 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 420 :arm-push 150 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 400 :arm-push 170 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 400 :arm-push 200 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 230 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 250 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 270 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 290 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 310 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 330 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 350 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 370 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (initial-pose :com-move 390 :arm-push 390 :box-height box-height :step-width step-width :hand-dis hand-dis)
  (send *robot* :rleg :crotch-p :joint-angle -90)
  (send *robot* :rleg :knee-p :joint-angle 180)
  (send *ri* :angle-vector (send *robot* :angle-vector))
  )


