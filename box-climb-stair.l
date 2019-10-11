;; How to use

;; 1. execute roseus
;; 2. (load "box-climb.l")
;; 3. (jaxoninit) 
;; 4. make *ri* -> y
;; send *ri* :set-gait-generator-param :toe-angle 20  ;;wrote in function so no need to do this in your terminal by hand
;; send *ri* :set-gait-generator-param :heel-angle 10 ;;same as above
;; 5. (walk-up)



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
  ;; (when (y-or-n-p "make *ri*?") (jaxon_red-init))
  )

(defun initial-set-up (&key (start-pos 0.1))
;  (send *ri* :go-pos start-pos 0 0)
;  (send *ri* :wait-interpolation)
  (warning-message 3 "stop st~%")
  (send *ri* :stop-st)
  (warning-message 3 "(reset-pose)~%")
  (reset-pose)
  (warning-message 3 "stop abc~%")
  (send *ri* :stop-auto-balancer)
  (warning-message 3 "stop ic~%")
  (send *ri* :stop-impedance :arms)
  (warning-message 3 "start st~%")
  (send *ri* :start-st)
  (send *ri* :set-st-param :emergency-check-mode 0)
  )




(defun walk-up (&key (box-height 200) (stride 200) (bend-knee-height 100))
  (send *ri* :go-pos 0.1 0 0)
  (send *ri* :wait-interpolation)
  (send *robot* :reset-pose)
  (send *robot* :legs :move-end-pos (float-vector 0 0 bend-knee-height)) ;;to curve the knees
  (send *ri* :angle-vector (send *robot* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :set-gait-generator-param
        :default-orbit-type :stair
        :toe-angle 20
        :heel-angle 10
        :default-step-time 2.5);; default-orbit-type :stair -> foot-moving-orbit is cycroid(default), which causes hitting to the stair, so it changes orbit to up and move forward
  (send *ri* :set-foot-steps
        (list (make-coords :coords (send *robot* :lleg :end-coords :copy-worldcoords) :name :lleg)
              (make-coords :coords (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector stride 0 box-height)) :rotate (deg2rad 0) :y) :name :rleg)
              (make-coords :coords (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector stride 0 box-height)) :rotate (deg2rad 0) :y) :name :lleg)
              ))
  )


(defun generate-walk-up-motion (&key (box-height 200) (stride 250) (step-time 10) (step-height 210) (dt 0.02) (debug-view :no-message) (bend-knee-height 50) (torso-angle 30))
  (let* ((fs-list)
;         (rarm-end-coords (send *robot* :rarm :end-coords :copy-worldcoords))
;         (larm-end-coords (send *robot* :larm :end-coords :copy-worldcoords))
         (rleg-end-coords (send *robot* :rleg :end-coords :copy-worldcoords))
         (lleg-end-coords (send *robot* :lleg :end-coords :copy-worldcoords))
         )
    (setq fs-list (list (send (make-coords :coords (send rleg-end-coords :copy-worldcoords) :name :rleg) :translate (float-vector 0 0 0) :world)
   ;                     (send (make-coords :coords (send lleg-end-coords :copy-worldcoords) :name :lleg) :translate (float-vector 0 0 0) :world)
 ;                       (send (make-coords :coords (send rarm-end-coords :copy-worldcoords) :name :rarm) :translate (float-vector 0 0 0) :world)
  ;                      (send (make-coords :coords (send larm-end-coords :copy-worldcoords) :name :larm) :translate (float-vector 0 0 0) :world)
;                        (send (make-coords :coords (send rleg-end-coords :copy-worldcoords) :name :rleg) :translate (float-vector stride 0 box-height) :world)
                        (send (make-coords :coords (send lleg-end-coords :copy-worldcoords) :name :lleg) :translate (float-vector stride 0 box-height) :world)
                        (send (make-coords :coords (send rleg-end-coords :copy-worldcoords) :name :rleg) :translate (float-vector stride 0 box-height) :world)
                        ))
    (send *robot* :reset-pose)
    (send *robot* :legs :move-end-pos (float-vector 0 0 bend-knee-height)) ;;to curve the knees
    (send *robot* :torso-waist-p :joint-angle torso-angle)
    ;; (send *ri* :angle-vector (send *robot* :angle-vector))
    ;; (send *ri* :wait-interpolation)
    (send *robot* :move-centroid-on-foot :both '(:lleg :rleg))
    (send *robot* :fix-leg-to-coords (make-coords))
    (warning-message 3 "generating walking pattern ...~%")
    (setq *walk-pattern*
	  (send *robot* :calc-walk-pattern-from-footstep-list fs-list
;;		:default-zmp-offsets (list (list :rleg #f(0 0 0)) (list :lleg #f(0 0 0)))
		:all-limbs '(:rleg :lleg) ;rarm,larm
		:init-pose-function #'(lambda ())
		:debug-view debug-view
		:default-step-time step-time
        :default-step-height step-height
        :dt dt
        ;:solve-angle-vector-args
        ;(list :root-link-virtual-joint-weight #f(0.1 0.1 0.1 0.01 0.5 0.5))
		))

    (setq av-list nil
          zmp-list nil
          root-coord-list nil
          contact-state-list nil)
    (setq tm-list nil)
    (setq additional-av (if (is-choreonoid) #f(0 0 0 0) #f()))
    (dolist (a *walk-pattern*)
      ;; a is like (:angle-vector #f(0 ...) :middle middle :time 0 :other other)
      ;; then, (memq :time a) extracts (:time 0 :other other)
      ;; (push (concatenate float-vector (cadr (memq :angle-vector a)) #f(-80 80 -80 80)) av-list)
      (push (concatenate float-vector (cadr (memq :angle-vector a)) additional-av) av-list)
      (push (cadr (memq :zmp a)) zmp-list)
      (push (cadr (memq :root-coords a)) root-coord-list)
      (push
       (concatenate float-vector (mapcar #'(lambda (cs) (if (equal :support cs) 1 0)) (cadr (memq :contact-state a))) '(0 0 1 1 1 1)) ;; need to support arm contact
       ;; contact states: cs, remain phase time: pt
       ;; '(rleg-cs lleg-cs rarm-cs larm-cs rleg-pt lleg-pt rarm-pt larm-pt)
       contact-state-list)
      ;; (push (* 1000 (cadr (memq :time a))) tm-list) ;; convert from sec to msec
      (push (* 1000 dt) tm-list) ;; convert from sec to msec
      )
    (nreverse av-list)
    (nreverse zmp-list)
    (nreverse root-coord-list)
    (nreverse contact-state-list)
    (nreverse tm-list)

    )
  )

(defun execute-motion (&key (real t) (bend-knee-height 50))
  (warning-message 3 "start st~%")
  (send *ri* :start-st)
  (unix:sleep 1)

  (send *ri* :set-log-maxlength (* 500 45))
  (send *ri* :start-log)

  (when real
    (warn "real machine move~%")
    ;;   (when (y-or-n-p "walk-up?~%")

    ;; set initial pose
    (let ((tm 10000))
      ;; (send *robot* :reset-pose)
      ;; (send *robot* :legs :move-end-pos (float-vector 0 0 bend-knee-height)) ;;to curve the knees
      ;; (send *robot* :fix-leg-to-coords (make-coords))
      ;; ;; (reset-pose :tm tm)
      ;; (set-current-pose :tm tm)
      (warning-message 3 "move to initial pose~%")
      (send *ri* :angle-vector-sequence-full (list (car av-list)) (list tm)
            :root-coords (list (car root-coord-list))
            :zmp (list (car zmp-list)) :optional (list (car contact-state-list)))
      (send *ri* :wait-interpolation-seq)
      ;; (unix:usleep (* 1000 tm))
      )

    (warning-message 3 "interpoltaion-mode linear~%")
    (send *ri* :set-interpolation-mode :linear)
    (warning-message 3 "move~%")
    (send *ri* :angle-vector-sequence-full av-list tm-list :root-coords root-coord-list :zmp zmp-list :optional contact-state-list)
    (send *ri* :wait-interpolation-seq)
    (warning-message 3 "interpoltaion-mode hoffarbib~%")
    (send *ri* :set-interpolation-mode :hoffarbib)
    (save-log :fname "walk-up")
    )
  )


(defun walk-up-demo (&key (start-pos 0) (real t) (box-height 200) (stride 250) (step-time 5) (step-height 210) (dt 0.02) (debug-view :no-message) (bend-knee-height 50) (torso-angle 30))
  ;;use this after jaxoninit
  ;;start-pos should be 0.1 when simulate at Choreonoid TASK:=CLIMB_BOX_40
  ;;do go-pos 0.1 0 0 when choreonoid
  ;;when real machine, put 20cm-box 5cm away from jaxonred's foot
  (initial-set-up :start-pos start-pos)
  (generate-walk-up-motion :box-height box-height :stride stride :step-time step-time :step-height step-height :dt dt :debug-view debug-view :bend-knee-height bend-knee-height :torso-angle torso-angle)
  (when (y-or-n-p "execute-motion Ready?~%")
    (execute-motion :real real)
    )
  )

(defun initial-pose (&key (com-offset 0) (box-height 300) (arm-xpos-target 200));;(&key (leg-xpos-target 800) (leg-zpos-target 50))
  (let* (
	 (centroid-coords
	  (send *robot* :centroid))
	 (rleg-coords
	  (send *robot* :rleg :end-coords :copy-worldcoords)) ;;:worldpos
	 (lleg-coords
	  (send *robot* :lleg :end-coords :copy-worldcoords))
	 (rarm-pos
	  (send *robot* :rarm :end-coords :worldpos))
	 (larm-pos
	  (send *robot* :larm :end-coords :worldpos))
     (sum-pos
      (v+ (v+ (v+ rarm-pos larm-pos) (send rleg-coords :pos)) (send lleg-coords :pos)))
     (target-centroid-pos
      (float-vector (* 0.25 (elt sum-pos 0)) (* 0.25 (elt sum-pos 1)) (* 0.5 (elt sum-pos 2))))
	 (target-coords
	  (list
       (make-coords :pos (float-vector (+ arm-xpos-target (elt rarm-pos 0)) (elt rarm-pos 1) box-height) :rpy (float-vector 0 (* 0.5 pi) 0))
       (make-coords :pos (float-vector (+ arm-xpos-target (elt larm-pos 0)) (elt larm-pos 1) box-height) :rpy (float-vector 0 (* 0.5 pi) 0))
	   rleg-coords
	   lleg-coords
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
	  ;; :target-centroid-pos (float-vector (+ -40 (elt centroid-coords 0)) (elt centroid-coords 1) (elt centroid-coords 2))
	  ;; :target-centroid-pos (v+ (float-vector com-offset 0 0) target-centroid-pos)
	  )
    (setq ri-vector (concatenate float-vector (send *robot* :angle-vector) #f(-80 80 -80 80)))))




(defun generate-four-limbs-walk-up (&key (step-time 10) (real t) (dt 0.002) (step-height 50) (debug-view :no-message) (stride 200) (box-height 300))
  (let* ((fs-list)
         (rarm-end-coords (send *robot* :rarm :end-coords :copy-worldcoords))
         (larm-end-coords (send *robot* :larm :end-coords :copy-worldcoords))
         (rleg-end-coords (send *robot* :rleg :end-coords :copy-worldcoords))
         (lleg-end-coords (send *robot* :lleg :end-coords :copy-worldcoords))
         )
    (setq fs-list (list (send (make-coords :coords (send rleg-end-coords :copy-worldcoords) :name :rleg) :translate (float-vector 0 0 0) :world)
                        (send (make-coords :coords (send lleg-end-coords :copy-worldcoords) :name :lleg) :translate (float-vector 0 0 0) :world)
                        (send (make-coords :coords (send rarm-end-coords :copy-worldcoords) :name :rarm) :translate (float-vector 0 0 0) :world)
                        (send (make-coords :coords (send larm-end-coords :copy-worldcoords) :name :larm) :translate (float-vector 0 0 0) :world)
                        (send (make-coords :coords (send rleg-end-coords :copy-worldcoords) :name :rleg) :translate (float-vector stride 0 box-height) :world)
                        (send (make-coords :coords (send lleg-end-coords :copy-worldcoords) :name :lleg) :translate (float-vector stride 0 box-height) :world)
                        ))
    (warning-message 3 "generating walking pattern ...~%")
    (setq walk-pattern
	  (send *robot* :calc-walk-pattern-from-footstep-list fs-list
		:all-limbs '(:rarm :larm :rleg :lleg)
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
      (push (concatenate float-vector (cadr (memq :angle-vector a)) #f(-80 80 -80 80)) av-list)
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
