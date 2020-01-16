;;How To Use
;;
;;(init)
;;(sitting-init-pose)
;;(change-leg-end-coords-to-knee)
;;(seated-walk3)
;;(calc-contact-wrench *ret*)
;;(view-motion *rs-list*)
;;

(defmethod robot-model
  (:change-end-coords (limb new-cascoords);;end-coordsを変更するメソッド追加
    (cond 
      ((eq limb :rarm)
        (setq rarm-end-coords new-cascoords)
        )
      ((eq limb :larm)
        (setq larm-end-coords new-cascoords)
        )
      ((eq limb :rleg)
        (setq rleg-end-coords new-cascoords)
        )
      ((eq limb :lleg)
        (setq lleg-end-coords new-cascoords)
        )
      )
    )
  )

;;(defmethod gait-generator
;;  (:calc-current-swing-leg-coords
;;    (ratio src dst &key (type :shuffling) (step-height default-step-height))
;;    )
;;  )

(defun load-additional-files ()
  (load "~/prog/euslib/irteus_proposals/motion-lib-proposal.l")
  (load "~/prog/euslib/irteus_proposals/wrench-torque-optimization.l")
  )

(defun init()
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
  (load-additional-files)
  (setq *robot* (jaxon_red))
  (send *robot* :fix-leg-to-coords (make-coords))
  (objects (list *robot*))
  (set-additional-coords)
  )

(defun set-additional-coords ();;デフォルトのend-coords以外に追加する
  (let* ((waist-link (send *robot* :link "BODY"))
         (chest-link (send *robot* :link "CHEST_LINK2"))
         )
    (send *robot* :put :waist-coords
          (make-cascoords
            :name :waist-coords
            :coords (send (send waist-link :copy-worldcoords) :worldcoords)
            :parent waist-link
            )
          )
    (send *robot* :put :chest-coords
          (make-cascoords
            :name :chest-coords
            :coords (send (send chest-link :copy-worldcoords) :worldcoords)
            :parent chest-link
            )
          )
    )
  )

(defun change-leg-end-coords-to-knee ();;leg-end-coordsを膝裏へ移動
  (let* ((rleg-new-coords (send (send *robot* :rleg :knee-p :parent-link) :copy-worldcoords))
         (lleg-new-coords (send (send *robot* :lleg :knee-p :parent-link) :copy-worldcoords))
         )
    (send rleg-new-coords :translate #f(-30 0 -270))
    (send lleg-new-coords :translate #f(-30 0 -270))
    (send rleg-new-coords :rotate (deg2rad 90) :y)
    (send lleg-new-coords :rotate (deg2rad 90) :y)
    (setq rleg-new-coords (make-cascoords :coords rleg-new-coords :name :rleg-end-coords))
    (setq lleg-new-coords (make-cascoords :coords lleg-new-coords :name :lleg-end-coords))
    (send (send *robot* :rleg :knee-p :parent-link) :assoc rleg-new-coords)
    (send (send *robot* :lleg :knee-p :parent-link) :assoc lleg-new-coords)
    (send *robot* :change-end-coords :rleg rleg-new-coords)
    (send *robot* :change-end-coords :lleg lleg-new-coords)
    )
  )

(defun fullbody-ik (rleg-tc lleg-tc rarm-tc larm-tc)
  (let* ((move-target-list (append (mapcar #'(lambda (limb) (send *robot* limb :end-coords)) (list :rleg :lleg :rarm :larm))
                                   (list (send *robot* :get :waist-coords) (send *robot* :get :chest-coords))))
         )
    (send *robot* :fullbody-inverse-kinematics
          (list rleg-tc lleg-tc rarm-tc larm-tc (make-coords) (make-coords))
          :move-target move-target-list
          :link-list (mapcar #'(lambda (mt) (send *robot* :link-list (send mt :parent))) move-target-list)
          :translation-axis (list t t t t nil nil)
          :rotation-axis (list t t t t :y :y)
          :target-centroid-pos (scale 0.25 (reduce #'v+ (mapcar #'(lambda (x) (send x :worldpos)) (list rleg-tc lleg-tc rarm-tc larm-tc))))
          :centroid-thre 100.0
          )
    )
  )

(defun reset-arm-pose ()
  (let* ((rleg-av (send *robot* :rleg :angle-vector))
         (lleg-av (send *robot* :lleg :angle-vector))
         )
    (send *robot* :reset-manip-pose)
    (send *robot* :rleg :angle-vector rleg-av)
    (send *robot* :lleg :angle-vector lleg-av)
    )
  )

(defun 4leg-walk-init-pose ();;4脚歩行用初期姿勢
  (let* ((rleg-tc)
         (lleg-tc)
         (rarm-tc)
         (larm-tc)
         )
    (send *robot* :reset-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (setq rleg-tc (send *robot* :rleg :end-coords :copy-worldcoords))
    (setq lleg-tc (send *robot* :lleg :end-coords :copy-worldcoords))
    (send *robot* :legs :move-end-pos #f(0 0 200) :world)
    (send *robot* :rleg :crotch-p :joint-angle -90)
    (send *robot* :lleg :crotch-p :joint-angle -90)
    (send *robot* :fix-leg-to-coords (make-coords))
    (setq rarm-tc (send rleg-tc :copy-worldcoords))
    (setq larm-tc (send lleg-tc :copy-worldcoords))
    (send rarm-tc :translate #f(600 -200 0) :world)
    (send rarm-tc :rotate (deg2rad 90) :y)
    (send larm-tc :translate #f(600 200 0) :world)
    (send larm-tc :rotate (deg2rad 90) :y)
    (fullbody-ik rleg-tc lleg-tc rarm-tc larm-tc)
    )
  )

(defun 4leg-walk1 ();;通常四脚歩行
  (let* ((ik-args
           (list
             :root-link-virtual-joint-weight #f(0.1 0.1 0.1 0.5 0.5 0.5)
           ))
         (footstep-list (send *robot* :go-pos-quadruped-params->footstep-list 200 100 0 :type :trot))
         )
    (send *robot* :calc-walk-pattern-from-footstep-list
          footstep-list :debug-view :no-message
          :all-limbs '(:rleg :lleg :rarm :larm)
          :init-pose-function #'(lambda ())
          :solve-angle-vector-args ik-args
          )
    )
  )

(defun 4leg-walk2 ();;両手を浮かして戻す
  (let* ((ik-args
           (list
             :root-link-virtual-joint-weight #f(0.1 0.1 0.1 0.5 0.5 0.5)
             ))
         (footstep-list)
         )
    (setq footstep-list
          (list
            (list (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
                  (make-coords :coords (send *robot* :lleg :end-coords :copy-worldcoords) :name :lleg))
            (list (make-coords :coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #f(1 0 0) :world) :name :rarm)
                  (make-coords :coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(1 0 0) :world) :name :larm))
            ))
    (send *robot* :calc-walk-pattern-from-footstep-list
          footstep-list :debug-view :no-message
          :all-limbs '(:rleg :lleg :rarm :larm)
          :init-pose-function #'(lambda ())
          :solve-angle-vector-args ik-args
          :default-step-height 70
          )
    )
  )

(defun 4leg-walk3 ();;両足を浮かして戻す
  (let* ((ik-args
           (list
             :root-link-virtual-joint-weight #f(0.1 0.1 0.1 0.5 0.5 0.5)
             ))
         (footstep-list)
         )
    (setq footstep-list
          (list
            (list (make-coords :coords (send *robot* :rarm :end-coords :copy-worldcoords) :name :rarm)
                  (make-coords :coords (send *robot* :larm :end-coords :copy-worldcoords) :name :larm))
            (list (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(1 0 0) :world) :name :rleg)
                  (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(1 0 0) :world) :name :lleg))
            ))
    (send *robot* :calc-walk-pattern-from-footstep-list
          footstep-list :debug-view :no-message
          :all-limbs '(:rleg :lleg :rarm :larm)
          :init-pose-function #'(lambda ())
          :solve-angle-vector-args ik-args
          :default-step-height 70
          )
    )
  )

(defun sitting-init-pose ();;座り移動動作の初期姿勢
  (when (and (boundp '*sitting-angle-vector*) (boundp '*sitting-coords*))
    (send *robot* :angle-vector *sitting-angle-vector*)
    (send *robot* :transform (send (car (send *robot* :links)) :transformation *sitting-coords* :world) :world)
    (objects (list *robot* *box* (send *robot* :rleg :end-coords) (send *robot* :lleg :end-coords) (send *robot* :rarm :end-coords) (send *robot* :larm :end-coords)))
    (return-from sitting-init-pose t)
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (let* ((rarm-tc)
         (larm-tc)
         )
    (setq *box* (make-cube 1000 1000 1000))
    (setq rarm-tc (make-coords :pos (copy-object (send *robot* :rarm :end-coords :worldpos))))
    (setq larm-tc (make-coords :pos (copy-object (send *robot* :larm :end-coords :worldpos))))
    (send rarm-tc :translate #f(-200 0 -150) :world)
    (send larm-tc :translate #f(-200 0 -150) :world)
    (send rarm-tc :rotate (deg2rad 90) :y)
    (send larm-tc :rotate (deg2rad 90) :y)
    (send rarm-tc :rotate (deg2rad -90) :x)
    (send larm-tc :rotate (deg2rad 90) :x)
    (send *robot* :chest_joint1 :joint-angle 0)
    (send *robot* :rarm :inverse-kinematics rarm-tc)
    (send *robot* :larm :inverse-kinematics larm-tc)
    (send *robot* :rleg :crotch-p :joint-angle -70)
    (send *robot* :lleg :crotch-p :joint-angle -70)
    (send *robot* :rleg :knee-p :joint-angle 60)
    (send *robot* :lleg :knee-p :joint-angle 60)
    (send *robot* :rleg :inverse-kinematics (make-coords :pos (copy-object (send *robot* :rleg :end-coords :worldpos))))
    (send *robot* :lleg :inverse-kinematics (make-coords :pos (copy-object (send *robot* :lleg :end-coords :worldpos))))
    (send *robot* :fix-leg-to-coords (make-coords))
    (setq *sitting-angle-vector* (send *robot* :angle-vector))
    (setq *sitting-coords* (send (car (send *robot* :links)) :copy-worldcoords))
    (send *box* :translate #f(-700 0 -16))
    (objects (list *robot* *box* (send *robot* :rleg :end-coords) (send *robot* :lleg :end-coords) (send *robot* :rarm :end-coords) (send *robot* :larm :end-coords)))
    )
  )

(defun seated-walk1 ();;膝裏を離して戻す？
  (let* ((ik-args
           (list 
             :rotation-axis (list :y :y t t)
             :translation-axis (list :x :x t t)
             :root-link-virtual-joint-weight #f(0.5 0.1 0.1 0.1 0.5 0.1)
             )
           )
         (footstep-list)
         )
    (setq footstep-list
          (list
            (list (make-coords :coords (send *robot* :rarm :end-coords :copy-worldcoords) :name :rarm)
                  (make-coords :coords (send *robot* :larm :end-coords :copy-worldcoords) :name :larm))
            (list (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
                  (make-coords :coords (send *robot* :lleg :end-coords :copy-worldcoords) :name :lleg))
            ))
    (setq *ret* (send *robot* :calc-walk-pattern-from-footstep-list
                      footstep-list :debug-view :no-message
                      :all-limbs '(:rleg :lleg :rarm :larm)
                      :init-pose-function #'(lambda ())
                      :solve-angle-vector-args ik-args
                      :default-step-height 30
                      )
          )
    )
  )

(defun seated-walk2 ();;座り水平移動(rotation-axisをtにすると全身が水平移動するが一部IKが解けない、nilにするとIKは解けるが膝裏のみ移動しているような動作になる)
  (let* ((ik-args
           (list 
             :rotation-axis (list nil nil t t)
             :translation-axis (list :x :x t t)
             :root-link-virtual-joint-weight #f(0.5 0.1 0.1 0.1 0.5 0.1)
             )
           )
         (footstep-list)
         )
    (setq footstep-list
          (list
            (list (make-coords :coords (send *robot* :rarm :end-coords :copy-worldcoords) :name :rarm)
                  (make-coords :coords (send *robot* :larm :end-coords :copy-worldcoords) :name :larm))
            (list (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(0 50 0) :world) :name :rleg)
                  (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(0 50 0) :world) :name :lleg))
            ))
    (setq *ret* (send *robot* :calc-walk-pattern-from-footstep-list
                      footstep-list :debug-view :no-message
                      :all-limbs '(:rleg :lleg :rarm :larm)
                      :init-pose-function #'(lambda ())
                      :solve-angle-vector-args ik-args
                      :default-step-height 30
                      ;; :dt 0.01
                      )
          )
    )
  )

(defun seated-walk3 ();;座り前進移動
  (let* ((ik-args
           (list 
             :rotation-axis (list :y :y t t)
             :translation-axis (list t t t t)
             :root-link-virtual-joint-weight #f(0.5 0.1 0.1 0.1 0.5 0.1)
             )
           )
         (footstep-list)
         )
    (setq footstep-list
          (list
            (list (make-coords :coords (send *robot* :rarm :end-coords :copy-worldcoords) :name :rarm)
                  (make-coords :coords (send *robot* :larm :end-coords :copy-worldcoords) :name :larm))
            (list (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0) :world) :name :rleg)
                  (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0) :world) :name :lleg))
            ))
    (setq *ret* (send *robot* :calc-walk-pattern-from-footstep-list
                      footstep-list :debug-view :no-message
                      :all-limbs '(:rleg :lleg :rarm :larm)
                      :init-pose-function #'(lambda ())
                      :solve-angle-vector-args ik-args
                      :default-step-height 30
                      ;; :dt 0.01
                      )
          )
    )
  )

(defun calc-contact-wrench (data)
  (let* ((ret)
         )
    (setq ret (convert-to-robot-state-data data))
    (setq ret (calc-robot-state-data-list-with-wrench-torque ret))
    (setq *rs-list* ret)
    )
  )

(defun view-motion (rs-list)
  (play-robot-state-data-list rs-list)
  )

(defun view-cog (ret)
  (load "~/prog/euslib/jsk/gnuplotlib.l")
  (let* ((cogx-list)
         (cogz-list)
         (zmpx-list)
         (czmpx-list)
         (pzx-list)
         (time-list)
         (rlegx-list)
         (rlegz-list)
         )
    (dolist (l ret)
      (setq cogx-list (append cogx-list (list (elt (cadr (memq :cog l)) 0))))
      (setq cogz-list (append cogz-list (list (elt (cadr (memq :cog l)) 2))))
      (setq zmpx-list (append zmpx-list (list (elt (cadr (memq :zmp l)) 0))))
      (setq czmpx-list (append czmpx-list (list (elt (cadr (memq :czmp l)) 0))))
      (setq pzx-list (append pzx-list (list (elt (cadr (memq :pz l)) 0))))
      (setq time-list (append time-list (list (cadr (memq :time l)))))
      (let* ((rleg-coords (elt (cadr (memq :end-coords-list l)) 0))
             )
        (setq rlegx-list (append rlegx-list (list (elt (send rleg-coords :worldpos) 0))))
        (setq rlegz-list (append rlegz-list (list (elt (send rleg-coords :worldpos) 2))))
        )
      )
    (graph-view
      (list cogx-list zmpx-list czmpx-list pzx-list)
      time-list
      )
    )
  )


(defun seated-walk3-auto ()
  (sitting-init-pose)
  (change-leg-end-coords-to-knee)
  (seated-walk3)
  (calc-contact-wrench *ret*)
  (view-motion *rs-list*)
  )

(defun seated-walk2-auto ()
  (sitting-init-pose)
  (change-leg-end-coords-to-knee)
  (seated-walk2)
  (calc-contact-wrench *ret*)
  (view-motion *rs-list*)
  )
