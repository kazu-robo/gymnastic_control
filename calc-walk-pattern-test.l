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


(defun change-leg-end-coords-to-thigh ();;leg-end-coordsを膝裏へ移動
  (let* ((rleg-new-coords (send (send (send (send *robot* :rleg :knee-p :parent-link) :copy-worldcoords) :rotate (deg2rad 90) :y) :translate #f(100 0 0)))
         (lleg-new-coords (send (send (send (send *robot* :lleg :knee-p :parent-link) :copy-worldcoords) :rotate (deg2rad 90) :y) :translate #f(100 0 0)))
         )
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


(defun sitting-init-pose2 ();;座り移動動作の初期姿勢
  ;; (when (and (boundp '*sitting-angle-vector*) (boundp '*sitting-coords*))
  ;;   (send *robot* :angle-vector *sitting-angle-vector*)
  ;;   (send *robot* :transform (send (car (send *robot* :links)) :transformation *sitting-coords* :world) :world)
  ;;   (objects (list *robot* *box* (send *robot* :rleg :end-coords) (send *robot* :lleg :end-coords) (send *robot* :rarm :end-coords) (send *robot* :larm :end-coords)))
  ;;   (return-from sitting-init-pose t)
  ;;   )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *box* (make-cube 1000 2000 700))
  (send *box* :translate #f(500 0 350))
  (send *robot* :move-coords (make-coords :pos #f(133.027 86.451 886.373) :rot #2f((-0.999995 -0.001581 0.002839) (0.001585 -0.999998 0.001425) (0.002837 0.00143 0.999995))) (car (send *robot* :links)))
  (send *robot* :angle-vector #f(-0.00125 0.000429 -90.0 100.0 -41.2631 -0.059189 -0.001299 0.048607 -90.0 100.0 -41.3527 -0.006105 -0.041261 0.0 0.000185 -5.729886e-07 0.002006 43.9041 69.8959 -28.6728 12.2567 -79.6918 -24.1632 -4.92543 5.69747 -42.309 70.0927 26.4068 -13.7306 -81.0846 23.8146 7.61558 6.38332))
  (objects (list *robot* *box* (send *robot* :rleg :end-coords) (send *robot* :lleg :end-coords) (send *robot* :rarm :end-coords) (send *robot* :larm :end-coords)))
  ;; (send *robot* :arms :move-end-pos #f(50 0 0) :world)
  (send *robot* :rarm :move-end-pos #f(0 50 0) :world)
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
             :rotation-axis (list t t t t)
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

(defun sit-down ();;after seated-walk2
  (send *robot* :legs :crotch-p :joint-angle -90)
  (send *robot* :arms :move-end-pos #f(-70 0 0))
  (send *robot* :translate #f(0 0 -62))
  )
  
(defun seated-walk2-tmp ();;座り水平移動(rotation-axisをtにすると全身が水平移動するが一部IKが解けない、nilにするとIKは解けるが膝裏のみ移動しているような動作になる)
  (let* ((ik-args
           (list 
             ;; :rotation-axis (list nil nil t t)
             :rotation-axis (list t t t t)
             :translation-axis (list t t t t)
             :root-link-virtual-joint-weight #f(0.5 0.1 0.1 0.1 0.5 0.1)
             )
           )
         (footstep-list)
         (dis 30)
         )
    (setq footstep-list
          (list
            (list (make-coords :coords (send *robot* :rarm :end-coords :copy-worldcoords) :name :rarm)
                  (make-coords :coords (send *robot* :larm :end-coords :copy-worldcoords) :name :larm))
            (list (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 0 dis 0) :world) :name :rleg)
                  (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 0 dis 0) :world) :name :lleg))
            (list (make-coords :coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (float-vector 0 dis 0) :world) :name :rarm)
                  (make-coords :coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate (float-vector 0 dis 0) :world) :name :larm))
            ))
    (setq *ret* (send *robot* :calc-walk-pattern-from-footstep-list
                      footstep-list :debug-view :no-message
                      :all-limbs '(:rleg :lleg :rarm :larm)
                      :init-pose-function #'(lambda ())
                      :solve-angle-vector-args ik-args
                      :default-step-height 30
                      :dt 0.01
                      )
          )
    )
  )

(defun seated-walk2-after ();;座り水平移動(rotation-axisをtにすると全身が水平移動するが一部IKが解けない、nilにするとIKは解けるが膝裏のみ移動しているような動作になる)
  (let* ((ik-args
           (list 
             :rotation-axis (list nil nil t t)
             ;; :rotation-axis (list t t t t)
             ;; :translation-axis (list :x :x t t)
             :translation-axis (list t t t t)
             :root-link-virtual-joint-weight #f(0.5 0.1 0.1 0.1 0.5 0.1)
             )
           )
         (footstep-list)
         )
    (setq footstep-list
          (list
            (list (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
                  (make-coords :coords (send *robot* :lleg :end-coords :copy-worldcoords) :name :lleg))
            (list (make-coords :coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #f(0 50 0) :world) :name :rarm)
                  (make-coords :coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(0 50 0) :world) :name :larm))
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

(defun sit-right ();;座り水平移動(rotation-axisをtにすると全身が水平移動するが一部IKが解けない、nilにするとIKは解けるが膝裏のみ移動しているような動作になる)
  (let* ((ik-args
           (list 
             :rotation-axis (list t t t t)
             :translation-axis (list t t t t)
             :root-link-virtual-joint-weight #f(0.5 0.1 0.1 0.1 0.1 0)
             :additional-weight-list (list (list (send *robot* :torso :waist-y :child-link) 0.0)
                                           (list (send *robot* :torso :waist-p :child-link) 0.2)
                                           (list (send *robot* :rarm :collar-y :child-link) 0.1)
                                           (list (send *robot* :larm :collar-y :child-link) 0.1))
             :centroid-thre 400
             :additional-check #'(lambda () (< (abs (elt (v- (send *robot* :centroid) (send *robot* :larm :end-coords :worldpos)) 0)) 3))
             :thre (list 10 10 10 10)
             )
           )
         (footstep-list)
         (dis 50)
         )
    (setq footstep-list
          (list
            (list (make-coords :coords (send *robot* :rarm :end-coords :copy-worldcoords) :name :rarm)
                  (make-coords :coords (send *robot* :larm :end-coords :copy-worldcoords) :name :larm))
            (list (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 0 dis 0) :world) :name :rleg)
                  (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 0 dis 0) :world) :name :lleg))
            ;; (list (make-coords :coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (float-vector 0 dis 0) :world) :name :rarm)
            ;;       (make-coords :coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate (float-vector 0 dis 0) :world) :name :larm))
            ))
    (setq *ret* (send *robot* :calc-walk-pattern-from-footstep-list
                      footstep-list :debug-view :no-message
                      :all-limbs '(:rleg :lleg :rarm :larm)
                      :init-pose-function #'(lambda ())
                      :solve-angle-vector-args ik-args
                      :default-step-height 20
                      :default-step-time 3
                      ;; :default-step-time 2
                      :dt 0.01
                      ;; :dt 0.05
                      )
          )
    )
  )

;; (defun hand-move ();;座り水平移動(rotation-axisをtにすると全身が水平移動するが一部IKが解けない、nilにするとIKは解けるが膝裏のみ移動しているような動作になる)
;;   (let* ((ik-args
;;            (list 
;;              :rotation-axis (list t t t t)
;;              :translation-axis (list t t t t)
;;              :root-link-virtual-joint-weight #f(0.5 0.1 0.1 0.1 0.1 0)
;;              :additional-weight-list (list (list (send *robot* :torso :waist-y :child-link) 0.0)
;;                                            (list (send *robot* :torso :waist-p :child-link) 0.2)
;;                                            (list (send *robot* :rarm :collar-y :child-link) 0.1)
;;                                            (list (send *robot* :larm :collar-y :child-link) 0.1))
;;              :centroid-thre 400
;;              :additional-check #'(lambda () (< (abs (elt (v- (send *robot* :centroid) (send *robot* :larm :end-coords :worldpos)) 0)) 3))
;;              :thre (list 10 10 10 10)
;;              )
;;            )
;;          (footstep-list)
;;          (dis 50)
;;          )
;;     (setq footstep-list
;;           (list
;;             ;; (list (make-coords :coords (send *robot* :rarm :end-coords :copy-worldcoords) :name :rarm)
;;             ;;       (make-coords :coords (send *robot* :larm :end-coords :copy-worldcoords) :name :larm))
;;             (list (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
;;                   (make-coords :coords (send *robot* :lleg :end-coords :copy-worldcoords) :name :lleg))
;;             (list (make-coords :coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (float-vector 0 dis 0) :world) :name :rarm)
;;                   (make-coords :coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate (float-vector 0 dis 0) :world) :name :larm))
;;             ))
;;     (setq *ret* (send *robot* :calc-walk-pattern-from-footstep-list
;;                       footstep-list :debug-view :no-message
;;                       :all-limbs '(:rleg :lleg :rarm :larm)
;;                       :init-pose-function #'(lambda ())
;;                       :solve-angle-vector-args ik-args
;;                       :default-step-height 20
;;                       :default-step-time 1.5
;;                       ;; :default-step-time 2
;;                       :dt 0.01
;;                       ;; :dt 0.05
;;                       )
;;           )
;;     )
;;   )

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

(defun seated-walk2-tmp-auto ()
  (sitting-init-pose)
  (change-leg-end-coords-to-knee)
  (seated-walk2-tmp)
  (calc-contact-wrench *ret*)
  (view-motion *rs-list*)
  )

(defun seated-walk2-after-auto ()
  (sitting-init-pose)
  (change-leg-end-coords-to-knee)
  (send *robot* :move-coords (make-coords :pos #f(-424.278 14.248 733.671) :rot #2f((0.999254 0.029422 0.024997) (-0.029468 0.999565 0.001456) (-0.024944 -0.002191 0.999686))) (car (send *robot* :links)))
  (send *robot* :angle-vector #f(11.1151 2.83926 -69.2908 48.8696 21.012 1.195051e-05 10.7202 4.02729 -68.7446 48.8696 21.012 -1.195051e-05 1.06823 7.52032 0.664543 0.0 0.0 18.1424 47.4942 -34.436 22.2183 -69.6967 32.9243 2.12014 10.363 -18.4724 45.2004 29.468 -24.1269 -67.9893 -33.7431 -1.09776 11.2021))
  (seated-walk2-after)
  (calc-contact-wrench *ret*)
  (view-motion *rs-list*)
  )


(defun sit-right-auto ()
  (sitting-init-pose2)
  (change-leg-end-coords-to-thigh)
  (sit-right)
  (calc-contact-wrench *ret*)
  (view-motion *rs-list*)
  (setq *sit-right-move* *rs-list*)
  )


;; (defun arm-move ()
;;   (send *robot* :arms :move-end-pos #f(0 50 50) :world)
;;   (send *ri* :angle-vector (send *robot* :angle-vector))
;;   (send *robot* :arms :move-end-pos #f(0 0 -50) :world)
;;   (send *ri* :angle-vector (send *robot* :angle-vector))
;;   (send *ri* :angle-vector-sequence-full (list (car av-list-seq)) (list 10000) :fsensor-length fnum :root-coords (list (car rc-list)) :zmp (list (car zmp-list)) :wrench (list (car wrench-list)) :optional (list (car cs-list)))
;;   )


  
;; (defun hand-move-auto ()
;;   (send *robot* :move-coords (make-coords :pos #f(152.838 136.346 884.711) :rot #2f((-0.986347 -0.002765 0.164656) (0.002926 -0.999995 0.000734) (0.164654 0.001206 0.986351))) (car (send *robot* :links)))
;;   (send *robot* :angle-vector #f(0.063478 0.024051 -80.685 100.0 -41.2631 -0.059189 0.055625 0.071614 -80.6853 100.0 -41.3527 -0.006105 -4.11011 22.1896 0.000185 -5.729886e-07 0.002006 23.7694 40.7738 -61.2952 44.087 -83.7723 -40.437 -16.5019 1.15739 -18.3607 56.0922 70.4297 -37.7342 -90.9572 45.8661 24.1926 -1.66276))
;;   (change-leg-end-coords-to-thigh)
;;   (hand-move)
;;   (calc-contact-wrench *ret*)
;;   (view-motion *rs-list*)
;;   (setq *hand-move* *rs-list*)
;;   )


(defun save-param (&key (filename "tmp-param-file.l"))
  (dump-loadable-structure filename av-list tm-list-for-seq zmp-list rc-list wrench-list cs-list)
  )
