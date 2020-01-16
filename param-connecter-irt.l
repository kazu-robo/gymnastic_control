
(defun get-param (&key (dt 0.01));;todo  timeの遷移で一定期間angle-vector変わらなくなったらそこまでにするなどする
  ;; (setq time-list (reverse (send simint::*logger-buf* :get :tm)))
  ;; (setq av-list (mapcar '(lambda (a) (scale (rad2deg 1) a)) (reverse (send simint::*logger-buf* :get :cav))))
  ;; ;; (setq av-list (mapcar '(lambda (a) (scale (rad2deg 1) a)) (reverse (send simint::*logger-buf* :get :tav))))
  ;; (setq pose0-av (simint::calc-lagrange-traj 0))
  ;; (setq rc-list (reverse (send simint::*logger-buf* :get :rc)))
  ;; (setq rc-list (mapcar #'(lambda (rc) (send rc :locate (scale 1000 (send rc :worldpos)) :world)) rc-list))
  ;; ;; (dotimes (i (length rc-list))
  ;; ;;   (setq tmp-rc (scale 1000 (send (elt rc-list i) :pos)))
  ;; ;;   (send (elt rc-list i) :translate (scale -1 (send (elt rc-list i) :pos)))
  ;; ;;   (send (elt rc-list i) :translate tmp-rc));;rc-listのposをmmに変更
  ;; (setq cs-list (reverse (send simint::*logger-buf* :get :cs)));;virtual force sensor の位置変えたい場合はnlopt-interpolation.lのcs変えるように
  ;; (setq zmp-list (reverse (send simint::*logger-buf* :get :zmp)))
  ;; (setq tm-list-for-seq nil)
  ;; (setq fnum (+ (length (send *robot* :force-sensors)) 2));;change num(2) when you change numbers of virtual force sensors
  
  ;; (setq rff (reverse (send simint::*logger-buf* :get :rff)))
  ;; (setq rfm (reverse (send simint::*logger-buf* :get :rfm)))
  ;; (setq lff (reverse (send simint::*logger-buf* :get :lff)))
  ;; (setq lfm (reverse (send simint::*logger-buf* :get :lfm)))
  ;; (setq rhf (reverse (send simint::*logger-buf* :get :rhf)))
  ;; (setq rhm (reverse (send simint::*logger-buf* :get :rhm)))
  ;; (setq lhf (reverse (send simint::*logger-buf* :get :lhf)))
  ;; (setq lhm (reverse (send simint::*logger-buf* :get :lhm)))
   
  ;; (dotimes (i (length time-list))
  ;;   (setq tm-list-for-seq (cons (* 1000 dt) (copy-seq tm-list-for-seq)))
  ;;   )
  ;; (setq full-torque-start-time simint::*blend-time-rate*)

  ;; ;;接触点の代表値（virtualforcesensorに渡す値）
  ;; (setq rvs-point #f(0 0 0) lvs-point #f(0 0 0))
  ;; (dotimes (i (length (send simint::*logger-buf* :get :rvs-p)))
  ;;   (setq rvs-point
  ;;         (if (not (elt (send simint::*logger-buf* :get :rvs-p) i))
  ;;             rvs-point
  ;;           (v+ (elt (send simint::*logger-buf* :get :rvs-p) i) rvs-point))))
  ;; (setq rvs-x (/ (aref rvs-point 0) (length (send simint::*logger-buf* :get :rvs-p))))
  ;; (setq rvs-y (/ (aref rvs-point 1) (length (send simint::*logger-buf* :get :rvs-p))))
  ;; (setq rvs-z (/ (aref rvs-point 2) (length (send simint::*logger-buf* :get :rvs-p))))
  ;; (setq rvs-contact (float-vector rvs-x rvs-y rvs-z))
  
  ;; (dotimes (i (length (send simint::*logger-buf* :get :lvs-p)))
  ;;   (setq lvs-point
  ;;         (if (not (elt (send simint::*logger-buf* :get :lvs-p) i))
  ;;             lvs-point
  ;;           (v+ (elt (send simint::*logger-buf* :get :lvs-p) i) lvs-point))))
  ;; (setq lvs-x (/ (aref lvs-point 0) (length (send simint::*logger-buf* :get :lvs-p))))
  ;; (setq lvs-y (/ (aref lvs-point 1) (length (send simint::*logger-buf* :get :lvs-p))))
  ;; (setq lvs-z (/ (aref lvs-point 2) (length (send simint::*logger-buf* :get :lvs-p))))
  ;; (setq lvs-contact (float-vector lvs-x lvs-y lvs-z))
  
  ;; ;;時系列ごとの予想接触レンチ（接触代表点に換算）;;rcrotch-wrench-list, lcrotch-wrench-list が各目標レンチの時系列リスト #f(fx fy fz mx my mz);;接触力推定する座標系しろ！(momentのみ）
  ;; (setq rcrotch-jmax-list (reverse (send simint::*logger-buf* :get :rcrotch-jmax)))
  ;; (setq rcrotch-f-list (reverse (send simint::*logger-buf* :get :rcrotchf-list)))
  ;; (setq rcrotch-poslist (reverse (send simint::*logger-buf* :get :rcrotch-poslist)))
  ;; (setq rcrotch-f-list-world (reverse (send simint::*logger-buf* :get :rcrotchf-list-world)))
  ;; (setq lcrotch-jmax-list (reverse (send simint::*logger-buf* :get :lcrotch-jmax)))
  ;; (setq lcrotch-f-list (reverse (send simint::*logger-buf* :get :lcrotchf-list)))
  ;; (setq lcrotch-poslist (reverse (send simint::*logger-buf* :get :lcrotch-poslist)))
  ;; (setq lcrotch-f-list-world (reverse (send simint::*logger-buf* :get :lcrotchf-list-world)))
  ;; (setq rcrotch-wrench-list (list) lcrotch-wrench-list (list))
  ;; (dotimes (i (length rcrotch-jmax-list))
  ;;   (setq rjmax-tmp (elt rcrotch-jmax-list i))
  ;;   (setq rflist-tmp (elt rcrotch-f-list i))
  ;;   (setq rflist-world-tmp (elt rcrotch-f-list-world i))
  ;;   (setq rposlist-tmp (elt rcrotch-poslist i))
  ;;   (setq tmp 0)
  ;;   (setq rcrotch-f #f(0 0 0) rcrotch-moment #f(0 0 0))
  ;;   (while (< tmp rjmax-tmp)
  ;;     (setq rcrotch-f (v+ rcrotch-f (elt rflist-world-tmp tmp)))
  ;;     (setq rcrotch-moment (v+ rcrotch-moment (v* (v- (elt rposlist-tmp tmp) rvs-contact) (elt rflist-tmp tmp))));;;v* float-vectorの外積
  ;;     (setq tmp (+ tmp 1)))
  ;;   (setq rcrotch-wrench-list (cons (concatenate float-vector rcrotch-f (scale 0.001 rcrotch-moment)) rcrotch-wrench-list))
  ;;   )
  ;; (setq rcrotch-wrench-list (reverse rcrotch-wrench-list))
  ;; (dotimes (i (length lcrotch-jmax-list))
  ;;   (setq ljmax-tmp (elt lcrotch-jmax-list i))
  ;;   (setq lflist-tmp (elt lcrotch-f-list i))
  ;;   (setq lflist-world-tmp (elt lcrotch-f-list-world i))
  ;;   (setq lposlist-tmp (elt lcrotch-poslist i))
  ;;   (setq tmp 0)
  ;;   (setq lcrotch-f #f(0 0 0) lcrotch-moment #f(0 0 0))
  ;;   (while (< tmp ljmax-tmp)
  ;;     (setq lcrotch-f (v+ lcrotch-f (elt lflist-world-tmp tmp)))
  ;;     (setq lcrotch-moment (v+ lcrotch-moment (v* (v- (elt lposlist-tmp tmp) lvs-contact) (elt lflist-tmp tmp))))
  ;;     (setq tmp (+ tmp 1)))
  ;;   (setq lcrotch-wrench-list (cons (concatenate float-vector lcrotch-f (scale 0.001 lcrotch-moment)) lcrotch-wrench-list))
  ;;   )
  ;; (setq lcrotch-wrench-list (reverse lcrotch-wrench-list))
  ;; (setq wrench-list (list))
  ;; (dotimes (i (length time-list))
    
  ;;   (setq wrench-list (cons
  ;;                      (concatenate float-vector
  ;;                                   (scale -1 (concatenate float-vector (elt rff i) (elt rfm i) (elt lff i) (elt lfm i) (elt rhf i) (elt rhm i) (elt lhf i) (elt lhm i)))
  ;;                                   (elt rcrotch-wrench-list i) (elt lcrotch-wrench-list i)) wrench-list)))
  ;; ;;rffなどで取ってくる値、全成分で値が正負反転してる（モーメントはたぶん）ここでロボットが受ける力に直してる
  ;; (setq wrench-list (reverse wrench-list))
  ;; (print "get-param finished :: next -> jaxoninit or make-key-pose")
  
  ;; ;; (pose0)
  ;; ;; (send *robot* :arms :move-end-pos (float-vector 0 0 200))
  ;; ;; (setq arm-target-key (send *robot* :angle-vector))
  ;; ;; (setq key-rc (send *robot* :copy-worldcoords))
  
  ;; ;; (setq key-zmp (send *robot* :centroid))
  (setq av-list (list))
  (setq prev-tm 100)
  (setq tm-list-for-seq (list))
  (setq zmp-list (list))
  (setq rc-list (list))
  (setq cs-list (list))
  (setq wrench-list (list))
  (setq rs-list-tmp *rs-list*)
  (dotimes (i (length rs-list-tmp))
    (setq rs (car rs-list-tmp))
    (setq rs-list-tmp (cdr rs-list-tmp))
    (setq av-list (push (send rs :angle-vector) av-list))
    (setq tm-list-for-seq
          (push
           (* 1000 (- (send rs :time) prev-tm))
           tm-list-for-seq))
    (setq prev-tm (send rs :time))
    (setq zmp-list (push (send rs :zmp) zmp-list))
    (setq rc-list (push (send rs :root-coords) rc-list))
    (setq cs (send rs :contact-state))
    (setq cs-now (list))
    (dotimes (i (length (send rs :contact-state)))
      (if (equal (car cs) :support)
          (push 1 cs-now)
        (push 0 cs-now))
      (setq cs (cdr cs))
      )
    (setq cs-now (reverse cs-now))
    (setq cs-one (car cs-now))
    (setq cs-two (car (cdr cs-now)))
    (setq cs-three (car (cdr (cdr cs-now))))
    (setq cs-four (car (cdr (cdr (cdr cs-now)))))
    (setq cs-now (list))
    (push 1.5 cs-now);;contact phase time
    (push 1.5 cs-now)
    (push 1.5 cs-now)
    (push 1.5 cs-now)
    (push 1.5 cs-now)
    (push 1.5 cs-now)
    (push cs-two cs-now)
    (push cs-one cs-now)
    (push cs-four cs-now)
    (push cs-three cs-now)
    (push 0 cs-now)
    (push 0 cs-now)
    (setq cs-list (push cs-now cs-list))

    (setq wrench (send rs :wrench-list))
    (setq wrench-now (list))
    (setq force-now (car wrench))
    (setq moment-now (car (cdr wrench)))
    (setq wrench-one (concatenate float-vector (car force-now) (car moment-now)))
    (setq wrench-two (concatenate float-vector (car (cdr force-now)) (car (cdr moment-now))))
    (setq wrench-three (concatenate float-vector (car (cdr (cdr force-now))) (car (cdr (cdr moment-now)))))
    (setq wrench-four (concatenate float-vector (car (cdr (cdr (cdr force-now)))) (car (cdr (cdr (cdr moment-now))))))
    (setq wrench-now (concatenate float-vector #f(0 0 0 0 0 0) #f(0 0 0 0 0 0) wrench-three wrench-four wrench-one wrench-two))
    (setq wrench-list (push wrench-now wrench-list));;;wrenchとcsの順番腿裏の前提で決め打ちなので注意
    )
  (setq av-list (reverse av-list))
  (push (car av-list) av-list)
  (push (car av-list) av-list)
  (setq tm-list-for-seq (reverse tm-list-for-seq))
  (push (car tm-list-for-seq) tm-list-for-seq)
  (push (car tm-list-for-seq) tm-list-for-seq)
  (setq zmp-list (reverse zmp-list))
  (push (car zmp-list) zmp-list)
  (push (car zmp-list) zmp-list)
  (setq rc-list (reverse rc-list))
  (push (car rc-list) rc-list)
  (push (car rc-list) rc-list)
  (setq cs-list (reverse cs-list))
  (push (car cs-list) cs-list)
  (push (car cs-list) cs-list)
  (setq wrench-list (reverse wrench-list))
  (push (car wrench-list) wrench-list)
  (push (car wrench-list) wrench-list)
  (setq fnum (+ (length (send *robot* :force-sensors)) 2))
  )
;; 最初の二つ足す

;;send *robot* :fix-leg-to-coords (make-coords :pos (send (midcoords 0.5 (send *robot* :lleg :end-coords :copy-worldcoords) (send *robot* :rleg :end-coords :copy-worldcoords)) :pos) :rpy #f(3.14 0 0))
;;key-zmp#f(-137.828 0.063792 968.308)になるはず
;; (defun get-root-link-coords ();;;;TODO;;;;;;時間に合わせて取るようにする
;;   (setq rc-from-sim (send simint::*logger-buf* :get :rc));;格納順が逆になってるから注意、avの順も確認,その他全部

;;   )

;; (defun get-contact-force ()
;;   (setq rff-from-sim (send simint::*logger-buf* :get :rff))
;;   (setq lff-from-sim (send simint::*logger-buf* :get :lff))
  ;;   )



(defun jaxoninit ()
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
  ;; (load "package://control_tools/log-tools.l")
  (load "package://control_tools/play-hrpsysfile.l")
  (unless (boundp '*robot*) (setq *robot* (jaxon_red)))
  (send *robot* :fix-leg-to-coords (make-coords))
  (objects (list *robot*))
  (jaxon_red-init)
  )


(defun initial-set-up ()
  (warning-message 3 "remove external force offset (for estimating force)~%")
  (send *ri* :remove-external-force-offset)
  (warning-message 3 "start estimating force~%")
  (send *ri* :start-virtual-force-sensor :name "vrthighsensor")
  (send *ri* :start-virtual-force-sensor :name "vlthighsensor")
  
  (warning-message 3 "stop st~%")
  (send *ri* :stop-st)
  (warning-message 3 "(reset-pose)~%")
  (reset-pose :contact-state-list '(1 1 0 0 0 0))
  (unix:sleep 10)
  (warning-message 3 "stop abc~%")
  (send *ri* :stop-auto-balancer)
  (warning-message 3 "stop ic~%")
  (send *ri* :stop-impedance :arms)

  (warning-message 3 "set st param~%")
  ;; (send *ri* :set-st-param :is-ik-enable (list t t t t nil nil));;トルク制御ならいらない
  (send *ri* :set-st-param :st-algorithm :eefmqpcop2)
  (send *ri* :set-st-param :is-zmp-calc-enable (list t t t t t t));;この辺の後々確認
  ;;t t t t nil nil試す
  (send *ri* :set-st-param :eefm-zmp-delay-time-const #f(0 0));;一時進み要素的なやつのパラメータ０にするやつ
  (send *ri* :set-st-param :is-feedback-control-enable (list t t t t t t));;再分配可能にする？
  (send *ri* :set-st-param :eefm-ee-forcemoment-distribution-weight
        (list #f(0 0 1 1 1 0) #f(0 0 1 1 1 0) #f(0 0 1 0 0 0) #f(0 0 1 0 0 0) #f(0 0 1 1 1 0) #f(0 0 1 1 1 0)));;zmp再分配の重み
  
  (warning-message 3 "start st~%")
  (send *ri* :start-st)
  (warning-message 3 "disable emergency check~%")
  (send *ri* :set-st-param :emergency-check-mode 0)
  
  ;; (warning-message 3 "remove external force offset (for estimating force)~%")
  ;; (send *ri* :remove-external-force-offset)
  ;; (warning-message 3 "start estimating force~%")
  ;; (send *ri* :start-virtual-force-sensor :name "vrthighsensor")
  ;; (send *ri* :start-virtual-force-sensor :name "vlthighsensor")
  )


(defun initial-set-up-jikki ()
  (when (y-or-n-p "standing with reset-pose (without st)?~%")
    (send *ri* :start-grasp)
    (warning-message 3 "remove external force offset (for estimating force)~%")
    (send *ri* :remove-external-force-offset)
    (warning-message 3 "start estimating force~%")
    (send *ri* :start-virtual-force-sensor :name "vrthighsensor")
    (send *ri* :start-virtual-force-sensor :name "vlthighsensor")
    (warning-message 3 "set st param~%")
    (send *ri* :set-st-param :st-algorithm :eefmqpcop2)
    (send *ri* :set-st-param :is-zmp-calc-enable (list t t t t t t));;この辺の後々確認
    ;;t t t t nil nil試す
    (send *ri* :set-st-param :eefm-zmp-delay-time-const #f(0 0));;一時進み要素的なやつのパラメータ０にするやつ
    (send *ri* :set-st-param :is-feedback-control-enable (list t t t t t t));;再分配可能にする？
    (send *ri* :set-st-param :eefm-ee-forcemoment-distribution-weight
          (list #f(0 0 1 1 1 0) #f(0 0 1 1 1 0) #f(0 0 1 0 0 0) #f(0 0 1 0 0 0) #f(0 0 1 0 0 0) #f(0 0 1 0 0 0)));;zmp再分配の重み
    (warning-message 3 "disable emergency check~%")
    (send *ri* :set-st-param :emergency-check-mode 0)
    )
  )

(defun make-key-pose (&key (com-offset -50) (key-arm-height 0));;(&key (leg-xpos-target 800) (leg-zpos-target 50))
  ;; (pose0)
  ;; (send *robot* :arms :move-end-pos (float-vector (- 0 key-arm-height) 0 500))
  (let* (
	 (centroid-coords
	  (send *robot* :centroid))
	 (rleg-coords
	  (send *robot* :rleg :end-coords :copy-worldcoords))
	 (lleg-coords
	  (send *robot* :lleg :end-coords :copy-worldcoords))
	 (rarm-coords
	  (send *robot* :rarm :end-coords :copy-worldcoords))
	 (larm-coords
	  (send *robot* :larm :end-coords :copy-worldcoords))
	 (target-coords
	  (list
       rarm-coords
       larm-coords
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
    (send *robot* :reset-pose)

    ;; (send *robot*
	;;   :fullbody-inverse-kinematics target-coords
	;;   :move-target limbs
	;;   :link-list (mapcar #' (lambda (limb) (send *robot* :link-list (send limb :parent))) limbs)
    ;;   :target-centroid-pos (v+ (float-vector com-offset 0 0) (send (midcoords 0.5 lleg-coords rleg-coords) :worldpos))
      
	;;   )
    ;; (setq key-av (send *robot* :angle-vector))
    
    ;; (setq key-rc (send *robot* :copy-worldcoords))
    ;; (setq key-zmp (send *robot* :centroid))

    (setq finger-av (scale 60 #f(-1 1 -1 1)))
    ;; (setq finger-av (scale 65 #f(-1 1 -1 1)))
    ;; (setq finger-av (scale 70 #f(-1 1 -1 1)));;without touch, not move much at start-st
    ;; (if (and (is-choreonoid) (equal (length pose0-av) 33))
    ;;     ;; (setq pose0-av (concatenate float-vector pose0-av #f(-80 80 -80 80)))
    ;;     (setq pose0-av (concatenate float-vector pose0-av finger-av))
    ;;   )
    ;; (if (and (is-choreonoid) (equal (length key-av) 33))
    ;;     (setq key-av (concatenate float-vector key-av #f(-80 80 -80 80)))
    ;;   )
    (setq av-list-choreonoid-tmp nil)
    (dolist (n av-list)
      ;; (push (concatenate float-vector n #f(-80 80 -80 80)) av-list-choreonoid-tmp)
      (push (concatenate float-vector n finger-av) av-list-choreonoid-tmp)
      )
    (setq av-list-choreonoid (reverse (copy-seq av-list-choreonoid-tmp)))
    (setq av-list-seq nil)
    (cond ((is-choreonoid) (setq av-list-seq (copy-seq av-list-choreonoid)))
          ((not (is-choreonoid)) (setq av-list-seq (copy-seq av-list)))
          )
    ;; (setq av-list-inv (list))
    ;; (cond ((is-choreonoid)
    ;;        (dolist (n av-list-seq)
    ;;          (push
    ;;           (float-vector (aref n 6) (- (aref n 7)) (aref n 8) (aref n 9) (aref n 10) (aref n 11) (aref n 0) (- (aref n 1)) (aref n 2) (aref n 3) (aref n 4) (aref n 5) (aref n 12) (aref n 13) (aref n 14) (aref n 15) (aref n 16) (- (aref n 25)) (aref n 26) (- (aref n 27)) (- (aref n 28)) (aref n 29) (- (aref n 30)) (- (aref n 31)) (aref n 32) (- (aref n 17)) (aref n 18) (- (aref n 19)) (- (aref n 20)) (aref n 21) (- (aref n 22)) (- (aref n 23)) (aref n 24) (aref n 33) (aref n 34) (aref n 35) (aref n 36))
    ;;           av-list-inv)
    ;;          ))
    ;;       ((not (is-choreonoid))
    ;;        (dolist (n av-list-seq)
    ;;          (push
    ;;           (float-vector (aref n 6) (- (aref n 7)) (aref n 8) (aref n 9) (aref n 10) (aref n 11) (aref n 0) (- (aref n 1)) (aref n 2) (aref n 3) (aref n 4) (aref n 5) (aref n 12) (aref n 13) (aref n 14) (aref n 15) (aref n 16) (- (aref n 25)) (aref n 26) (- (aref n 27)) (- (aref n 28)) (aref n 29) (- (aref n 30)) (- (aref n 31)) (aref n 32) (- (aref n 17)) (aref n 18) (- (aref n 19)) (- (aref n 20)) (aref n 21) (- (aref n 22)) (- (aref n 23)) (aref n 24))
    ;;           av-list-inv))
    ;;         )
    ;;       )
    ;; ;; (dolist (n av-list-seq)
    ;; ;;   (push
    ;; ;;    (float-vector (aref n 6) (- (aref n 7)) (aref n 8) (aref n 9) (aref n 10) (aref n 11) (aref n 0) (- (aref n 1)) (aref n 2) (aref n 3) (aref n 4) (aref n 5) (aref n 12) (aref n 13) (aref n 14) (aref n 15) (aref n 16) (- (aref n 25)) (aref n 26) (- (aref n 27)) (- (aref n 28)) (aref n 29) (- (aref n 30)) (- (aref n 31)) (aref n 32) (- (aref n 17)) (aref n 18) (- (aref n 19)) (- (aref n 20)) (aref n 21) (- (aref n 22)) (- (aref n 23)) (aref n 24))
    ;; ;;    av-list-inv)
    ;; ;;   )
    ;; (setq av-list-inv (reverse (copy-seq av-list-inv)))
    
    ;; (setq root-local-zmp-mirror-tmp-list (mapcar #'(lambda (zz cc) (send cc :inverse-transform-vector zz)) zmp-list rc-list))
    ;; (setq root-local-zmp-mirror-list (list))
    ;; (dolist (n root-local-zmp-mirror-tmp-list)
    ;;   (push (float-vector (aref n 0) (- (aref n 1)) (aref n 2)) root-local-zmp-mirror-list)
    ;;   )
    ;; (setq root-local-zmp-mirror-list (reverse (copy-seq root-local-zmp-mirror-list)))
    
    ;; (setq wrench-mirror-list (list))
    ;; (dotimes (i (length time-list))
    ;;   (setq wrench-mirror-list (cons
    ;;                      (concatenate float-vector
    ;;                                   (scale -1 (concatenate float-vector (elt lff i) (elt lfm i) (elt rff i) (elt rfm i) (elt lhf i) (elt lhm i) (elt rhf i) (elt rhm i)))
    ;;                                   (elt lcrotch-wrench-list i) (elt rcrotch-wrench-list i)) wrench-mirror-list)))
    ;; (setq wrench-mirror-list (reverse wrench-mirror-list))

    ;; (setq cs-mirror-list (list))
    ;; (dolist (n cs-list)
    ;;   (push (float-vector (aref n 1) (aref n 0) (aref n 3) (aref n 2) (aref n 5) (aref n 4) (aref n 6) (aref n 7) (aref n 8) (aref n 9) (aref n 10) (aref n 11)) cs-mirror-list)
    ;;   )
    ;; (setq cs-mirror-list (reverse (copy-seq cs-mirror-list)))
    (print "next->initial-set-up")
    ))

(defun initial-pose()
  ;; (pose0)
  ;; (send *robot* :angle-vector key-av)
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  ;; (set-current-pose)
  ;; (unix:sleep 10)
  (send *robot* :angle-vector (car av-list-seq))
;;  (pose0)
  (send *robot* :fix-leg-to-coords (make-coords))
  (set-current-pose)
  (unix:sleep 10)
  (send *robot* :angle-vector (car av-list-seq))
  (send *ri* :angle-vector (send *robot* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector-sequence-full (list (car av-list-seq)) (list 2000) :fsensor-length fnum :root-coords (list (car rc-list)) :zmp (list (car zmp-list)) :wrench (list (car wrench-list)) :optional (list (car cs-list)))
  (send *ri* :wait-interpolation-seq)
  )

(defun initial-pose-mirror()
  ;; (pose0)
  ;; (send *robot* :angle-vector key-av)
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  ;; (set-current-pose)
  ;; (unix:sleep 10)
  (send *robot* :angle-vector (car av-list-inv))
;;  (pose0)
  (send *robot* :fix-leg-to-coords (make-coords))
  (set-current-pose)
  (unix:sleep 10)
  (send *robot* :angle-vector (car av-list-inv))
  (send *ri* :angle-vector (send *robot* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector-sequence-full (list (car av-list-inv)) (list (car tm-list-for-seq)) :fsensor-length fnum :root-local-zmp (list (car root-local-zmp-mirror-list)) :wrench (list (car wrench-mirror-list)) :optional (list (car cs-mirror-list)))
  (send *ri* :wait-interpolation-seq)
  )
;; ;; comment these out without time using for test
;; (defun simint::interpolate-callback
;;     (tm &optional av tu bl (dtm eus_anim::*ref-cdt*))
;;   (setq av (scale (deg2rad 1) #f(-0.00125 0.000429 -90.0 100.0 -41.2631 -0.059189 -0.001299 0.048607 -90.0 100.0 -41.3527 -0.006105 -0.041261 0.0 0.000185 -5.729886e-07 0.002006 32.9715 71.6886 -15.8261 13.8257 -74.1106 -22.626 -8.28847 -1.42803 -33.054 71.7556 15.8261 -14.1607 -74.1042 22.9992 8.64372 -1.62462)));;(send *robot* :reset-pose);;posen #f(-0.00125 0.000429 -90.0 100.0 -41.2631 -0.059189 -0.001299 0.048607 -90.0 100.0 -41.3527 -0.006105 -0.041261 0.0 0.000185 -5.729886e-07 0.002006 32.9715 71.6886 -15.8261 13.8257 -74.1106 -22.626 -8.28847 -1.42803 -33.054 71.7556 15.8261 -14.1607 -74.1042 22.9992 8.64372 -1.62462) ;;pose0 #f(-0.00125 0.000429 -30.0457 71.5588 -41.2631 -0.059189 -0.001299 0.048607 -30.1077 71.5897 -41.3527 -0.006105 -0.041261 0.0 0.000185 -5.729886e-07 0.002006 -5.0647 60.4568 -25.3647 15.6572 -25.7493 -10.5389 5.87984 -37.3349 5.09398 60.2026 25.2667 -15.969 -25.2551 10.5693 -5.71767 -37.6718) ;;pose0? #f(-0.001501 -0.032394 -29.9482 71.7159 -41.2113 -0.097682 -0.001129 0.052928 -30.1463 71.7875 -41.4917 -0.000793 -0.105983 -0.035735 0.000422 4.032923e-06 0.005655 -7.2413 57.5368 -26.644 16.737 -23.0949 -9.40413 6.47505 -37.6256 7.30871 57.2005 26.5497 -17.0565 -22.6392 9.41936 -6.31805 -37.9526)
;;   (eus_anim::set-position-control-reference av dtm)
;;   (eus_anim::set-torque-control-reference (scale 0 av))
;;   (eus_anim::set-blend-control-reference (scale 0 av));;この辺悪そうjaxon_simint.lでは呼ばれてるけど大丈夫か
;;   )

;; (defun simint::init-target-pose nil
;;   ;; (send *robot* :reset-pose)
;;   ;; (send *robot* :newcoords (make-coords))
;;   ;; (send *robot* :fix-leg-to-coords
;;   ;;       (make-coords :pos #F(-500 0 10)) :both)
;;   ;; (pose0)
;;   ;; (send *robot* :arms :move-end-pos (float-vector 10 0 0))
;;   ;; (send *robot* :translate #f(0 0 10))
;;   (poseN)
;;   )
;; (defun test-animation ()
;;   (setq simint::*log?* (* 100 10))
;;   (simint::start-simulation))



(defun seq-full-initial-info ()
    (send *ri* :angle-vector-sequence-full (list (car av-list-seq)) (list (car tm-list-for-seq)) :fsensor-length fnum :root-coords (list (car rc-list)) :zmp (list (car zmp-list)) :wrench (list (car wrench-list)) :optional (list (car cs-list)))
  (send *ri* :wait-interpolation-seq)
)

(defun execute-motion (&key (real t))

  (send *ri* :set-log-maxlength (* 500 45))
  (send *ri* :start-log) ;;;;;;;;;;;;;;;;;;;;;;;;;control tools 入れて
  ;; (setq av-list-choreonoid-tmp nil)
  ;; (dolist (n av-list)
  ;;   (push (concatenate float-vector n #f(-80 80 -80 80)) av-list-choreonoid-tmp)
  ;;   )
  ;; (setq av-list-choreonoid (reverse (copy-seq av-list-choreonoid-tmp)))
  ;; (setq av-list-seq nil)
  ;; (cond ((is-choreonoid) (setq av-list-seq (copy-seq av-list-choreonoid)))
  ;;       ((not (is-choreonoid)) (setq av-list-seq (copy-seq av-list)))
  ;;       )
  (when real
    (warning-message 3 "interpoltaion-mode linear~%")
    (send *ri* :set-interpolation-mode :linear)
    (warning-message 3 "move~%")
    (send *ri* :angle-vector-sequence-full av-list-seq tm-list-for-seq :fsensor-length fnum :root-coords rc-list :zmp zmp-list :wrench wrench-list :optional cs-list);;attention for fnum!! determined in get-param  wrenchの順番。大丈夫そうな雰囲気だったけど
    (warning-message 3 "wait~%")
    (send *ri* :wait-interpolation-seq)
    (unix:sleep 10)
    (warning-message 3 "interpoltaion-mode hoffarbib~%")
    (send *ri* :set-interpolation-mode :hoffarbib)
    (save-log :fname "sit-again")
    )
  )


(defun execute-motion3 (&key (real t))

  (send *ri* :set-log-maxlength (* 500 45))
  (send *ri* :start-log) ;;;;;;;;;;;;;;;;;;;;;;;;;control tools 入れて
  ;; (setq av-list-choreonoid-tmp nil)
  ;; (dolist (n av-list)
  ;;   (push (concatenate float-vector n #f(-80 80 -80 80)) av-list-choreonoid-tmp)
  ;;   )
  ;; (setq av-list-choreonoid (reverse (copy-seq av-list-choreonoid-tmp)))
  ;; (setq av-list-seq nil)
  ;; (cond ((is-choreonoid) (setq av-list-seq (copy-seq av-list-choreonoid)))
  ;;       ((not (is-choreonoid)) (setq av-list-seq (copy-seq av-list)))
  ;;       )
  (when real
    (warning-message 3 "interpoltaion-mode linear~%")
    (send *ri* :set-interpolation-mode :linear)
    (warning-message 3 "move~%")
    (send *ri* :angle-vector-sequence-full av-list-seq tm-list-for-seq :fsensor-length fnum :root-coords rc-list :zmp zmp-list :wrench wrench-list :optional cs-list);;attention for fnum!! determined in get-param  wrenchの順番。大丈夫そうな雰囲気だったけど
    (warning-message 3 "wait~%")
    (send *ri* :wait-interpolation-seq)
    (send *ri* :angle-vector-sequence-full av-list-seq tm-list-for-seq :fsensor-length fnum :root-coords rc-list :zmp zmp-list :wrench wrench-list :optional cs-list);;attention for fnum!! determined in get-param  wrenchの順番。大丈夫そうな雰囲気だったけど
    (warning-message 3 "wait~%")
    (send *ri* :wait-interpolation-seq)
    (send *ri* :angle-vector-sequence-full av-list-seq tm-list-for-seq :fsensor-length fnum :root-coords rc-list :zmp zmp-list :wrench wrench-list :optional cs-list);;attention for fnum!! determined in get-param  wrenchの順番。大丈夫そうな雰囲気だったけど
    (warning-message 3 "wait~%")
    (send *ri* :wait-interpolation-seq)
    (warning-message 3 "interpoltaion-mode hoffarbib~%")
    (send *ri* :set-interpolation-mode :hoffarbib)
    (save-log :fname "sit-again*3")
    )
  )


(defun execute-motion-mirror (&key (real t))

  (send *ri* :set-log-maxlength (* 500 45))
  (send *ri* :start-log) ;;;;;;;;;;;;;;;;;;;;;;;;;control tools 入れて
  ;; (setq av-list-choreonoid-tmp nil)
  ;; (dolist (n av-list)
  ;;   (push (concatenate float-vector n #f(-80 80 -80 80)) av-list-choreonoid-tmp)
  ;;   )
  ;; (setq av-list-choreonoid (reverse (copy-seq av-list-choreonoid-tmp)))
  ;; (setq av-list-seq nil)
  ;; (cond ((is-choreonoid) (setq av-list-seq (copy-seq av-list-choreonoid)))
  ;;       ((not (is-choreonoid)) (setq av-list-seq (copy-seq av-list)))
  ;;       )
  (when real
    (warning-message 3 "interpoltaion-mode linear~%")
    (send *ri* :set-interpolation-mode :linear)
    (warning-message 3 "move~%")
    (send *ri* :angle-vector-sequence-full av-list-inv tm-list-for-seq :fsensor-length fnum :root-local-zmp root-local-zmp-mirror-list :wrench wrench-mirror-list :optional cs-mirror-list);;attention for fnum!! determined in get-param  wrenchの順番。大丈夫そうな雰囲気だったけど
    (warning-message 3 "wait~%")
    ;;    (send *ri* :wait-interpolation-seq)
    (unix:sleep 10)
    (warning-message 3 "interpoltaion-mode hoffarbib~%")
    (send *ri* :set-interpolation-mode :hoffarbib)
    (save-log :fname "sit-again")
    )
  )

(defun execute-motion-default-and-inversion (&key (real t))

  (send *ri* :set-log-maxlength (* 500 45))
  (send *ri* :start-log) ;;;;;;;;;;;;;;;;;;;;;;;;;control tools 入れて
  ;; (setq av-list-choreonoid-tmp nil)
  ;; (dolist (n av-list)
  ;;   (push (concatenate float-vector n #f(-80 80 -80 80)) av-list-choreonoid-tmp)
  ;;   )
  ;; (setq av-list-choreonoid (reverse (copy-seq av-list-choreonoid-tmp)))
  ;; (setq av-list-seq nil)
  ;; (cond ((is-choreonoid) (setq av-list-seq (copy-seq av-list-choreonoid)))
  ;;       ((not (is-choreonoid)) (setq av-list-seq (copy-seq av-list)))
  ;;       )
  (when real
    (warning-message 3 "interpoltaion-mode linear~%")
    (send *ri* :set-interpolation-mode :linear)
    (warning-message 3 "move~%")
    (send *ri* :angle-vector-sequence-full av-list-seq tm-list-for-seq :fsensor-length fnum :root-coords rc-list :zmp zmp-list :wrench wrench-list :optional cs-list);;attention for fnum!! determined in get-param  wrenchの順番。大丈夫そうな雰囲気だったけど
    (warning-message 3 "wait~%")
    (send *ri* :wait-interpolation-seq)
    (send *ri* :angle-vector-sequence-full av-list-seq tm-list-for-seq :fsensor-length fnum :root-coords rc-list :zmp zmp-list :wrench wrench-list :optional cs-list);;attention for fnum!! determined in get-param  wrenchの順番。大丈夫そうな雰囲気だったけど
    (warning-message 3 "wait~%")
    (send *ri* :wait-interpolation-seq)
    (warning-message 3 "interpoltaion-mode hoffarbib~%")
    (send *ri* :set-interpolation-mode :hoffarbib)
    (save-log :fname "sit-again*3")
    )
  )


;; (defun execute-seq (&key (real t))
;;   (send *ri* :set-log-maxlength (* 500 45))
;;   (send *ri* :start-log) ;;;;;;;;;;;;;;;;;;;;;;;;;control tools 入れて
;;   (setq av-list-choreonoid-tmp nil)
;;   (dolist (n av-list)
;;     (push (concatenate float-vector n #f(-80 80 -80 80)) av-list-choreonoid-tmp)
;;     )
;;   (setq av-list-choreonoid (reverse (copy-seq av-list-choreonoid-tmp)))
;;   (if (is-choreonoid)
;;       (setq av-list (copy-seq av-list-choreonoid))
;;     )
;;   (when real
;;     (send *ri* :angle-vector-sequence-full (list (car av-list)) (list tm))
;;     (send *ri* :wait-interpolation-seq)
;;     ;; (unix:usleep (* 1000 tm))
;;     )

;;       (when (y-or-n-p "debug::finish the first pose. ready to move the rest?")
;;         )
;;       (warning-message 3 "interpoltaion-mode linear~%")
;;       (send *ri* :set-interpolation-mode :linear)
;;       (warning-message 3 "move~%")
;;       (send *ri* :angle-vector-sequence-full av-list tm-list-for-seq)
;;       (send *ri* :wait-interpolation-seq)
;;       (warning-message 3 "interpoltaion-mode hoffarbib~%")
;;       (send *ri* :set-interpolation-mode :hoffarbib)
;;       (save-log :fname "walk-up")
;;       )
;;   )






;;;;;;;元のコードへの書き換え
;(send buf :put :rc (cons (make-coords :pos (copy-seq p) :rot (copy-matrix R)) (send buf :get :rc))) ;;野田さん11/3メールより引用、ルートリンクの位置姿勢を記録するため
;;nlopt-interpolation.l *logger-buf*につける


;;nlopt-interpolation.l にてcontact state取るためにかなり書き込んでる（全部コメント入れてる

;; (defun auto ()
;;   (simint::jbf-sample)
;;   (warning-message 3 "finish simulation~%")
;;   (get-param)
;;   (warning-message 3 "get param~%")
;;   (jaxoninit)
;;   (send *ri* :go-pos -0.2 0 0)
;;   (send *ri* :go-pos 0 0 180)
;;   (send *ri* :go-pos -0.29 0 0)
;;   (warning-message 3 "came to start position~%")
;;   (initial-set-up)
;;   (warning-message 3 "move to inital-pose")
;;   (initial-pose)
;;   (warning-message 3 "start motion~%")
;;   (execute-motion)
;;   )


;; (defun auto-set ()
;;   (simint::jbf-sample)
;;   (warning-message 3 "finish simulation~%")
;;   (get-param)
;;   (warning-message 3 "get param~%")
;;   (y-or-n-p "choreonoid ready?") 
;;   (jaxoninit)
;;   (initial-set-up)
;;   (make-key-pose)
;;   )
