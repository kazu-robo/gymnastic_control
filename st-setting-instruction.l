;;STの設定を大きく変える場合は可能ならばシミュレータで確認してから実機に移したい

(send *ri* :set-st-param :emergency-check-mode 0)
;;STのemergency-modeをoffにする。通常時は有用だが特殊な動作の際にはoffにしておいたほうがいい。

(send *ri* :set-st-param :is-ik-enable (list t t t t))
;;STでIKを解く四肢のon/off。全てtにすると腕についてもIKを解くようになる。

(send *ri* :set-st-param :is-feedback-control-enable (list t t t t))
;;STで力センサを使ったフィードバック制御を有効にする四肢のon/off。全てtにすると手も目標力に対してダンピング制御を行うようになる。

(send *ri* :set-st-param :eefm-use-force-difference-control nil)
;;脚の目標力を左右の足裏反力のdiffで制御するかどうか。

(send *ri* :set-st-param :st-algorithm :eefmqpcop2)
;;STのアルゴリズムをEEFMQPCOP2に(AutoBalancerを使わないような場合。使う場合はEEFMQPCOPでいいと思われる)

(send *ri* :set-st-param :is-zmp-calc-enable (list t t t t))
;;腕も込みでZMPを計算する。この場合送るzmpの値も両手両足の接触力から算出したzmpにする必要がある。
;;両足の接触力のみから算出したzmpの場合は、(list t t nil nil)となる

(send *ri* :set-st-param :eefm-pos-damping-gain hogehoge)
(send *ri* :set-st-param :eefm-rot-damping-gain hogehoge)
;;手足のダンピング制御のゲインを設定。以下は一例
(send *ri* :set-st-param :eefm-pos-damping-gain
      (list (float-vector 1.75e5 1.75e5 7000)
            (float-vector 1.75e5 1.75e5 7000)
            (float-vector 500 400 500)
            (float-vector 500 400 500)))
(send *ri* :set-st-param :eefm-rot-damping-gain
      (list (float-vector 50 50 50)
            (float-vector 50 50 50)
            (float-vector 22 22 22)
            (float-vector 22 22 22)))

(send *ri* :set-st-param :eefm-pos-compensation-limit (float-vector 0.025 0.025 0.025 0.025))
;;手足の可動限界[m]。この可動範囲の内部でダンピング制御を行う。必要なければ、わざわざ設定しなくてもよい。

(send *ri* :set-interpolation-mode :linear)
;;送られた関節角軌道に対して躍度最小補間ではなく、線形補間する。躍度最小補間だと振動する場合がある。

;;STの設定については以下を参照するとよい
;;四脚の場合
;;euslib/demo/kuroiwa/master-thesis/IREX/memo.l
;;euslib/demo/kuroiwa/master-thesis/IREX/4legs-walking.l
;;二脚＋手を使うという感じの場合
;;euslib/demo/nozawa/multicontact_motion_planner/test-multicontact-motion-planner-real.l

;;STが機能するためにはZMPの指令値などが必要。通常はAutoBalancerが送る。
;;AutoBalancerを使わない場合はangle-vector-sequence-fullやload-patternなどを用いる

;;angle-vector-sequence-fullの場合
;;hrpsys_ros_bridge/test/hrpsys-samples/samplerobot-sequence-player.lを参照
(send *ri* :angle-vector-sequence-full av-list dt-list
      :root-coords root-coords-list
      :zmp zmp-list
      :wrench wrench-list 
      :optional optional-list)
(send *ri* :wait-interpolation-seq)
;;av-list: angle-vectorのlist
;;dt-list: angle-vectorの間を遷移していく時間のlist
;;root-coords-list: root-coordsのlist
;;zmp-list: zmpのlist
;;wrench-list: 手足の接触wrenchのlist
;;optional: contact state と swing-support-remain-timeのlist。四脚接地の場合は #f(1 1 1 1 1 1 1 1) でいいと思われる。
