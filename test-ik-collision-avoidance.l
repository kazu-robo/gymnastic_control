(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(setq *robot* (jaxon_red))
(objects (list *robot*))

(warn "Test for ik collision avoidance~%Press Enter for continue~%")
(do-until-key (unix:usleep 100) (x::window-main-one))

(warn "without collision avoidance~%")
(send *robot* :reset-manip-pose)
(send *robot* :rarm :inverse-kinematics (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #f(-80 400 100))
      :rotation-axis nil
      :debug-view :no-message
      :collision-avoidance-link-pair nil
      :avoid-collision-distance 200
      :stop 200)

(warn "Press Enter for continue~%")
(do-until-key (unix:usleep 100) (x::window-main-one))
(warn "with collision avoidance~%")
(send *robot* :reset-manip-pose)
(send *robot* :rarm :inverse-kinematics (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #f(-80 400 100))
      :rotation-axis nil
      :debug-view :no-message
      :collision-avoidance-link-pair (list (list (send *robot* :rarm :wrist-p :child-link) (send *robot* :torso :waist-p :child-link )))
      :avoid-collision-null-gain 10
      :avoid-collision-joint-gain 0
      :avoid-collision-distance 200
      :stop 200)
