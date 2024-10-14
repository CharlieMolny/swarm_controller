
(cl:in-package :asdf)

(defsystem "swarm_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "WheelsCmdStamped" :depends-on ("_package_WheelsCmdStamped"))
    (:file "_package_WheelsCmdStamped" :depends-on ("_package"))
    (:file "velocity_msg" :depends-on ("_package_velocity_msg"))
    (:file "_package_velocity_msg" :depends-on ("_package"))
  ))