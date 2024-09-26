
(cl:in-package :asdf)

(defsystem "swarm_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "location_msg" :depends-on ("_package_location_msg"))
    (:file "_package_location_msg" :depends-on ("_package"))
    (:file "velocity_msg" :depends-on ("_package_velocity_msg"))
    (:file "_package_velocity_msg" :depends-on ("_package"))
  ))