;; Auto-generated. Do not edit!


(when (boundp 'swarm_controller::velocity_msg)
  (if (not (find-package "SWARM_CONTROLLER"))
    (make-package "SWARM_CONTROLLER"))
  (shadow 'velocity_msg (find-package "SWARM_CONTROLLER")))
(unless (find-package "SWARM_CONTROLLER::VELOCITY_MSG")
  (make-package "SWARM_CONTROLLER::VELOCITY_MSG"))

(in-package "ROS")
;;//! \htmlinclude velocity_msg.msg.html


(defclass swarm_controller::velocity_msg
  :super ros::object
  :slots (_velocity _slot_id ))

(defmethod swarm_controller::velocity_msg
  (:init
   (&key
    ((:velocity __velocity) 0.0)
    ((:slot_id __slot_id) 0)
    )
   (send-super :init)
   (setq _velocity (float __velocity))
   (setq _slot_id (round __slot_id))
   self)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:slot_id
   (&optional __slot_id)
   (if __slot_id (setq _slot_id __slot_id)) _slot_id)
  (:serialization-length
   ()
   (+
    ;; float32 _velocity
    4
    ;; int32 _slot_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _slot_id
       (write-long _slot_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _velocity
     (setq _velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _slot_id
     (setq _slot_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get swarm_controller::velocity_msg :md5sum-) "bd5e463c1daf6ea79e9b4940515d2e74")
(setf (get swarm_controller::velocity_msg :datatype-) "swarm_controller/velocity_msg")
(setf (get swarm_controller::velocity_msg :definition-)
      "float32 velocity
int32 slot_id
")



(provide :swarm_controller/velocity_msg "bd5e463c1daf6ea79e9b4940515d2e74")


