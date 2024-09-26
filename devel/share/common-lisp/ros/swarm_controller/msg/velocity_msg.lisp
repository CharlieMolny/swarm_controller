; Auto-generated. Do not edit!


(cl:in-package swarm_controller-msg)


;//! \htmlinclude velocity_msg.msg.html

(cl:defclass <velocity_msg> (roslisp-msg-protocol:ros-message)
  ((velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0)
   (slot_id
    :reader slot_id
    :initarg :slot_id
    :type cl:integer
    :initform 0))
)

(cl:defclass velocity_msg (<velocity_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <velocity_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'velocity_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name swarm_controller-msg:<velocity_msg> is deprecated: use swarm_controller-msg:velocity_msg instead.")))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <velocity_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swarm_controller-msg:velocity-val is deprecated.  Use swarm_controller-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'slot_id-val :lambda-list '(m))
(cl:defmethod slot_id-val ((m <velocity_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swarm_controller-msg:slot_id-val is deprecated.  Use swarm_controller-msg:slot_id instead.")
  (slot_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <velocity_msg>) ostream)
  "Serializes a message object of type '<velocity_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'slot_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <velocity_msg>) istream)
  "Deserializes a message object of type '<velocity_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'slot_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<velocity_msg>)))
  "Returns string type for a message object of type '<velocity_msg>"
  "swarm_controller/velocity_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'velocity_msg)))
  "Returns string type for a message object of type 'velocity_msg"
  "swarm_controller/velocity_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<velocity_msg>)))
  "Returns md5sum for a message object of type '<velocity_msg>"
  "bd5e463c1daf6ea79e9b4940515d2e74")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'velocity_msg)))
  "Returns md5sum for a message object of type 'velocity_msg"
  "bd5e463c1daf6ea79e9b4940515d2e74")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<velocity_msg>)))
  "Returns full string definition for message of type '<velocity_msg>"
  (cl:format cl:nil "float32 velocity~%int32 slot_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'velocity_msg)))
  "Returns full string definition for message of type 'velocity_msg"
  (cl:format cl:nil "float32 velocity~%int32 slot_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <velocity_msg>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <velocity_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'velocity_msg
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':slot_id (slot_id msg))
))
