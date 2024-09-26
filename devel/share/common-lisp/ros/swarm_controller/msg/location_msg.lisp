; Auto-generated. Do not edit!


(cl:in-package swarm_controller-msg)


;//! \htmlinclude location_msg.msg.html

(cl:defclass <location_msg> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0))
)

(cl:defclass location_msg (<location_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <location_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'location_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name swarm_controller-msg:<location_msg> is deprecated: use swarm_controller-msg:location_msg instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <location_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swarm_controller-msg:x-val is deprecated.  Use swarm_controller-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <location_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swarm_controller-msg:y-val is deprecated.  Use swarm_controller-msg:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <location_msg>) ostream)
  "Serializes a message object of type '<location_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <location_msg>) istream)
  "Deserializes a message object of type '<location_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<location_msg>)))
  "Returns string type for a message object of type '<location_msg>"
  "swarm_controller/location_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'location_msg)))
  "Returns string type for a message object of type 'location_msg"
  "swarm_controller/location_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<location_msg>)))
  "Returns md5sum for a message object of type '<location_msg>"
  "ff8d7d66dd3e4b731ef14a45d38888b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'location_msg)))
  "Returns md5sum for a message object of type 'location_msg"
  "ff8d7d66dd3e4b731ef14a45d38888b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<location_msg>)))
  "Returns full string definition for message of type '<location_msg>"
  (cl:format cl:nil "float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'location_msg)))
  "Returns full string definition for message of type 'location_msg"
  (cl:format cl:nil "float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <location_msg>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <location_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'location_msg
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
