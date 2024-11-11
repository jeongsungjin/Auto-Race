; Auto-generated. Do not edit!


(cl:in-package sign_slowdown-msg)


;//! \htmlinclude Drive_command.msg.html

(cl:defclass <Drive_command> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0)
   (flag
    :reader flag
    :initarg :flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Drive_command (<Drive_command>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Drive_command>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Drive_command)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sign_slowdown-msg:<Drive_command> is deprecated: use sign_slowdown-msg:Drive_command instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <Drive_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sign_slowdown-msg:speed-val is deprecated.  Use sign_slowdown-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <Drive_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sign_slowdown-msg:angle-val is deprecated.  Use sign_slowdown-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <Drive_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sign_slowdown-msg:flag-val is deprecated.  Use sign_slowdown-msg:flag instead.")
  (flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Drive_command>) ostream)
  "Serializes a message object of type '<Drive_command>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Drive_command>) istream)
  "Deserializes a message object of type '<Drive_command>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Drive_command>)))
  "Returns string type for a message object of type '<Drive_command>"
  "sign_slowdown/Drive_command")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Drive_command)))
  "Returns string type for a message object of type 'Drive_command"
  "sign_slowdown/Drive_command")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Drive_command>)))
  "Returns md5sum for a message object of type '<Drive_command>"
  "95ff01497d9197f16d490d6e129fd2cd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Drive_command)))
  "Returns md5sum for a message object of type 'Drive_command"
  "95ff01497d9197f16d490d6e129fd2cd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Drive_command>)))
  "Returns full string definition for message of type '<Drive_command>"
  (cl:format cl:nil "float32 speed~%float32 angle~%bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Drive_command)))
  "Returns full string definition for message of type 'Drive_command"
  (cl:format cl:nil "float32 speed~%float32 angle~%bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Drive_command>))
  (cl:+ 0
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Drive_command>))
  "Converts a ROS message object to a list"
  (cl:list 'Drive_command
    (cl:cons ':speed (speed msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':flag (flag msg))
))
