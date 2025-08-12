; Auto-generated. Do not edit!


(cl:in-package movemaster_msg-msg)


;//! \htmlinclude setpoint.msg.html

(cl:defclass <setpoint> (roslisp-msg-protocol:ros-message)
  ((set_1
    :reader set_1
    :initarg :set_1
    :type cl:float
    :initform 0.0)
   (set_2
    :reader set_2
    :initarg :set_2
    :type cl:float
    :initform 0.0)
   (set_3
    :reader set_3
    :initarg :set_3
    :type cl:float
    :initform 0.0)
   (set_4
    :reader set_4
    :initarg :set_4
    :type cl:float
    :initform 0.0)
   (set_5
    :reader set_5
    :initarg :set_5
    :type cl:float
    :initform 0.0)
   (set_GRIP
    :reader set_GRIP
    :initarg :set_GRIP
    :type cl:boolean
    :initform cl:nil)
   (emergency_stop
    :reader emergency_stop
    :initarg :emergency_stop
    :type cl:boolean
    :initform cl:nil)
   (GoHome
    :reader GoHome
    :initarg :GoHome
    :type cl:fixnum
    :initform 0))
)

(cl:defclass setpoint (<setpoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setpoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setpoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name movemaster_msg-msg:<setpoint> is deprecated: use movemaster_msg-msg:setpoint instead.")))

(cl:ensure-generic-function 'set_1-val :lambda-list '(m))
(cl:defmethod set_1-val ((m <setpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader movemaster_msg-msg:set_1-val is deprecated.  Use movemaster_msg-msg:set_1 instead.")
  (set_1 m))

(cl:ensure-generic-function 'set_2-val :lambda-list '(m))
(cl:defmethod set_2-val ((m <setpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader movemaster_msg-msg:set_2-val is deprecated.  Use movemaster_msg-msg:set_2 instead.")
  (set_2 m))

(cl:ensure-generic-function 'set_3-val :lambda-list '(m))
(cl:defmethod set_3-val ((m <setpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader movemaster_msg-msg:set_3-val is deprecated.  Use movemaster_msg-msg:set_3 instead.")
  (set_3 m))

(cl:ensure-generic-function 'set_4-val :lambda-list '(m))
(cl:defmethod set_4-val ((m <setpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader movemaster_msg-msg:set_4-val is deprecated.  Use movemaster_msg-msg:set_4 instead.")
  (set_4 m))

(cl:ensure-generic-function 'set_5-val :lambda-list '(m))
(cl:defmethod set_5-val ((m <setpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader movemaster_msg-msg:set_5-val is deprecated.  Use movemaster_msg-msg:set_5 instead.")
  (set_5 m))

(cl:ensure-generic-function 'set_GRIP-val :lambda-list '(m))
(cl:defmethod set_GRIP-val ((m <setpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader movemaster_msg-msg:set_GRIP-val is deprecated.  Use movemaster_msg-msg:set_GRIP instead.")
  (set_GRIP m))

(cl:ensure-generic-function 'emergency_stop-val :lambda-list '(m))
(cl:defmethod emergency_stop-val ((m <setpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader movemaster_msg-msg:emergency_stop-val is deprecated.  Use movemaster_msg-msg:emergency_stop instead.")
  (emergency_stop m))

(cl:ensure-generic-function 'GoHome-val :lambda-list '(m))
(cl:defmethod GoHome-val ((m <setpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader movemaster_msg-msg:GoHome-val is deprecated.  Use movemaster_msg-msg:GoHome instead.")
  (GoHome m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setpoint>) ostream)
  "Serializes a message object of type '<setpoint>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'set_1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'set_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'set_3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'set_4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'set_5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'set_GRIP) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'emergency_stop) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'GoHome)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setpoint>) istream)
  "Deserializes a message object of type '<setpoint>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'set_1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'set_2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'set_3) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'set_4) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'set_5) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'set_GRIP) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'emergency_stop) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'GoHome) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setpoint>)))
  "Returns string type for a message object of type '<setpoint>"
  "movemaster_msg/setpoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setpoint)))
  "Returns string type for a message object of type 'setpoint"
  "movemaster_msg/setpoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setpoint>)))
  "Returns md5sum for a message object of type '<setpoint>"
  "aaa39b4215432c08ac9b36247ded9610")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setpoint)))
  "Returns md5sum for a message object of type 'setpoint"
  "aaa39b4215432c08ac9b36247ded9610")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setpoint>)))
  "Returns full string definition for message of type '<setpoint>"
  (cl:format cl:nil "float32 set_1~%float32 set_2~%float32 set_3~%float32 set_4~%float32 set_5~%bool 	set_GRIP~%bool 	emergency_stop~%int8 	GoHome~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setpoint)))
  "Returns full string definition for message of type 'setpoint"
  (cl:format cl:nil "float32 set_1~%float32 set_2~%float32 set_3~%float32 set_4~%float32 set_5~%bool 	set_GRIP~%bool 	emergency_stop~%int8 	GoHome~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setpoint>))
  (cl:+ 0
     4
     4
     4
     4
     4
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setpoint>))
  "Converts a ROS message object to a list"
  (cl:list 'setpoint
    (cl:cons ':set_1 (set_1 msg))
    (cl:cons ':set_2 (set_2 msg))
    (cl:cons ':set_3 (set_3 msg))
    (cl:cons ':set_4 (set_4 msg))
    (cl:cons ':set_5 (set_5 msg))
    (cl:cons ':set_GRIP (set_GRIP msg))
    (cl:cons ':emergency_stop (emergency_stop msg))
    (cl:cons ':GoHome (GoHome msg))
))
