; Auto-generated. Do not edit!


(cl:in-package movemaster_msg-msg)


;//! \htmlinclude status.msg.html

(cl:defclass <status> (roslisp-msg-protocol:ros-message)
  ((joint
    :reader joint
    :initarg :joint
    :type cl:string
    :initform "")
   (setpoint
    :reader setpoint
    :initarg :setpoint
    :type cl:float
    :initform 0.0)
   (pulse_count
    :reader pulse_count
    :initarg :pulse_count
    :type cl:float
    :initform 0.0)
   (error
    :reader error
    :initarg :error
    :type cl:float
    :initform 0.0)
   (output
    :reader output
    :initarg :output
    :type cl:float
    :initform 0.0)
   (control_loop
    :reader control_loop
    :initarg :control_loop
    :type cl:float
    :initform 0.0)
   (IsDone
    :reader IsDone
    :initarg :IsDone
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass status (<status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name movemaster_msg-msg:<status> is deprecated: use movemaster_msg-msg:status instead.")))

(cl:ensure-generic-function 'joint-val :lambda-list '(m))
(cl:defmethod joint-val ((m <status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader movemaster_msg-msg:joint-val is deprecated.  Use movemaster_msg-msg:joint instead.")
  (joint m))

(cl:ensure-generic-function 'setpoint-val :lambda-list '(m))
(cl:defmethod setpoint-val ((m <status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader movemaster_msg-msg:setpoint-val is deprecated.  Use movemaster_msg-msg:setpoint instead.")
  (setpoint m))

(cl:ensure-generic-function 'pulse_count-val :lambda-list '(m))
(cl:defmethod pulse_count-val ((m <status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader movemaster_msg-msg:pulse_count-val is deprecated.  Use movemaster_msg-msg:pulse_count instead.")
  (pulse_count m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader movemaster_msg-msg:error-val is deprecated.  Use movemaster_msg-msg:error instead.")
  (error m))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader movemaster_msg-msg:output-val is deprecated.  Use movemaster_msg-msg:output instead.")
  (output m))

(cl:ensure-generic-function 'control_loop-val :lambda-list '(m))
(cl:defmethod control_loop-val ((m <status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader movemaster_msg-msg:control_loop-val is deprecated.  Use movemaster_msg-msg:control_loop instead.")
  (control_loop m))

(cl:ensure-generic-function 'IsDone-val :lambda-list '(m))
(cl:defmethod IsDone-val ((m <status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader movemaster_msg-msg:IsDone-val is deprecated.  Use movemaster_msg-msg:IsDone instead.")
  (IsDone m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <status>) ostream)
  "Serializes a message object of type '<status>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'joint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'joint))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'setpoint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pulse_count))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'control_loop))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'IsDone) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <status>) istream)
  "Deserializes a message object of type '<status>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'joint) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'joint) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'setpoint) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pulse_count) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'output) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'control_loop) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'IsDone) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<status>)))
  "Returns string type for a message object of type '<status>"
  "movemaster_msg/status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'status)))
  "Returns string type for a message object of type 'status"
  "movemaster_msg/status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<status>)))
  "Returns md5sum for a message object of type '<status>"
  "283025d1e58370fa7a43c23c572de06b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'status)))
  "Returns md5sum for a message object of type 'status"
  "283025d1e58370fa7a43c23c572de06b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<status>)))
  "Returns full string definition for message of type '<status>"
  (cl:format cl:nil "string joint~%float32 setpoint~%float32 pulse_count~%float32 error~%float32 output~%float32 control_loop~%bool IsDone~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'status)))
  "Returns full string definition for message of type 'status"
  (cl:format cl:nil "string joint~%float32 setpoint~%float32 pulse_count~%float32 error~%float32 output~%float32 control_loop~%bool IsDone~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <status>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'joint))
     4
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <status>))
  "Converts a ROS message object to a list"
  (cl:list 'status
    (cl:cons ':joint (joint msg))
    (cl:cons ':setpoint (setpoint msg))
    (cl:cons ':pulse_count (pulse_count msg))
    (cl:cons ':error (error msg))
    (cl:cons ':output (output msg))
    (cl:cons ':control_loop (control_loop msg))
    (cl:cons ':IsDone (IsDone msg))
))
