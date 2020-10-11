; Auto-generated. Do not edit!


(cl:in-package franka_msgs-msg)


;//! \htmlinclude servoj.msg.html

(cl:defclass <servoj> (roslisp-msg-protocol:ros-message)
  ((keepalive
    :reader keepalive
    :initarg :keepalive
    :type cl:integer
    :initform 0)
   (cmd_q
    :reader cmd_q
    :initarg :cmd_q
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass servoj (<servoj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <servoj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'servoj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_msgs-msg:<servoj> is deprecated: use franka_msgs-msg:servoj instead.")))

(cl:ensure-generic-function 'keepalive-val :lambda-list '(m))
(cl:defmethod keepalive-val ((m <servoj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_msgs-msg:keepalive-val is deprecated.  Use franka_msgs-msg:keepalive instead.")
  (keepalive m))

(cl:ensure-generic-function 'cmd_q-val :lambda-list '(m))
(cl:defmethod cmd_q-val ((m <servoj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_msgs-msg:cmd_q-val is deprecated.  Use franka_msgs-msg:cmd_q instead.")
  (cmd_q m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <servoj>) ostream)
  "Serializes a message object of type '<servoj>"
  (cl:let* ((signed (cl:slot-value msg 'keepalive)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cmd_q))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'cmd_q))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <servoj>) istream)
  "Deserializes a message object of type '<servoj>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'keepalive) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cmd_q) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cmd_q)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<servoj>)))
  "Returns string type for a message object of type '<servoj>"
  "franka_msgs/servoj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'servoj)))
  "Returns string type for a message object of type 'servoj"
  "franka_msgs/servoj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<servoj>)))
  "Returns md5sum for a message object of type '<servoj>"
  "961076d13ed378a92e75de5ec5b3a69b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'servoj)))
  "Returns md5sum for a message object of type 'servoj"
  "961076d13ed378a92e75de5ec5b3a69b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<servoj>)))
  "Returns full string definition for message of type '<servoj>"
  (cl:format cl:nil "int64 keepalive~%float64[] cmd_q~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'servoj)))
  "Returns full string definition for message of type 'servoj"
  (cl:format cl:nil "int64 keepalive~%float64[] cmd_q~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <servoj>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cmd_q) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <servoj>))
  "Converts a ROS message object to a list"
  (cl:list 'servoj
    (cl:cons ':keepalive (keepalive msg))
    (cl:cons ':cmd_q (cmd_q msg))
))
