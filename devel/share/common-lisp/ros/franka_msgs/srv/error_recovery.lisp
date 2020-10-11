; Auto-generated. Do not edit!


(cl:in-package franka_msgs-srv)


;//! \htmlinclude error_recovery-request.msg.html

(cl:defclass <error_recovery-request> (roslisp-msg-protocol:ros-message)
  ((req
    :reader req
    :initarg :req
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass error_recovery-request (<error_recovery-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <error_recovery-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'error_recovery-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_msgs-srv:<error_recovery-request> is deprecated: use franka_msgs-srv:error_recovery-request instead.")))

(cl:ensure-generic-function 'req-val :lambda-list '(m))
(cl:defmethod req-val ((m <error_recovery-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_msgs-srv:req-val is deprecated.  Use franka_msgs-srv:req instead.")
  (req m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <error_recovery-request>) ostream)
  "Serializes a message object of type '<error_recovery-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'req) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <error_recovery-request>) istream)
  "Deserializes a message object of type '<error_recovery-request>"
    (cl:setf (cl:slot-value msg 'req) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<error_recovery-request>)))
  "Returns string type for a service object of type '<error_recovery-request>"
  "franka_msgs/error_recoveryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'error_recovery-request)))
  "Returns string type for a service object of type 'error_recovery-request"
  "franka_msgs/error_recoveryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<error_recovery-request>)))
  "Returns md5sum for a message object of type '<error_recovery-request>"
  "7f352b44c251978f0843cd60887550cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'error_recovery-request)))
  "Returns md5sum for a message object of type 'error_recovery-request"
  "7f352b44c251978f0843cd60887550cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<error_recovery-request>)))
  "Returns full string definition for message of type '<error_recovery-request>"
  (cl:format cl:nil "bool req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'error_recovery-request)))
  "Returns full string definition for message of type 'error_recovery-request"
  (cl:format cl:nil "bool req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <error_recovery-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <error_recovery-request>))
  "Converts a ROS message object to a list"
  (cl:list 'error_recovery-request
    (cl:cons ':req (req msg))
))
;//! \htmlinclude error_recovery-response.msg.html

(cl:defclass <error_recovery-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass error_recovery-response (<error_recovery-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <error_recovery-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'error_recovery-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_msgs-srv:<error_recovery-response> is deprecated: use franka_msgs-srv:error_recovery-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <error_recovery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_msgs-srv:result-val is deprecated.  Use franka_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <error_recovery-response>) ostream)
  "Serializes a message object of type '<error_recovery-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <error_recovery-response>) istream)
  "Deserializes a message object of type '<error_recovery-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<error_recovery-response>)))
  "Returns string type for a service object of type '<error_recovery-response>"
  "franka_msgs/error_recoveryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'error_recovery-response)))
  "Returns string type for a service object of type 'error_recovery-response"
  "franka_msgs/error_recoveryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<error_recovery-response>)))
  "Returns md5sum for a message object of type '<error_recovery-response>"
  "7f352b44c251978f0843cd60887550cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'error_recovery-response)))
  "Returns md5sum for a message object of type 'error_recovery-response"
  "7f352b44c251978f0843cd60887550cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<error_recovery-response>)))
  "Returns full string definition for message of type '<error_recovery-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'error_recovery-response)))
  "Returns full string definition for message of type 'error_recovery-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <error_recovery-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <error_recovery-response>))
  "Converts a ROS message object to a list"
  (cl:list 'error_recovery-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'error_recovery)))
  'error_recovery-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'error_recovery)))
  'error_recovery-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'error_recovery)))
  "Returns string type for a service object of type '<error_recovery>"
  "franka_msgs/error_recovery")