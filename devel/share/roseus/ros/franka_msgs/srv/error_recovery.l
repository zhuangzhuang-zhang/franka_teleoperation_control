;; Auto-generated. Do not edit!


(when (boundp 'franka_msgs::error_recovery)
  (if (not (find-package "FRANKA_MSGS"))
    (make-package "FRANKA_MSGS"))
  (shadow 'error_recovery (find-package "FRANKA_MSGS")))
(unless (find-package "FRANKA_MSGS::ERROR_RECOVERY")
  (make-package "FRANKA_MSGS::ERROR_RECOVERY"))
(unless (find-package "FRANKA_MSGS::ERROR_RECOVERYREQUEST")
  (make-package "FRANKA_MSGS::ERROR_RECOVERYREQUEST"))
(unless (find-package "FRANKA_MSGS::ERROR_RECOVERYRESPONSE")
  (make-package "FRANKA_MSGS::ERROR_RECOVERYRESPONSE"))

(in-package "ROS")





(defclass franka_msgs::error_recoveryRequest
  :super ros::object
  :slots (_req ))

(defmethod franka_msgs::error_recoveryRequest
  (:init
   (&key
    ((:req __req) nil)
    )
   (send-super :init)
   (setq _req __req)
   self)
  (:req
   (&optional __req)
   (if __req (setq _req __req)) _req)
  (:serialization-length
   ()
   (+
    ;; bool _req
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _req
       (if _req (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _req
     (setq _req (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass franka_msgs::error_recoveryResponse
  :super ros::object
  :slots (_result ))

(defmethod franka_msgs::error_recoveryResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass franka_msgs::error_recovery
  :super ros::object
  :slots ())

(setf (get franka_msgs::error_recovery :md5sum-) "7f352b44c251978f0843cd60887550cf")
(setf (get franka_msgs::error_recovery :datatype-) "franka_msgs/error_recovery")
(setf (get franka_msgs::error_recovery :request) franka_msgs::error_recoveryRequest)
(setf (get franka_msgs::error_recovery :response) franka_msgs::error_recoveryResponse)

(defmethod franka_msgs::error_recoveryRequest
  (:response () (instance franka_msgs::error_recoveryResponse :init)))

(setf (get franka_msgs::error_recoveryRequest :md5sum-) "7f352b44c251978f0843cd60887550cf")
(setf (get franka_msgs::error_recoveryRequest :datatype-) "franka_msgs/error_recoveryRequest")
(setf (get franka_msgs::error_recoveryRequest :definition-)
      "bool req
---
bool result

")

(setf (get franka_msgs::error_recoveryResponse :md5sum-) "7f352b44c251978f0843cd60887550cf")
(setf (get franka_msgs::error_recoveryResponse :datatype-) "franka_msgs/error_recoveryResponse")
(setf (get franka_msgs::error_recoveryResponse :definition-)
      "bool req
---
bool result

")



(provide :franka_msgs/error_recovery "7f352b44c251978f0843cd60887550cf")


