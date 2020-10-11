;; Auto-generated. Do not edit!


(when (boundp 'robot_msgs::omega)
  (if (not (find-package "ROBOT_MSGS"))
    (make-package "ROBOT_MSGS"))
  (shadow 'omega (find-package "ROBOT_MSGS")))
(unless (find-package "ROBOT_MSGS::OMEGA")
  (make-package "ROBOT_MSGS::OMEGA"))

(in-package "ROS")
;;//! \htmlinclude omega.msg.html


(defclass robot_msgs::omega
  :super ros::object
  :slots (_data _button ))

(defmethod robot_msgs::omega
  (:init
   (&key
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:button __button) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _data __data)
   (setq _button __button)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:button
   (&optional __button)
   (if __button (setq _button __button)) _button)
  (:serialization-length
   ()
   (+
    ;; float64[] _data
    (* 8    (length _data)) 4
    ;; float64[] _button
    (* 8    (length _button)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _button
     (write-long (length _button) s)
     (dotimes (i (length _button))
       (sys::poke (elt _button i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _button
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _button (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _button i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get robot_msgs::omega :md5sum-) "90483ddf79d66b127324c21be866ac8d")
(setf (get robot_msgs::omega :datatype-) "robot_msgs/omega")
(setf (get robot_msgs::omega :definition-)
      "float64[] data
float64[] button

")



(provide :robot_msgs/omega "90483ddf79d66b127324c21be866ac8d")


