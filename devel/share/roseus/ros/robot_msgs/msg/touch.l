;; Auto-generated. Do not edit!


(when (boundp 'robot_msgs::touch)
  (if (not (find-package "ROBOT_MSGS"))
    (make-package "ROBOT_MSGS"))
  (shadow 'touch (find-package "ROBOT_MSGS")))
(unless (find-package "ROBOT_MSGS::TOUCH")
  (make-package "ROBOT_MSGS::TOUCH"))

(in-package "ROS")
;;//! \htmlinclude touch.msg.html


(defclass robot_msgs::touch
  :super ros::object
  :slots (_data _button ))

(defmethod robot_msgs::touch
  (:init
   (&key
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:button __button) 0)
    )
   (send-super :init)
   (setq _data __data)
   (setq _button (round __button))
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
    ;; int64 _button
    8
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
     ;; int64 _button
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _button (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _button) (= (length (_button . bv)) 2)) ;; bignum
              (write-long (ash (elt (_button . bv) 0) 0) s)
              (write-long (ash (elt (_button . bv) 1) -1) s))
             ((and (class _button) (= (length (_button . bv)) 1)) ;; big1
              (write-long (elt (_button . bv) 0) s)
              (write-long (if (>= _button 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _button s)(write-long (if (>= _button 0) 0 #xffffffff) s)))
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
   ;; int64 _button
#+(or :alpha :irix6 :x86_64)
      (setf _button (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _button (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get robot_msgs::touch :md5sum-) "1f1272e2491adfe1bf33324ccd3fec13")
(setf (get robot_msgs::touch :datatype-) "robot_msgs/touch")
(setf (get robot_msgs::touch :definition-)
      "float64[] data
int64 button

")



(provide :robot_msgs/touch "1f1272e2491adfe1bf33324ccd3fec13")


