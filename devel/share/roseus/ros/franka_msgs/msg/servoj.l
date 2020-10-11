;; Auto-generated. Do not edit!


(when (boundp 'franka_msgs::servoj)
  (if (not (find-package "FRANKA_MSGS"))
    (make-package "FRANKA_MSGS"))
  (shadow 'servoj (find-package "FRANKA_MSGS")))
(unless (find-package "FRANKA_MSGS::SERVOJ")
  (make-package "FRANKA_MSGS::SERVOJ"))

(in-package "ROS")
;;//! \htmlinclude servoj.msg.html


(defclass franka_msgs::servoj
  :super ros::object
  :slots (_keepalive _cmd_q ))

(defmethod franka_msgs::servoj
  (:init
   (&key
    ((:keepalive __keepalive) 0)
    ((:cmd_q __cmd_q) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _keepalive (round __keepalive))
   (setq _cmd_q __cmd_q)
   self)
  (:keepalive
   (&optional __keepalive)
   (if __keepalive (setq _keepalive __keepalive)) _keepalive)
  (:cmd_q
   (&optional __cmd_q)
   (if __cmd_q (setq _cmd_q __cmd_q)) _cmd_q)
  (:serialization-length
   ()
   (+
    ;; int64 _keepalive
    8
    ;; float64[] _cmd_q
    (* 8    (length _cmd_q)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _keepalive
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _keepalive (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _keepalive) (= (length (_keepalive . bv)) 2)) ;; bignum
              (write-long (ash (elt (_keepalive . bv) 0) 0) s)
              (write-long (ash (elt (_keepalive . bv) 1) -1) s))
             ((and (class _keepalive) (= (length (_keepalive . bv)) 1)) ;; big1
              (write-long (elt (_keepalive . bv) 0) s)
              (write-long (if (>= _keepalive 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _keepalive s)(write-long (if (>= _keepalive 0) 0 #xffffffff) s)))
     ;; float64[] _cmd_q
     (write-long (length _cmd_q) s)
     (dotimes (i (length _cmd_q))
       (sys::poke (elt _cmd_q i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _keepalive
#+(or :alpha :irix6 :x86_64)
      (setf _keepalive (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _keepalive (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64[] _cmd_q
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cmd_q (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cmd_q i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get franka_msgs::servoj :md5sum-) "961076d13ed378a92e75de5ec5b3a69b")
(setf (get franka_msgs::servoj :datatype-) "franka_msgs/servoj")
(setf (get franka_msgs::servoj :definition-)
      "int64 keepalive
float64[] cmd_q

")



(provide :franka_msgs/servoj "961076d13ed378a92e75de5ec5b3a69b")


