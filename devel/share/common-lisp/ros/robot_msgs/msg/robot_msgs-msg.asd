
(cl:in-package :asdf)

(defsystem "robot_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "omega" :depends-on ("_package_omega"))
    (:file "_package_omega" :depends-on ("_package"))
    (:file "touch" :depends-on ("_package_touch"))
    (:file "_package_touch" :depends-on ("_package"))
  ))