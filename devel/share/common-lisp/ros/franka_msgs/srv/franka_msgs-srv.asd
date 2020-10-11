
(cl:in-package :asdf)

(defsystem "franka_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "error_recovery" :depends-on ("_package_error_recovery"))
    (:file "_package_error_recovery" :depends-on ("_package"))
  ))