
(cl:in-package :asdf)

(defsystem "movemaster_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "setpoint" :depends-on ("_package_setpoint"))
    (:file "_package_setpoint" :depends-on ("_package"))
    (:file "status" :depends-on ("_package_status"))
    (:file "_package_status" :depends-on ("_package"))
  ))