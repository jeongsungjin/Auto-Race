
(cl:in-package :asdf)

(defsystem "car_planner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Drive_command" :depends-on ("_package_Drive_command"))
    (:file "_package_Drive_command" :depends-on ("_package"))
  ))