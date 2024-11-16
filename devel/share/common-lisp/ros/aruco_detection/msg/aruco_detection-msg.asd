
(cl:in-package :asdf)

(defsystem "aruco_detection-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MarkerArray" :depends-on ("_package_MarkerArray"))
    (:file "_package_MarkerArray" :depends-on ("_package"))
  ))