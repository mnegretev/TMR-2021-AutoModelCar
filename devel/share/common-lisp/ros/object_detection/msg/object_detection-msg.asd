
(cl:in-package :asdf)

(defsystem "object_detection-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "points_objects" :depends-on ("_package_points_objects"))
    (:file "_package_points_objects" :depends-on ("_package"))
  ))