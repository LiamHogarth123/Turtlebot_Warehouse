
(cl:in-package :asdf)

(defsystem "marker_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "marker" :depends-on ("_package_marker"))
    (:file "_package_marker" :depends-on ("_package"))
  ))