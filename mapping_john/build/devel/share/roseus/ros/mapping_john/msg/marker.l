;; Auto-generated. Do not edit!


(when (boundp 'mapping_john::marker)
  (if (not (find-package "MAPPING_JOHN"))
    (make-package "MAPPING_JOHN"))
  (shadow 'marker (find-package "MAPPING_JOHN")))
(unless (find-package "MAPPING_JOHN::MARKER")
  (make-package "MAPPING_JOHN::MARKER"))

(in-package "ROS")
;;//! \htmlinclude marker.msg.html


(defclass mapping_john::marker
  :super ros::object
  :slots (_number ))

(defmethod mapping_john::marker
  (:init
   (&key
    ((:number __number) 0.0)
    )
   (send-super :init)
   (setq _number (float __number))
   self)
  (:number
   (&optional __number)
   (if __number (setq _number __number)) _number)
  (:serialization-length
   ()
   (+
    ;; float32 _number
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _number
       (sys::poke _number (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _number
     (setq _number (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mapping_john::marker :md5sum-) "ded049c24c756963282afab14b2d0f6d")
(setf (get mapping_john::marker :datatype-) "mapping_john/marker")
(setf (get mapping_john::marker :definition-)
      "float32 number
")



(provide :mapping_john/marker "ded049c24c756963282afab14b2d0f6d")


