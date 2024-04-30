;; Auto-generated. Do not edit!


(when (boundp 'marker_msgs::marker)
  (if (not (find-package "MARKER_MSGS"))
    (make-package "MARKER_MSGS"))
  (shadow 'marker (find-package "MARKER_MSGS")))
(unless (find-package "MARKER_MSGS::MARKER")
  (make-package "MARKER_MSGS::MARKER"))

(in-package "ROS")
;;//! \htmlinclude marker.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass marker_msgs::marker
  :super ros::object
  :slots (_ids _xErrors ))

(defmethod marker_msgs::marker
  (:init
   (&key
    ((:ids __ids) (instance std_msgs::Int16MultiArray :init))
    ((:xErrors __xErrors) (instance std_msgs::Float32MultiArray :init))
    )
   (send-super :init)
   (setq _ids __ids)
   (setq _xErrors __xErrors)
   self)
  (:ids
   (&rest __ids)
   (if (keywordp (car __ids))
       (send* _ids __ids)
     (progn
       (if __ids (setq _ids (car __ids)))
       _ids)))
  (:xErrors
   (&rest __xErrors)
   (if (keywordp (car __xErrors))
       (send* _xErrors __xErrors)
     (progn
       (if __xErrors (setq _xErrors (car __xErrors)))
       _xErrors)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Int16MultiArray _ids
    (send _ids :serialization-length)
    ;; std_msgs/Float32MultiArray _xErrors
    (send _xErrors :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Int16MultiArray _ids
       (send _ids :serialize s)
     ;; std_msgs/Float32MultiArray _xErrors
       (send _xErrors :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Int16MultiArray _ids
     (send _ids :deserialize buf ptr-) (incf ptr- (send _ids :serialization-length))
   ;; std_msgs/Float32MultiArray _xErrors
     (send _xErrors :deserialize buf ptr-) (incf ptr- (send _xErrors :serialization-length))
   ;;
   self)
  )

(setf (get marker_msgs::marker :md5sum-) "4a29dc35e14078fe013cf3949883c4e9")
(setf (get marker_msgs::marker :datatype-) "marker_msgs/marker")
(setf (get marker_msgs::marker :definition-)
      "std_msgs/Int16MultiArray ids
std_msgs/Float32MultiArray xErrors
================================================================================
MSG: std_msgs/Int16MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int16[]           data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/Float32MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data


")



(provide :marker_msgs/marker "4a29dc35e14078fe013cf3949883c4e9")


