; Auto-generated. Do not edit!


(cl:in-package marker_msgs-msg)


;//! \htmlinclude marker.msg.html

(cl:defclass <marker> (roslisp-msg-protocol:ros-message)
  ((ids
    :reader ids
    :initarg :ids
    :type std_msgs-msg:Int16MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Int16MultiArray))
   (xErrors
    :reader xErrors
    :initarg :xErrors
    :type std_msgs-msg:Float32MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float32MultiArray))
   (yaws
    :reader yaws
    :initarg :yaws
    :type std_msgs-msg:Float32MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float32MultiArray)))
)

(cl:defclass marker (<marker>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <marker>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'marker)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name marker_msgs-msg:<marker> is deprecated: use marker_msgs-msg:marker instead.")))

(cl:ensure-generic-function 'ids-val :lambda-list '(m))
(cl:defmethod ids-val ((m <marker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader marker_msgs-msg:ids-val is deprecated.  Use marker_msgs-msg:ids instead.")
  (ids m))

(cl:ensure-generic-function 'xErrors-val :lambda-list '(m))
(cl:defmethod xErrors-val ((m <marker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader marker_msgs-msg:xErrors-val is deprecated.  Use marker_msgs-msg:xErrors instead.")
  (xErrors m))

(cl:ensure-generic-function 'yaws-val :lambda-list '(m))
(cl:defmethod yaws-val ((m <marker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader marker_msgs-msg:yaws-val is deprecated.  Use marker_msgs-msg:yaws instead.")
  (yaws m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <marker>) ostream)
  "Serializes a message object of type '<marker>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ids) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'xErrors) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'yaws) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <marker>) istream)
  "Deserializes a message object of type '<marker>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ids) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'xErrors) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'yaws) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<marker>)))
  "Returns string type for a message object of type '<marker>"
  "marker_msgs/marker")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'marker)))
  "Returns string type for a message object of type 'marker"
  "marker_msgs/marker")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<marker>)))
  "Returns md5sum for a message object of type '<marker>"
  "29b5307f7b0077b567c8b87adcaca5dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'marker)))
  "Returns md5sum for a message object of type 'marker"
  "29b5307f7b0077b567c8b87adcaca5dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<marker>)))
  "Returns full string definition for message of type '<marker>"
  (cl:format cl:nil "std_msgs/Int16MultiArray ids~%std_msgs/Float32MultiArray xErrors~%std_msgs/Float32MultiArray yaws~%================================================================================~%MSG: std_msgs/Int16MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%int16[]           data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'marker)))
  "Returns full string definition for message of type 'marker"
  (cl:format cl:nil "std_msgs/Int16MultiArray ids~%std_msgs/Float32MultiArray xErrors~%std_msgs/Float32MultiArray yaws~%================================================================================~%MSG: std_msgs/Int16MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%int16[]           data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <marker>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ids))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'xErrors))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'yaws))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <marker>))
  "Converts a ROS message object to a list"
  (cl:list 'marker
    (cl:cons ':ids (ids msg))
    (cl:cons ':xErrors (xErrors msg))
    (cl:cons ':yaws (yaws msg))
))
