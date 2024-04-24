; Auto-generated. Do not edit!


(cl:in-package mapping_john-msg)


;//! \htmlinclude marker.msg.html

(cl:defclass <marker> (roslisp-msg-protocol:ros-message)
  ((number
    :reader number
    :initarg :number
    :type cl:float
    :initform 0.0))
)

(cl:defclass marker (<marker>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <marker>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'marker)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mapping_john-msg:<marker> is deprecated: use mapping_john-msg:marker instead.")))

(cl:ensure-generic-function 'number-val :lambda-list '(m))
(cl:defmethod number-val ((m <marker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapping_john-msg:number-val is deprecated.  Use mapping_john-msg:number instead.")
  (number m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <marker>) ostream)
  "Serializes a message object of type '<marker>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'number))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <marker>) istream)
  "Deserializes a message object of type '<marker>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'number) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<marker>)))
  "Returns string type for a message object of type '<marker>"
  "mapping_john/marker")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'marker)))
  "Returns string type for a message object of type 'marker"
  "mapping_john/marker")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<marker>)))
  "Returns md5sum for a message object of type '<marker>"
  "ded049c24c756963282afab14b2d0f6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'marker)))
  "Returns md5sum for a message object of type 'marker"
  "ded049c24c756963282afab14b2d0f6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<marker>)))
  "Returns full string definition for message of type '<marker>"
  (cl:format cl:nil "float32 number~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'marker)))
  "Returns full string definition for message of type 'marker"
  (cl:format cl:nil "float32 number~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <marker>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <marker>))
  "Converts a ROS message object to a list"
  (cl:list 'marker
    (cl:cons ':number (number msg))
))
