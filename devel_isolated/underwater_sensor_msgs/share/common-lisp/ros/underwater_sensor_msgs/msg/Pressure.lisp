; Auto-generated. Do not edit!


(cl:in-package underwater_sensor_msgs-msg)


;//! \htmlinclude Pressure.msg.html

(cl:defclass <Pressure> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pressure
    :reader pressure
    :initarg :pressure
    :type cl:float
    :initform 0.0))
)

(cl:defclass Pressure (<Pressure>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Pressure>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Pressure)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name underwater_sensor_msgs-msg:<Pressure> is deprecated: use underwater_sensor_msgs-msg:Pressure instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Pressure>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader underwater_sensor_msgs-msg:header-val is deprecated.  Use underwater_sensor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pressure-val :lambda-list '(m))
(cl:defmethod pressure-val ((m <Pressure>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader underwater_sensor_msgs-msg:pressure-val is deprecated.  Use underwater_sensor_msgs-msg:pressure instead.")
  (pressure m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Pressure>) ostream)
  "Serializes a message object of type '<Pressure>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pressure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Pressure>) istream)
  "Deserializes a message object of type '<Pressure>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pressure) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Pressure>)))
  "Returns string type for a message object of type '<Pressure>"
  "underwater_sensor_msgs/Pressure")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pressure)))
  "Returns string type for a message object of type 'Pressure"
  "underwater_sensor_msgs/Pressure")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Pressure>)))
  "Returns md5sum for a message object of type '<Pressure>"
  "cc86d3e590cd90e822f975defafdf965")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Pressure)))
  "Returns md5sum for a message object of type 'Pressure"
  "cc86d3e590cd90e822f975defafdf965")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Pressure>)))
  "Returns full string definition for message of type '<Pressure>"
  (cl:format cl:nil "Header header~%float32 pressure ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Pressure)))
  "Returns full string definition for message of type 'Pressure"
  (cl:format cl:nil "Header header~%float32 pressure ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Pressure>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Pressure>))
  "Converts a ROS message object to a list"
  (cl:list 'Pressure
    (cl:cons ':header (header msg))
    (cl:cons ':pressure (pressure msg))
))
