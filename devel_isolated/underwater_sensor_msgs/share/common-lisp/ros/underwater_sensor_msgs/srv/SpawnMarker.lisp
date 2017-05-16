; Auto-generated. Do not edit!


(cl:in-package underwater_sensor_msgs-srv)


;//! \htmlinclude SpawnMarker-request.msg.html

(cl:defclass <SpawnMarker-request> (roslisp-msg-protocol:ros-message)
  ((marker
    :reader marker
    :initarg :marker
    :type visualization_msgs-msg:Marker
    :initform (cl:make-instance 'visualization_msgs-msg:Marker)))
)

(cl:defclass SpawnMarker-request (<SpawnMarker-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpawnMarker-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpawnMarker-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name underwater_sensor_msgs-srv:<SpawnMarker-request> is deprecated: use underwater_sensor_msgs-srv:SpawnMarker-request instead.")))

(cl:ensure-generic-function 'marker-val :lambda-list '(m))
(cl:defmethod marker-val ((m <SpawnMarker-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader underwater_sensor_msgs-srv:marker-val is deprecated.  Use underwater_sensor_msgs-srv:marker instead.")
  (marker m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpawnMarker-request>) ostream)
  "Serializes a message object of type '<SpawnMarker-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'marker) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpawnMarker-request>) istream)
  "Deserializes a message object of type '<SpawnMarker-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'marker) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpawnMarker-request>)))
  "Returns string type for a service object of type '<SpawnMarker-request>"
  "underwater_sensor_msgs/SpawnMarkerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpawnMarker-request)))
  "Returns string type for a service object of type 'SpawnMarker-request"
  "underwater_sensor_msgs/SpawnMarkerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpawnMarker-request>)))
  "Returns md5sum for a message object of type '<SpawnMarker-request>"
  "a7395a01b635e68c485c6bc7fed0b5a1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpawnMarker-request)))
  "Returns md5sum for a message object of type 'SpawnMarker-request"
  "a7395a01b635e68c485c6bc7fed0b5a1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpawnMarker-request>)))
  "Returns full string definition for message of type '<SpawnMarker-request>"
  (cl:format cl:nil "visualization_msgs/Marker marker~%~%================================================================================~%MSG: visualization_msgs/Marker~%# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz~%~%uint8 ARROW=0~%uint8 CUBE=1~%uint8 SPHERE=2~%uint8 CYLINDER=3~%uint8 LINE_STRIP=4~%uint8 LINE_LIST=5~%uint8 CUBE_LIST=6~%uint8 SPHERE_LIST=7~%uint8 POINTS=8~%uint8 TEXT_VIEW_FACING=9~%uint8 MESH_RESOURCE=10~%uint8 TRIANGLE_LIST=11~%~%uint8 ADD=0~%uint8 MODIFY=0~%uint8 DELETE=2~%uint8 DELETEALL=3~%~%Header header                        # header for time/frame information~%string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object~%int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later~%int32 type 		                       # Type of object~%int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects~%geometry_msgs/Pose pose                 # Pose of the object~%geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)~%std_msgs/ColorRGBA color             # Color [0.0-1.0]~%duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever~%bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep~%~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%geometry_msgs/Point[] points~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%#number of colors must either be 0 or equal to the number of points~%#NOTE: alpha is not yet used~%std_msgs/ColorRGBA[] colors~%~%# NOTE: only used for text markers~%string text~%~%# NOTE: only used for MESH_RESOURCE markers~%string mesh_resource~%bool mesh_use_embedded_materials~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpawnMarker-request)))
  "Returns full string definition for message of type 'SpawnMarker-request"
  (cl:format cl:nil "visualization_msgs/Marker marker~%~%================================================================================~%MSG: visualization_msgs/Marker~%# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz~%~%uint8 ARROW=0~%uint8 CUBE=1~%uint8 SPHERE=2~%uint8 CYLINDER=3~%uint8 LINE_STRIP=4~%uint8 LINE_LIST=5~%uint8 CUBE_LIST=6~%uint8 SPHERE_LIST=7~%uint8 POINTS=8~%uint8 TEXT_VIEW_FACING=9~%uint8 MESH_RESOURCE=10~%uint8 TRIANGLE_LIST=11~%~%uint8 ADD=0~%uint8 MODIFY=0~%uint8 DELETE=2~%uint8 DELETEALL=3~%~%Header header                        # header for time/frame information~%string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object~%int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later~%int32 type 		                       # Type of object~%int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects~%geometry_msgs/Pose pose                 # Pose of the object~%geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)~%std_msgs/ColorRGBA color             # Color [0.0-1.0]~%duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever~%bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep~%~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%geometry_msgs/Point[] points~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%#number of colors must either be 0 or equal to the number of points~%#NOTE: alpha is not yet used~%std_msgs/ColorRGBA[] colors~%~%# NOTE: only used for text markers~%string text~%~%# NOTE: only used for MESH_RESOURCE markers~%string mesh_resource~%bool mesh_use_embedded_materials~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpawnMarker-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'marker))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpawnMarker-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SpawnMarker-request
    (cl:cons ':marker (marker msg))
))
;//! \htmlinclude SpawnMarker-response.msg.html

(cl:defclass <SpawnMarker-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (status_message
    :reader status_message
    :initarg :status_message
    :type cl:string
    :initform ""))
)

(cl:defclass SpawnMarker-response (<SpawnMarker-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpawnMarker-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpawnMarker-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name underwater_sensor_msgs-srv:<SpawnMarker-response> is deprecated: use underwater_sensor_msgs-srv:SpawnMarker-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SpawnMarker-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader underwater_sensor_msgs-srv:success-val is deprecated.  Use underwater_sensor_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <SpawnMarker-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader underwater_sensor_msgs-srv:status_message-val is deprecated.  Use underwater_sensor_msgs-srv:status_message instead.")
  (status_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpawnMarker-response>) ostream)
  "Serializes a message object of type '<SpawnMarker-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpawnMarker-response>) istream)
  "Deserializes a message object of type '<SpawnMarker-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status_message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status_message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpawnMarker-response>)))
  "Returns string type for a service object of type '<SpawnMarker-response>"
  "underwater_sensor_msgs/SpawnMarkerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpawnMarker-response)))
  "Returns string type for a service object of type 'SpawnMarker-response"
  "underwater_sensor_msgs/SpawnMarkerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpawnMarker-response>)))
  "Returns md5sum for a message object of type '<SpawnMarker-response>"
  "a7395a01b635e68c485c6bc7fed0b5a1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpawnMarker-response)))
  "Returns md5sum for a message object of type 'SpawnMarker-response"
  "a7395a01b635e68c485c6bc7fed0b5a1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpawnMarker-response>)))
  "Returns full string definition for message of type '<SpawnMarker-response>"
  (cl:format cl:nil "bool success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpawnMarker-response)))
  "Returns full string definition for message of type 'SpawnMarker-response"
  (cl:format cl:nil "bool success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpawnMarker-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpawnMarker-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SpawnMarker-response
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SpawnMarker)))
  'SpawnMarker-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SpawnMarker)))
  'SpawnMarker-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpawnMarker)))
  "Returns string type for a service object of type '<SpawnMarker>"
  "underwater_sensor_msgs/SpawnMarker")