;; Auto-generated. Do not edit!


(when (boundp 'underwater_sensor_msgs::Pressure)
  (if (not (find-package "UNDERWATER_SENSOR_MSGS"))
    (make-package "UNDERWATER_SENSOR_MSGS"))
  (shadow 'Pressure (find-package "UNDERWATER_SENSOR_MSGS")))
(unless (find-package "UNDERWATER_SENSOR_MSGS::PRESSURE")
  (make-package "UNDERWATER_SENSOR_MSGS::PRESSURE"))

(in-package "ROS")
;;//! \htmlinclude Pressure.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass underwater_sensor_msgs::Pressure
  :super ros::object
  :slots (_header _pressure ))

(defmethod underwater_sensor_msgs::Pressure
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pressure __pressure) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _pressure (float __pressure))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pressure
   (&optional __pressure)
   (if __pressure (setq _pressure __pressure)) _pressure)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _pressure
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _pressure
       (sys::poke _pressure (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _pressure
     (setq _pressure (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get underwater_sensor_msgs::Pressure :md5sum-) "cc86d3e590cd90e822f975defafdf965")
(setf (get underwater_sensor_msgs::Pressure :datatype-) "underwater_sensor_msgs/Pressure")
(setf (get underwater_sensor_msgs::Pressure :definition-)
      "Header header
float32 pressure 

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :underwater_sensor_msgs/Pressure "cc86d3e590cd90e822f975defafdf965")


