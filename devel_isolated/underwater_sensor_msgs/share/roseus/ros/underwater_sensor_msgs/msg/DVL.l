;; Auto-generated. Do not edit!


(when (boundp 'underwater_sensor_msgs::DVL)
  (if (not (find-package "UNDERWATER_SENSOR_MSGS"))
    (make-package "UNDERWATER_SENSOR_MSGS"))
  (shadow 'DVL (find-package "UNDERWATER_SENSOR_MSGS")))
(unless (find-package "UNDERWATER_SENSOR_MSGS::DVL")
  (make-package "UNDERWATER_SENSOR_MSGS::DVL"))

(in-package "ROS")
;;//! \htmlinclude DVL.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass underwater_sensor_msgs::DVL
  :super ros::object
  :slots (_header _header_dvl _date _salinity _temperature _depth _sound_speed _test _wi_x_axis _wi_y_axis _wi_z_axis _wi_error _wi_status _bi_x_axis _bi_y_axis _bi_z_axis _bi_error _bi_status _ws_transverse _ws_longitudinal _ws_normal _ws_status _bs_transverse _bs_longitudinal _bs_normal _bs_status _we_east _we_north _we_upwards _we_status _be_east _be_north _be_upwards _be_status _wd_east _wd_north _wd_upwards _wd_range _wd_time _bd_east _bd_north _bd_upwards _bd_range _bd_time _raw_data ))

(defmethod underwater_sensor_msgs::DVL
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:header_dvl __header_dvl) "")
    ((:date __date) "")
    ((:salinity __salinity) 0.0)
    ((:temperature __temperature) 0.0)
    ((:depth __depth) 0.0)
    ((:sound_speed __sound_speed) 0.0)
    ((:test __test) 0)
    ((:wi_x_axis __wi_x_axis) 0.0)
    ((:wi_y_axis __wi_y_axis) 0.0)
    ((:wi_z_axis __wi_z_axis) 0.0)
    ((:wi_error __wi_error) 0.0)
    ((:wi_status __wi_status) "")
    ((:bi_x_axis __bi_x_axis) 0.0)
    ((:bi_y_axis __bi_y_axis) 0.0)
    ((:bi_z_axis __bi_z_axis) 0.0)
    ((:bi_error __bi_error) 0.0)
    ((:bi_status __bi_status) "")
    ((:ws_transverse __ws_transverse) 0.0)
    ((:ws_longitudinal __ws_longitudinal) 0.0)
    ((:ws_normal __ws_normal) 0.0)
    ((:ws_status __ws_status) "")
    ((:bs_transverse __bs_transverse) 0.0)
    ((:bs_longitudinal __bs_longitudinal) 0.0)
    ((:bs_normal __bs_normal) 0.0)
    ((:bs_status __bs_status) "")
    ((:we_east __we_east) 0.0)
    ((:we_north __we_north) 0.0)
    ((:we_upwards __we_upwards) 0.0)
    ((:we_status __we_status) "")
    ((:be_east __be_east) 0.0)
    ((:be_north __be_north) 0.0)
    ((:be_upwards __be_upwards) 0.0)
    ((:be_status __be_status) "")
    ((:wd_east __wd_east) 0.0)
    ((:wd_north __wd_north) 0.0)
    ((:wd_upwards __wd_upwards) 0.0)
    ((:wd_range __wd_range) 0.0)
    ((:wd_time __wd_time) 0.0)
    ((:bd_east __bd_east) 0.0)
    ((:bd_north __bd_north) 0.0)
    ((:bd_upwards __bd_upwards) 0.0)
    ((:bd_range __bd_range) 0.0)
    ((:bd_time __bd_time) 0.0)
    ((:raw_data __raw_data) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _header_dvl (string __header_dvl))
   (setq _date (string __date))
   (setq _salinity (float __salinity))
   (setq _temperature (float __temperature))
   (setq _depth (float __depth))
   (setq _sound_speed (float __sound_speed))
   (setq _test (round __test))
   (setq _wi_x_axis (float __wi_x_axis))
   (setq _wi_y_axis (float __wi_y_axis))
   (setq _wi_z_axis (float __wi_z_axis))
   (setq _wi_error (float __wi_error))
   (setq _wi_status (string __wi_status))
   (setq _bi_x_axis (float __bi_x_axis))
   (setq _bi_y_axis (float __bi_y_axis))
   (setq _bi_z_axis (float __bi_z_axis))
   (setq _bi_error (float __bi_error))
   (setq _bi_status (string __bi_status))
   (setq _ws_transverse (float __ws_transverse))
   (setq _ws_longitudinal (float __ws_longitudinal))
   (setq _ws_normal (float __ws_normal))
   (setq _ws_status (string __ws_status))
   (setq _bs_transverse (float __bs_transverse))
   (setq _bs_longitudinal (float __bs_longitudinal))
   (setq _bs_normal (float __bs_normal))
   (setq _bs_status (string __bs_status))
   (setq _we_east (float __we_east))
   (setq _we_north (float __we_north))
   (setq _we_upwards (float __we_upwards))
   (setq _we_status (string __we_status))
   (setq _be_east (float __be_east))
   (setq _be_north (float __be_north))
   (setq _be_upwards (float __be_upwards))
   (setq _be_status (string __be_status))
   (setq _wd_east (float __wd_east))
   (setq _wd_north (float __wd_north))
   (setq _wd_upwards (float __wd_upwards))
   (setq _wd_range (float __wd_range))
   (setq _wd_time (float __wd_time))
   (setq _bd_east (float __bd_east))
   (setq _bd_north (float __bd_north))
   (setq _bd_upwards (float __bd_upwards))
   (setq _bd_range (float __bd_range))
   (setq _bd_time (float __bd_time))
   (setq _raw_data (string __raw_data))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:header_dvl
   (&optional __header_dvl)
   (if __header_dvl (setq _header_dvl __header_dvl)) _header_dvl)
  (:date
   (&optional __date)
   (if __date (setq _date __date)) _date)
  (:salinity
   (&optional __salinity)
   (if __salinity (setq _salinity __salinity)) _salinity)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:depth
   (&optional __depth)
   (if __depth (setq _depth __depth)) _depth)
  (:sound_speed
   (&optional __sound_speed)
   (if __sound_speed (setq _sound_speed __sound_speed)) _sound_speed)
  (:test
   (&optional __test)
   (if __test (setq _test __test)) _test)
  (:wi_x_axis
   (&optional __wi_x_axis)
   (if __wi_x_axis (setq _wi_x_axis __wi_x_axis)) _wi_x_axis)
  (:wi_y_axis
   (&optional __wi_y_axis)
   (if __wi_y_axis (setq _wi_y_axis __wi_y_axis)) _wi_y_axis)
  (:wi_z_axis
   (&optional __wi_z_axis)
   (if __wi_z_axis (setq _wi_z_axis __wi_z_axis)) _wi_z_axis)
  (:wi_error
   (&optional __wi_error)
   (if __wi_error (setq _wi_error __wi_error)) _wi_error)
  (:wi_status
   (&optional __wi_status)
   (if __wi_status (setq _wi_status __wi_status)) _wi_status)
  (:bi_x_axis
   (&optional __bi_x_axis)
   (if __bi_x_axis (setq _bi_x_axis __bi_x_axis)) _bi_x_axis)
  (:bi_y_axis
   (&optional __bi_y_axis)
   (if __bi_y_axis (setq _bi_y_axis __bi_y_axis)) _bi_y_axis)
  (:bi_z_axis
   (&optional __bi_z_axis)
   (if __bi_z_axis (setq _bi_z_axis __bi_z_axis)) _bi_z_axis)
  (:bi_error
   (&optional __bi_error)
   (if __bi_error (setq _bi_error __bi_error)) _bi_error)
  (:bi_status
   (&optional __bi_status)
   (if __bi_status (setq _bi_status __bi_status)) _bi_status)
  (:ws_transverse
   (&optional __ws_transverse)
   (if __ws_transverse (setq _ws_transverse __ws_transverse)) _ws_transverse)
  (:ws_longitudinal
   (&optional __ws_longitudinal)
   (if __ws_longitudinal (setq _ws_longitudinal __ws_longitudinal)) _ws_longitudinal)
  (:ws_normal
   (&optional __ws_normal)
   (if __ws_normal (setq _ws_normal __ws_normal)) _ws_normal)
  (:ws_status
   (&optional __ws_status)
   (if __ws_status (setq _ws_status __ws_status)) _ws_status)
  (:bs_transverse
   (&optional __bs_transverse)
   (if __bs_transverse (setq _bs_transverse __bs_transverse)) _bs_transverse)
  (:bs_longitudinal
   (&optional __bs_longitudinal)
   (if __bs_longitudinal (setq _bs_longitudinal __bs_longitudinal)) _bs_longitudinal)
  (:bs_normal
   (&optional __bs_normal)
   (if __bs_normal (setq _bs_normal __bs_normal)) _bs_normal)
  (:bs_status
   (&optional __bs_status)
   (if __bs_status (setq _bs_status __bs_status)) _bs_status)
  (:we_east
   (&optional __we_east)
   (if __we_east (setq _we_east __we_east)) _we_east)
  (:we_north
   (&optional __we_north)
   (if __we_north (setq _we_north __we_north)) _we_north)
  (:we_upwards
   (&optional __we_upwards)
   (if __we_upwards (setq _we_upwards __we_upwards)) _we_upwards)
  (:we_status
   (&optional __we_status)
   (if __we_status (setq _we_status __we_status)) _we_status)
  (:be_east
   (&optional __be_east)
   (if __be_east (setq _be_east __be_east)) _be_east)
  (:be_north
   (&optional __be_north)
   (if __be_north (setq _be_north __be_north)) _be_north)
  (:be_upwards
   (&optional __be_upwards)
   (if __be_upwards (setq _be_upwards __be_upwards)) _be_upwards)
  (:be_status
   (&optional __be_status)
   (if __be_status (setq _be_status __be_status)) _be_status)
  (:wd_east
   (&optional __wd_east)
   (if __wd_east (setq _wd_east __wd_east)) _wd_east)
  (:wd_north
   (&optional __wd_north)
   (if __wd_north (setq _wd_north __wd_north)) _wd_north)
  (:wd_upwards
   (&optional __wd_upwards)
   (if __wd_upwards (setq _wd_upwards __wd_upwards)) _wd_upwards)
  (:wd_range
   (&optional __wd_range)
   (if __wd_range (setq _wd_range __wd_range)) _wd_range)
  (:wd_time
   (&optional __wd_time)
   (if __wd_time (setq _wd_time __wd_time)) _wd_time)
  (:bd_east
   (&optional __bd_east)
   (if __bd_east (setq _bd_east __bd_east)) _bd_east)
  (:bd_north
   (&optional __bd_north)
   (if __bd_north (setq _bd_north __bd_north)) _bd_north)
  (:bd_upwards
   (&optional __bd_upwards)
   (if __bd_upwards (setq _bd_upwards __bd_upwards)) _bd_upwards)
  (:bd_range
   (&optional __bd_range)
   (if __bd_range (setq _bd_range __bd_range)) _bd_range)
  (:bd_time
   (&optional __bd_time)
   (if __bd_time (setq _bd_time __bd_time)) _bd_time)
  (:raw_data
   (&optional __raw_data)
   (if __raw_data (setq _raw_data __raw_data)) _raw_data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _header_dvl
    4 (length _header_dvl)
    ;; string _date
    4 (length _date)
    ;; float64 _salinity
    8
    ;; float64 _temperature
    8
    ;; float64 _depth
    8
    ;; float64 _sound_speed
    8
    ;; int32 _test
    4
    ;; float64 _wi_x_axis
    8
    ;; float64 _wi_y_axis
    8
    ;; float64 _wi_z_axis
    8
    ;; float64 _wi_error
    8
    ;; string _wi_status
    4 (length _wi_status)
    ;; float64 _bi_x_axis
    8
    ;; float64 _bi_y_axis
    8
    ;; float64 _bi_z_axis
    8
    ;; float64 _bi_error
    8
    ;; string _bi_status
    4 (length _bi_status)
    ;; float64 _ws_transverse
    8
    ;; float64 _ws_longitudinal
    8
    ;; float64 _ws_normal
    8
    ;; string _ws_status
    4 (length _ws_status)
    ;; float64 _bs_transverse
    8
    ;; float64 _bs_longitudinal
    8
    ;; float64 _bs_normal
    8
    ;; string _bs_status
    4 (length _bs_status)
    ;; float64 _we_east
    8
    ;; float64 _we_north
    8
    ;; float64 _we_upwards
    8
    ;; string _we_status
    4 (length _we_status)
    ;; float64 _be_east
    8
    ;; float64 _be_north
    8
    ;; float64 _be_upwards
    8
    ;; string _be_status
    4 (length _be_status)
    ;; float64 _wd_east
    8
    ;; float64 _wd_north
    8
    ;; float64 _wd_upwards
    8
    ;; float64 _wd_range
    8
    ;; float64 _wd_time
    8
    ;; float64 _bd_east
    8
    ;; float64 _bd_north
    8
    ;; float64 _bd_upwards
    8
    ;; float64 _bd_range
    8
    ;; float64 _bd_time
    8
    ;; string _raw_data
    4 (length _raw_data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _header_dvl
       (write-long (length _header_dvl) s) (princ _header_dvl s)
     ;; string _date
       (write-long (length _date) s) (princ _date s)
     ;; float64 _salinity
       (sys::poke _salinity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _temperature
       (sys::poke _temperature (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _depth
       (sys::poke _depth (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _sound_speed
       (sys::poke _sound_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _test
       (write-long _test s)
     ;; float64 _wi_x_axis
       (sys::poke _wi_x_axis (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _wi_y_axis
       (sys::poke _wi_y_axis (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _wi_z_axis
       (sys::poke _wi_z_axis (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _wi_error
       (sys::poke _wi_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _wi_status
       (write-long (length _wi_status) s) (princ _wi_status s)
     ;; float64 _bi_x_axis
       (sys::poke _bi_x_axis (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _bi_y_axis
       (sys::poke _bi_y_axis (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _bi_z_axis
       (sys::poke _bi_z_axis (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _bi_error
       (sys::poke _bi_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _bi_status
       (write-long (length _bi_status) s) (princ _bi_status s)
     ;; float64 _ws_transverse
       (sys::poke _ws_transverse (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ws_longitudinal
       (sys::poke _ws_longitudinal (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ws_normal
       (sys::poke _ws_normal (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _ws_status
       (write-long (length _ws_status) s) (princ _ws_status s)
     ;; float64 _bs_transverse
       (sys::poke _bs_transverse (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _bs_longitudinal
       (sys::poke _bs_longitudinal (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _bs_normal
       (sys::poke _bs_normal (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _bs_status
       (write-long (length _bs_status) s) (princ _bs_status s)
     ;; float64 _we_east
       (sys::poke _we_east (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _we_north
       (sys::poke _we_north (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _we_upwards
       (sys::poke _we_upwards (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _we_status
       (write-long (length _we_status) s) (princ _we_status s)
     ;; float64 _be_east
       (sys::poke _be_east (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _be_north
       (sys::poke _be_north (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _be_upwards
       (sys::poke _be_upwards (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _be_status
       (write-long (length _be_status) s) (princ _be_status s)
     ;; float64 _wd_east
       (sys::poke _wd_east (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _wd_north
       (sys::poke _wd_north (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _wd_upwards
       (sys::poke _wd_upwards (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _wd_range
       (sys::poke _wd_range (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _wd_time
       (sys::poke _wd_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _bd_east
       (sys::poke _bd_east (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _bd_north
       (sys::poke _bd_north (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _bd_upwards
       (sys::poke _bd_upwards (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _bd_range
       (sys::poke _bd_range (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _bd_time
       (sys::poke _bd_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _raw_data
       (write-long (length _raw_data) s) (princ _raw_data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _header_dvl
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _header_dvl (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _date
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _date (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _salinity
     (setq _salinity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _temperature
     (setq _temperature (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _depth
     (setq _depth (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _sound_speed
     (setq _sound_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _test
     (setq _test (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _wi_x_axis
     (setq _wi_x_axis (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _wi_y_axis
     (setq _wi_y_axis (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _wi_z_axis
     (setq _wi_z_axis (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _wi_error
     (setq _wi_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _wi_status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _wi_status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _bi_x_axis
     (setq _bi_x_axis (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _bi_y_axis
     (setq _bi_y_axis (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _bi_z_axis
     (setq _bi_z_axis (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _bi_error
     (setq _bi_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _bi_status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _bi_status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _ws_transverse
     (setq _ws_transverse (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ws_longitudinal
     (setq _ws_longitudinal (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ws_normal
     (setq _ws_normal (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _ws_status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ws_status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _bs_transverse
     (setq _bs_transverse (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _bs_longitudinal
     (setq _bs_longitudinal (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _bs_normal
     (setq _bs_normal (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _bs_status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _bs_status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _we_east
     (setq _we_east (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _we_north
     (setq _we_north (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _we_upwards
     (setq _we_upwards (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _we_status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _we_status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _be_east
     (setq _be_east (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _be_north
     (setq _be_north (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _be_upwards
     (setq _be_upwards (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _be_status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _be_status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _wd_east
     (setq _wd_east (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _wd_north
     (setq _wd_north (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _wd_upwards
     (setq _wd_upwards (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _wd_range
     (setq _wd_range (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _wd_time
     (setq _wd_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _bd_east
     (setq _bd_east (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _bd_north
     (setq _bd_north (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _bd_upwards
     (setq _bd_upwards (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _bd_range
     (setq _bd_range (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _bd_time
     (setq _bd_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _raw_data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _raw_data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get underwater_sensor_msgs::DVL :md5sum-) "c455f331872552e620e26fc7caad335d")
(setf (get underwater_sensor_msgs::DVL :datatype-) "underwater_sensor_msgs/DVL")
(setf (get underwater_sensor_msgs::DVL :definition-)
      "Header header
string header_dvl
string date 
float64 salinity 
float64 temperature   	# TODO::ARNAU ha de ser generic ?
float64 depth  		# TODO::ARNAU ha de ser generic
float64 sound_speed 
int32 test
#WATER-MASS, INSTRUMENT-REFERENCED VELOCITY DATA
float64 wi_x_axis 
float64 wi_y_axis 
float64 wi_z_axis 
float64 wi_error 
string wi_status 
#BOTTOM-TRACK, INSTRUMENT-REFERENCED VELOCITY DATA
float64 bi_x_axis 
float64 bi_y_axis 
float64 bi_z_axis 
float64 bi_error 
string bi_status 
#WATER-MASS, SHIP-REFERENCED VELOCITY DATA
float64 ws_transverse 
float64 ws_longitudinal 
float64 ws_normal 
string ws_status 
#BOTTOM-TRACK, SHIP-REFERENCED VELOCITY DATA
float64 bs_transverse 
float64 bs_longitudinal 
float64 bs_normal 
string bs_status 
#WATER-MASS, EARTH-REFERENCED VELOCITY DATA
float64 we_east 
float64 we_north 
float64 we_upwards 
string  we_status 
#BOTTOM-TRACK, EARTH-REFERENCED VELOCITY DATA
float64 be_east 
float64 be_north 
float64 be_upwards 
string  be_status 
#  WATER-MASS, EARTH-REFERENCED DISTANCE DATA
float64 wd_east 
float64 wd_north 
float64 wd_upwards 
float64 wd_range 
float64 wd_time 
#BOTTOM-TRACK, EARTH-REFERENCED DISTANCE DATA
float64 bd_east 
float64 bd_north 
float64 bd_upwards 
float64 bd_range 
float64 bd_time
#RAW DATA 
string raw_data

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



(provide :underwater_sensor_msgs/DVL "c455f331872552e620e26fc7caad335d")


