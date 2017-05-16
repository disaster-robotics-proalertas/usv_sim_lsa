
(cl:in-package :asdf)

(defsystem "underwater_sensor_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DVL" :depends-on ("_package_DVL"))
    (:file "_package_DVL" :depends-on ("_package"))
    (:file "Pressure" :depends-on ("_package_Pressure"))
    (:file "_package_Pressure" :depends-on ("_package"))
  ))