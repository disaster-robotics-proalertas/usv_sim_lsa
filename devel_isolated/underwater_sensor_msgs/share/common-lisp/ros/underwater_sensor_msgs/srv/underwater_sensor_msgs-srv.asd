
(cl:in-package :asdf)

(defsystem "underwater_sensor_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :visualization_msgs-msg
)
  :components ((:file "_package")
    (:file "SpawnMarker" :depends-on ("_package_SpawnMarker"))
    (:file "_package_SpawnMarker" :depends-on ("_package"))
  ))