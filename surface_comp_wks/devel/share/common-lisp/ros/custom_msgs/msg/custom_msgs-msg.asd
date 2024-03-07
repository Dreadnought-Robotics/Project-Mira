
(cl:in-package :asdf)

(defsystem "custom_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "commands" :depends-on ("_package_commands"))
    (:file "_package_commands" :depends-on ("_package"))
    (:file "telemetry" :depends-on ("_package_telemetry"))
    (:file "_package_telemetry" :depends-on ("_package"))
  ))