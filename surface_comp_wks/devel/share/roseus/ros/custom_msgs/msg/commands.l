;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::commands)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'commands (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::COMMANDS")
  (make-package "CUSTOM_MSGS::COMMANDS"))

(in-package "ROS")
;;//! \htmlinclude commands.msg.html


(defclass custom_msgs::commands
  :super ros::object
  :slots (_arm _mode _forward _lateral _thrust _pitch _roll _yaw _servo1 _servo2 ))

(defmethod custom_msgs::commands
  (:init
   (&key
    ((:arm __arm) nil)
    ((:mode __mode) "")
    ((:forward __forward) 0)
    ((:lateral __lateral) 0)
    ((:thrust __thrust) 0)
    ((:pitch __pitch) 0)
    ((:roll __roll) 0)
    ((:yaw __yaw) 0)
    ((:servo1 __servo1) 0)
    ((:servo2 __servo2) 0)
    )
   (send-super :init)
   (setq _arm __arm)
   (setq _mode (string __mode))
   (setq _forward (round __forward))
   (setq _lateral (round __lateral))
   (setq _thrust (round __thrust))
   (setq _pitch (round __pitch))
   (setq _roll (round __roll))
   (setq _yaw (round __yaw))
   (setq _servo1 (round __servo1))
   (setq _servo2 (round __servo2))
   self)
  (:arm
   (&optional (__arm :null))
   (if (not (eq __arm :null)) (setq _arm __arm)) _arm)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:forward
   (&optional __forward)
   (if __forward (setq _forward __forward)) _forward)
  (:lateral
   (&optional __lateral)
   (if __lateral (setq _lateral __lateral)) _lateral)
  (:thrust
   (&optional __thrust)
   (if __thrust (setq _thrust __thrust)) _thrust)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:servo1
   (&optional __servo1)
   (if __servo1 (setq _servo1 __servo1)) _servo1)
  (:servo2
   (&optional __servo2)
   (if __servo2 (setq _servo2 __servo2)) _servo2)
  (:serialization-length
   ()
   (+
    ;; bool _arm
    1
    ;; string _mode
    4 (length _mode)
    ;; int16 _forward
    2
    ;; int16 _lateral
    2
    ;; int16 _thrust
    2
    ;; int16 _pitch
    2
    ;; int16 _roll
    2
    ;; int16 _yaw
    2
    ;; int16 _servo1
    2
    ;; int16 _servo2
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _arm
       (if _arm (write-byte -1 s) (write-byte 0 s))
     ;; string _mode
       (write-long (length _mode) s) (princ _mode s)
     ;; int16 _forward
       (write-word _forward s)
     ;; int16 _lateral
       (write-word _lateral s)
     ;; int16 _thrust
       (write-word _thrust s)
     ;; int16 _pitch
       (write-word _pitch s)
     ;; int16 _roll
       (write-word _roll s)
     ;; int16 _yaw
       (write-word _yaw s)
     ;; int16 _servo1
       (write-word _servo1 s)
     ;; int16 _servo2
       (write-word _servo2 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _arm
     (setq _arm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16 _forward
     (setq _forward (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _lateral
     (setq _lateral (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _thrust
     (setq _thrust (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _pitch
     (setq _pitch (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _roll
     (setq _roll (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _yaw
     (setq _yaw (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _servo1
     (setq _servo1 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _servo2
     (setq _servo2 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get custom_msgs::commands :md5sum-) "fd03263c66e74cb889ba23a832fbae12")
(setf (get custom_msgs::commands :datatype-) "custom_msgs/commands")
(setf (get custom_msgs::commands :definition-)
      "bool arm
string mode
int16 forward
int16 lateral
int16 thrust
int16 pitch
int16 roll
int16 yaw
int16 servo1
int16 servo2
")



(provide :custom_msgs/commands "fd03263c66e74cb889ba23a832fbae12")


