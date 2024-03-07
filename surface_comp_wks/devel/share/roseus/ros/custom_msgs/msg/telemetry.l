;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::telemetry)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'telemetry (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::TELEMETRY")
  (make-package "CUSTOM_MSGS::TELEMETRY"))

(in-package "ROS")
;;//! \htmlinclude telemetry.msg.html


(defclass custom_msgs::telemetry
  :super ros::object
  :slots (_timestamp _imu_gyro_x _imu_gyro_y _imu_gyro_z _imu_gyro_compass_x _imu_gyro_compass_y _imu_gyro_compass_z _q1 _q2 _q3 _q4 _rollspeed _pitchspeed _yawspeed _internal_pressure _external_pressure _heading ))

(defmethod custom_msgs::telemetry
  (:init
   (&key
    ((:timestamp __timestamp) 0)
    ((:imu_gyro_x __imu_gyro_x) 0)
    ((:imu_gyro_y __imu_gyro_y) 0)
    ((:imu_gyro_z __imu_gyro_z) 0)
    ((:imu_gyro_compass_x __imu_gyro_compass_x) 0)
    ((:imu_gyro_compass_y __imu_gyro_compass_y) 0)
    ((:imu_gyro_compass_z __imu_gyro_compass_z) 0)
    ((:q1 __q1) 0.0)
    ((:q2 __q2) 0.0)
    ((:q3 __q3) 0.0)
    ((:q4 __q4) 0.0)
    ((:rollspeed __rollspeed) 0.0)
    ((:pitchspeed __pitchspeed) 0.0)
    ((:yawspeed __yawspeed) 0.0)
    ((:internal_pressure __internal_pressure) 0.0)
    ((:external_pressure __external_pressure) 0.0)
    ((:heading __heading) 0)
    )
   (send-super :init)
   (setq _timestamp (round __timestamp))
   (setq _imu_gyro_x (round __imu_gyro_x))
   (setq _imu_gyro_y (round __imu_gyro_y))
   (setq _imu_gyro_z (round __imu_gyro_z))
   (setq _imu_gyro_compass_x (round __imu_gyro_compass_x))
   (setq _imu_gyro_compass_y (round __imu_gyro_compass_y))
   (setq _imu_gyro_compass_z (round __imu_gyro_compass_z))
   (setq _q1 (float __q1))
   (setq _q2 (float __q2))
   (setq _q3 (float __q3))
   (setq _q4 (float __q4))
   (setq _rollspeed (float __rollspeed))
   (setq _pitchspeed (float __pitchspeed))
   (setq _yawspeed (float __yawspeed))
   (setq _internal_pressure (float __internal_pressure))
   (setq _external_pressure (float __external_pressure))
   (setq _heading (round __heading))
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:imu_gyro_x
   (&optional __imu_gyro_x)
   (if __imu_gyro_x (setq _imu_gyro_x __imu_gyro_x)) _imu_gyro_x)
  (:imu_gyro_y
   (&optional __imu_gyro_y)
   (if __imu_gyro_y (setq _imu_gyro_y __imu_gyro_y)) _imu_gyro_y)
  (:imu_gyro_z
   (&optional __imu_gyro_z)
   (if __imu_gyro_z (setq _imu_gyro_z __imu_gyro_z)) _imu_gyro_z)
  (:imu_gyro_compass_x
   (&optional __imu_gyro_compass_x)
   (if __imu_gyro_compass_x (setq _imu_gyro_compass_x __imu_gyro_compass_x)) _imu_gyro_compass_x)
  (:imu_gyro_compass_y
   (&optional __imu_gyro_compass_y)
   (if __imu_gyro_compass_y (setq _imu_gyro_compass_y __imu_gyro_compass_y)) _imu_gyro_compass_y)
  (:imu_gyro_compass_z
   (&optional __imu_gyro_compass_z)
   (if __imu_gyro_compass_z (setq _imu_gyro_compass_z __imu_gyro_compass_z)) _imu_gyro_compass_z)
  (:q1
   (&optional __q1)
   (if __q1 (setq _q1 __q1)) _q1)
  (:q2
   (&optional __q2)
   (if __q2 (setq _q2 __q2)) _q2)
  (:q3
   (&optional __q3)
   (if __q3 (setq _q3 __q3)) _q3)
  (:q4
   (&optional __q4)
   (if __q4 (setq _q4 __q4)) _q4)
  (:rollspeed
   (&optional __rollspeed)
   (if __rollspeed (setq _rollspeed __rollspeed)) _rollspeed)
  (:pitchspeed
   (&optional __pitchspeed)
   (if __pitchspeed (setq _pitchspeed __pitchspeed)) _pitchspeed)
  (:yawspeed
   (&optional __yawspeed)
   (if __yawspeed (setq _yawspeed __yawspeed)) _yawspeed)
  (:internal_pressure
   (&optional __internal_pressure)
   (if __internal_pressure (setq _internal_pressure __internal_pressure)) _internal_pressure)
  (:external_pressure
   (&optional __external_pressure)
   (if __external_pressure (setq _external_pressure __external_pressure)) _external_pressure)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:serialization-length
   ()
   (+
    ;; int32 _timestamp
    4
    ;; int32 _imu_gyro_x
    4
    ;; int32 _imu_gyro_y
    4
    ;; int32 _imu_gyro_z
    4
    ;; int32 _imu_gyro_compass_x
    4
    ;; int32 _imu_gyro_compass_y
    4
    ;; int32 _imu_gyro_compass_z
    4
    ;; float32 _q1
    4
    ;; float32 _q2
    4
    ;; float32 _q3
    4
    ;; float32 _q4
    4
    ;; float32 _rollspeed
    4
    ;; float32 _pitchspeed
    4
    ;; float32 _yawspeed
    4
    ;; float32 _internal_pressure
    4
    ;; float32 _external_pressure
    4
    ;; int32 _heading
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _timestamp
       (write-long _timestamp s)
     ;; int32 _imu_gyro_x
       (write-long _imu_gyro_x s)
     ;; int32 _imu_gyro_y
       (write-long _imu_gyro_y s)
     ;; int32 _imu_gyro_z
       (write-long _imu_gyro_z s)
     ;; int32 _imu_gyro_compass_x
       (write-long _imu_gyro_compass_x s)
     ;; int32 _imu_gyro_compass_y
       (write-long _imu_gyro_compass_y s)
     ;; int32 _imu_gyro_compass_z
       (write-long _imu_gyro_compass_z s)
     ;; float32 _q1
       (sys::poke _q1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _q2
       (sys::poke _q2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _q3
       (sys::poke _q3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _q4
       (sys::poke _q4 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rollspeed
       (sys::poke _rollspeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitchspeed
       (sys::poke _pitchspeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yawspeed
       (sys::poke _yawspeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _internal_pressure
       (sys::poke _internal_pressure (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _external_pressure
       (sys::poke _external_pressure (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _heading
       (write-long _heading s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _timestamp
     (setq _timestamp (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _imu_gyro_x
     (setq _imu_gyro_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _imu_gyro_y
     (setq _imu_gyro_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _imu_gyro_z
     (setq _imu_gyro_z (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _imu_gyro_compass_x
     (setq _imu_gyro_compass_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _imu_gyro_compass_y
     (setq _imu_gyro_compass_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _imu_gyro_compass_z
     (setq _imu_gyro_compass_z (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _q1
     (setq _q1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _q2
     (setq _q2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _q3
     (setq _q3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _q4
     (setq _q4 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rollspeed
     (setq _rollspeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitchspeed
     (setq _pitchspeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yawspeed
     (setq _yawspeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _internal_pressure
     (setq _internal_pressure (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _external_pressure
     (setq _external_pressure (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _heading
     (setq _heading (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get custom_msgs::telemetry :md5sum-) "287f88c9716cc47a884bba700bcc2292")
(setf (get custom_msgs::telemetry :datatype-) "custom_msgs/telemetry")
(setf (get custom_msgs::telemetry :definition-)
      "int32 timestamp

int32 imu_gyro_x
int32 imu_gyro_y
int32 imu_gyro_z



int32 imu_gyro_compass_x
int32 imu_gyro_compass_y
int32 imu_gyro_compass_z


float32 q1
float32 q2
float32 q3
float32 q4
float32 rollspeed
float32 pitchspeed
float32 yawspeed



float32 internal_pressure
float32 external_pressure
int32 heading
")



(provide :custom_msgs/telemetry "287f88c9716cc47a884bba700bcc2292")


