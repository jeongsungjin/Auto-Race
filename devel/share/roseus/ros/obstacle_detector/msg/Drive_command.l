;; Auto-generated. Do not edit!


(when (boundp 'obstacle_detector::Drive_command)
  (if (not (find-package "OBSTACLE_DETECTOR"))
    (make-package "OBSTACLE_DETECTOR"))
  (shadow 'Drive_command (find-package "OBSTACLE_DETECTOR")))
(unless (find-package "OBSTACLE_DETECTOR::DRIVE_COMMAND")
  (make-package "OBSTACLE_DETECTOR::DRIVE_COMMAND"))

(in-package "ROS")
;;//! \htmlinclude Drive_command.msg.html


(defclass obstacle_detector::Drive_command
  :super ros::object
  :slots (_speed _angle _flag ))

(defmethod obstacle_detector::Drive_command
  (:init
   (&key
    ((:speed __speed) 0.0)
    ((:angle __angle) 0.0)
    ((:flag __flag) nil)
    )
   (send-super :init)
   (setq _speed (float __speed))
   (setq _angle (float __angle))
   (setq _flag __flag)
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:flag
   (&optional (__flag :null))
   (if (not (eq __flag :null)) (setq _flag __flag)) _flag)
  (:serialization-length
   ()
   (+
    ;; float32 _speed
    4
    ;; float32 _angle
    4
    ;; bool _flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _flag
       (if _flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _flag
     (setq _flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get obstacle_detector::Drive_command :md5sum-) "95ff01497d9197f16d490d6e129fd2cd")
(setf (get obstacle_detector::Drive_command :datatype-) "obstacle_detector/Drive_command")
(setf (get obstacle_detector::Drive_command :definition-)
      "float32 speed
float32 angle
bool flag

")



(provide :obstacle_detector/Drive_command "95ff01497d9197f16d490d6e129fd2cd")


