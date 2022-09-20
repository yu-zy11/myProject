;; Auto-generated. Do not edit!


(when (boundp 'ros_msgs::JointCommand)
  (if (not (find-package "ROS_MSGS"))
    (make-package "ROS_MSGS"))
  (shadow 'JointCommand (find-package "ROS_MSGS")))
(unless (find-package "ROS_MSGS::JOINTCOMMAND")
  (make-package "ROS_MSGS::JOINTCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude JointCommand.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ros_msgs::JointCommand
  :super ros::object
  :slots (_header _position _velocity _torque _stiffness _damping _mode ))

(defmethod ros_msgs::JointCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:position __position) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velocity __velocity) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:torque __torque) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:stiffness __stiffness) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:damping __damping) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _torque __torque)
   (setq _stiffness __stiffness)
   (setq _damping __damping)
   (setq _mode (round __mode))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:torque
   (&optional __torque)
   (if __torque (setq _torque __torque)) _torque)
  (:stiffness
   (&optional __stiffness)
   (if __stiffness (setq _stiffness __stiffness)) _stiffness)
  (:damping
   (&optional __damping)
   (if __damping (setq _damping __damping)) _damping)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[] _position
    (* 8    (length _position)) 4
    ;; float64[] _velocity
    (* 8    (length _velocity)) 4
    ;; float64[] _torque
    (* 8    (length _torque)) 4
    ;; float64[] _stiffness
    (* 8    (length _stiffness)) 4
    ;; float64[] _damping
    (* 8    (length _damping)) 4
    ;; int32 _mode
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[] _position
     (write-long (length _position) s)
     (dotimes (i (length _position))
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _velocity
     (write-long (length _velocity) s)
     (dotimes (i (length _velocity))
       (sys::poke (elt _velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _torque
     (write-long (length _torque) s)
     (dotimes (i (length _torque))
       (sys::poke (elt _torque i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _stiffness
     (write-long (length _stiffness) s)
     (dotimes (i (length _stiffness))
       (sys::poke (elt _stiffness i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _damping
     (write-long (length _damping) s)
     (dotimes (i (length _damping))
       (sys::poke (elt _damping i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int32 _mode
       (write-long _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[] _position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocity (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _torque
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _torque (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _torque i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _stiffness
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stiffness (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _stiffness i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _damping
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _damping (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _damping i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ros_msgs::JointCommand :md5sum-) "1c3afcbadeaf39bc5678ea97610e221e")
(setf (get ros_msgs::JointCommand :datatype-) "ros_msgs/JointCommand")
(setf (get ros_msgs::JointCommand :definition-)
      "Header header
float64[] position
float64[] velocity
float64[] torque
float64[] stiffness
float64[] damping
int32 mode

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
string frame_id

")



(provide :ros_msgs/JointCommand "1c3afcbadeaf39bc5678ea97610e221e")


