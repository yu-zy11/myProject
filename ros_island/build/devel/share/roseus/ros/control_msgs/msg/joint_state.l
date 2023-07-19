;; Auto-generated. Do not edit!


(when (boundp 'control_msgs::joint_state)
  (if (not (find-package "CONTROL_MSGS"))
    (make-package "CONTROL_MSGS"))
  (shadow 'joint_state (find-package "CONTROL_MSGS")))
(unless (find-package "CONTROL_MSGS::JOINT_STATE")
  (make-package "CONTROL_MSGS::JOINT_STATE"))

(in-package "ROS")
;;//! \htmlinclude joint_state.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass control_msgs::joint_state
  :super ros::object
  :slots (_header _joint_position _joint_velocity _torque ))

(defmethod control_msgs::joint_state
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:joint_position __joint_position) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:joint_velocity __joint_velocity) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:torque __torque) (make-array 12 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _joint_position __joint_position)
   (setq _joint_velocity __joint_velocity)
   (setq _torque __torque)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:joint_position
   (&optional __joint_position)
   (if __joint_position (setq _joint_position __joint_position)) _joint_position)
  (:joint_velocity
   (&optional __joint_velocity)
   (if __joint_velocity (setq _joint_velocity __joint_velocity)) _joint_velocity)
  (:torque
   (&optional __torque)
   (if __torque (setq _torque __torque)) _torque)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[12] _joint_position
    (* 4    12)
    ;; float32[12] _joint_velocity
    (* 4    12)
    ;; float32[12] _torque
    (* 4    12)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[12] _joint_position
     (dotimes (i 12)
       (sys::poke (elt _joint_position i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _joint_velocity
     (dotimes (i 12)
       (sys::poke (elt _joint_velocity i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _torque
     (dotimes (i 12)
       (sys::poke (elt _torque i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[12] _joint_position
   (dotimes (i (length _joint_position))
     (setf (elt _joint_position i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _joint_velocity
   (dotimes (i (length _joint_velocity))
     (setf (elt _joint_velocity i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _torque
   (dotimes (i (length _torque))
     (setf (elt _torque i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get control_msgs::joint_state :md5sum-) "cee320b1b25101ce84cadf5dd27b382f")
(setf (get control_msgs::joint_state :datatype-) "control_msgs/joint_state")
(setf (get control_msgs::joint_state :definition-)
      "#topic:'motion/joint/state'
Header header

float32[12] joint_position
float32[12] joint_velocity
float32[12] torque

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



(provide :control_msgs/joint_state "cee320b1b25101ce84cadf5dd27b382f")


