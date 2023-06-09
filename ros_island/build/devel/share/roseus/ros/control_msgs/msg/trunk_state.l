;; Auto-generated. Do not edit!


(when (boundp 'control_msgs::trunk_state)
  (if (not (find-package "CONTROL_MSGS"))
    (make-package "CONTROL_MSGS"))
  (shadow 'trunk_state (find-package "CONTROL_MSGS")))
(unless (find-package "CONTROL_MSGS::TRUNK_STATE")
  (make-package "CONTROL_MSGS::TRUNK_STATE"))

(in-package "ROS")
;;//! \htmlinclude trunk_state.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass control_msgs::trunk_state
  :super ros::object
  :slots (_header _trunk_position _trunk_velocity ))

(defmethod control_msgs::trunk_state
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:trunk_position __trunk_position) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:trunk_velocity __trunk_velocity) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _trunk_position __trunk_position)
   (setq _trunk_velocity __trunk_velocity)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:trunk_position
   (&optional __trunk_position)
   (if __trunk_position (setq _trunk_position __trunk_position)) _trunk_position)
  (:trunk_velocity
   (&optional __trunk_velocity)
   (if __trunk_velocity (setq _trunk_velocity __trunk_velocity)) _trunk_velocity)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[6] _trunk_position
    (* 4    6)
    ;; float32[6] _trunk_velocity
    (* 4    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[6] _trunk_position
     (dotimes (i 6)
       (sys::poke (elt _trunk_position i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[6] _trunk_velocity
     (dotimes (i 6)
       (sys::poke (elt _trunk_velocity i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[6] _trunk_position
   (dotimes (i (length _trunk_position))
     (setf (elt _trunk_position i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[6] _trunk_velocity
   (dotimes (i (length _trunk_velocity))
     (setf (elt _trunk_velocity i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get control_msgs::trunk_state :md5sum-) "d6da26616d9cb3c34772795c92291a45")
(setf (get control_msgs::trunk_state :datatype-) "control_msgs/trunk_state")
(setf (get control_msgs::trunk_state :definition-)
      "#topic:'motion/trunk/state'
Header header

float32[6] trunk_position
float32[6] trunk_velocity

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



(provide :control_msgs/trunk_state "d6da26616d9cb3c34772795c92291a45")


