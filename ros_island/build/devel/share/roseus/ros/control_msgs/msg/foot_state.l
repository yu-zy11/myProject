;; Auto-generated. Do not edit!


(when (boundp 'control_msgs::foot_state)
  (if (not (find-package "CONTROL_MSGS"))
    (make-package "CONTROL_MSGS"))
  (shadow 'foot_state (find-package "CONTROL_MSGS")))
(unless (find-package "CONTROL_MSGS::FOOT_STATE")
  (make-package "CONTROL_MSGS::FOOT_STATE"))

(in-package "ROS")
;;//! \htmlinclude foot_state.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass control_msgs::foot_state
  :super ros::object
  :slots (_header _foot_position _foot_velocity _contact_state _foot_force ))

(defmethod control_msgs::foot_state
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:foot_position __foot_position) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:foot_velocity __foot_velocity) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:contact_state __contact_state) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:foot_force __foot_force) (make-array 12 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _foot_position __foot_position)
   (setq _foot_velocity __foot_velocity)
   (setq _contact_state __contact_state)
   (setq _foot_force __foot_force)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:foot_position
   (&optional __foot_position)
   (if __foot_position (setq _foot_position __foot_position)) _foot_position)
  (:foot_velocity
   (&optional __foot_velocity)
   (if __foot_velocity (setq _foot_velocity __foot_velocity)) _foot_velocity)
  (:contact_state
   (&optional __contact_state)
   (if __contact_state (setq _contact_state __contact_state)) _contact_state)
  (:foot_force
   (&optional __foot_force)
   (if __foot_force (setq _foot_force __foot_force)) _foot_force)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[12] _foot_position
    (* 4    12)
    ;; float32[12] _foot_velocity
    (* 4    12)
    ;; float32[4] _contact_state
    (* 4    4)
    ;; float32[12] _foot_force
    (* 4    12)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[12] _foot_position
     (dotimes (i 12)
       (sys::poke (elt _foot_position i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _foot_velocity
     (dotimes (i 12)
       (sys::poke (elt _foot_velocity i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[4] _contact_state
     (dotimes (i 4)
       (sys::poke (elt _contact_state i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _foot_force
     (dotimes (i 12)
       (sys::poke (elt _foot_force i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[12] _foot_position
   (dotimes (i (length _foot_position))
     (setf (elt _foot_position i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _foot_velocity
   (dotimes (i (length _foot_velocity))
     (setf (elt _foot_velocity i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[4] _contact_state
   (dotimes (i (length _contact_state))
     (setf (elt _contact_state i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _foot_force
   (dotimes (i (length _foot_force))
     (setf (elt _foot_force i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get control_msgs::foot_state :md5sum-) "ed2e25b8bdf234bee1b653ec362f2558")
(setf (get control_msgs::foot_state :datatype-) "control_msgs/foot_state")
(setf (get control_msgs::foot_state :definition-)
      "#topic:'motion/foot/state'
Header header

float32[12] foot_position
float32[12] foot_velocity
float32[4]  contact_state
float32[12] foot_force

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



(provide :control_msgs/foot_state "ed2e25b8bdf234bee1b653ec362f2558")


