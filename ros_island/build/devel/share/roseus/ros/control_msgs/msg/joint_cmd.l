;; Auto-generated. Do not edit!


(when (boundp 'control_msgs::joint_cmd)
  (if (not (find-package "CONTROL_MSGS"))
    (make-package "CONTROL_MSGS"))
  (shadow 'joint_cmd (find-package "CONTROL_MSGS")))
(unless (find-package "CONTROL_MSGS::JOINT_CMD")
  (make-package "CONTROL_MSGS::JOINT_CMD"))

(in-package "ROS")
;;//! \htmlinclude joint_cmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass control_msgs::joint_cmd
  :super ros::object
  :slots (_header _joint_position_cmd _joint_velocity_cmd _torque_ff _joint_kp _joint_kd ))

(defmethod control_msgs::joint_cmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:joint_position_cmd __joint_position_cmd) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:joint_velocity_cmd __joint_velocity_cmd) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:torque_ff __torque_ff) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:joint_kp __joint_kp) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:joint_kd __joint_kd) (make-array 12 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _joint_position_cmd __joint_position_cmd)
   (setq _joint_velocity_cmd __joint_velocity_cmd)
   (setq _torque_ff __torque_ff)
   (setq _joint_kp __joint_kp)
   (setq _joint_kd __joint_kd)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:joint_position_cmd
   (&optional __joint_position_cmd)
   (if __joint_position_cmd (setq _joint_position_cmd __joint_position_cmd)) _joint_position_cmd)
  (:joint_velocity_cmd
   (&optional __joint_velocity_cmd)
   (if __joint_velocity_cmd (setq _joint_velocity_cmd __joint_velocity_cmd)) _joint_velocity_cmd)
  (:torque_ff
   (&optional __torque_ff)
   (if __torque_ff (setq _torque_ff __torque_ff)) _torque_ff)
  (:joint_kp
   (&optional __joint_kp)
   (if __joint_kp (setq _joint_kp __joint_kp)) _joint_kp)
  (:joint_kd
   (&optional __joint_kd)
   (if __joint_kd (setq _joint_kd __joint_kd)) _joint_kd)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[12] _joint_position_cmd
    (* 4    12)
    ;; float32[12] _joint_velocity_cmd
    (* 4    12)
    ;; float32[12] _torque_ff
    (* 4    12)
    ;; float32[12] _joint_kp
    (* 4    12)
    ;; float32[12] _joint_kd
    (* 4    12)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[12] _joint_position_cmd
     (dotimes (i 12)
       (sys::poke (elt _joint_position_cmd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _joint_velocity_cmd
     (dotimes (i 12)
       (sys::poke (elt _joint_velocity_cmd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _torque_ff
     (dotimes (i 12)
       (sys::poke (elt _torque_ff i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _joint_kp
     (dotimes (i 12)
       (sys::poke (elt _joint_kp i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _joint_kd
     (dotimes (i 12)
       (sys::poke (elt _joint_kd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[12] _joint_position_cmd
   (dotimes (i (length _joint_position_cmd))
     (setf (elt _joint_position_cmd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _joint_velocity_cmd
   (dotimes (i (length _joint_velocity_cmd))
     (setf (elt _joint_velocity_cmd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _torque_ff
   (dotimes (i (length _torque_ff))
     (setf (elt _torque_ff i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _joint_kp
   (dotimes (i (length _joint_kp))
     (setf (elt _joint_kp i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _joint_kd
   (dotimes (i (length _joint_kd))
     (setf (elt _joint_kd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get control_msgs::joint_cmd :md5sum-) "42cbcd630f7428dcf45148f745ff01c3")
(setf (get control_msgs::joint_cmd :datatype-) "control_msgs/joint_cmd")
(setf (get control_msgs::joint_cmd :definition-)
      "#topic:'motion/joint/cmd'
Header header

float32[12] joint_position_cmd
float32[12] joint_velocity_cmd
float32[12] torque_ff
float32[12] joint_kp
float32[12] joint_kd
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



(provide :control_msgs/joint_cmd "42cbcd630f7428dcf45148f745ff01c3")


