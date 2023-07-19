;; Auto-generated. Do not edit!


(when (boundp 'control_msgs::foot_cmd)
  (if (not (find-package "CONTROL_MSGS"))
    (make-package "CONTROL_MSGS"))
  (shadow 'foot_cmd (find-package "CONTROL_MSGS")))
(unless (find-package "CONTROL_MSGS::FOOT_CMD")
  (make-package "CONTROL_MSGS::FOOT_CMD"))

(in-package "ROS")
;;//! \htmlinclude foot_cmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass control_msgs::foot_cmd
  :super ros::object
  :slots (_header _foot_position_cmd _foot_velocity_cmd _foot_acc_cmd _contact_target _foot_force_ff _foot_kp _foot_kd ))

(defmethod control_msgs::foot_cmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:foot_position_cmd __foot_position_cmd) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:foot_velocity_cmd __foot_velocity_cmd) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:foot_acc_cmd __foot_acc_cmd) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:contact_target __contact_target) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:foot_force_ff __foot_force_ff) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:foot_kp __foot_kp) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:foot_kd __foot_kd) (make-array 12 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _foot_position_cmd __foot_position_cmd)
   (setq _foot_velocity_cmd __foot_velocity_cmd)
   (setq _foot_acc_cmd __foot_acc_cmd)
   (setq _contact_target __contact_target)
   (setq _foot_force_ff __foot_force_ff)
   (setq _foot_kp __foot_kp)
   (setq _foot_kd __foot_kd)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:foot_position_cmd
   (&optional __foot_position_cmd)
   (if __foot_position_cmd (setq _foot_position_cmd __foot_position_cmd)) _foot_position_cmd)
  (:foot_velocity_cmd
   (&optional __foot_velocity_cmd)
   (if __foot_velocity_cmd (setq _foot_velocity_cmd __foot_velocity_cmd)) _foot_velocity_cmd)
  (:foot_acc_cmd
   (&optional __foot_acc_cmd)
   (if __foot_acc_cmd (setq _foot_acc_cmd __foot_acc_cmd)) _foot_acc_cmd)
  (:contact_target
   (&optional __contact_target)
   (if __contact_target (setq _contact_target __contact_target)) _contact_target)
  (:foot_force_ff
   (&optional __foot_force_ff)
   (if __foot_force_ff (setq _foot_force_ff __foot_force_ff)) _foot_force_ff)
  (:foot_kp
   (&optional __foot_kp)
   (if __foot_kp (setq _foot_kp __foot_kp)) _foot_kp)
  (:foot_kd
   (&optional __foot_kd)
   (if __foot_kd (setq _foot_kd __foot_kd)) _foot_kd)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[12] _foot_position_cmd
    (* 4    12)
    ;; float32[12] _foot_velocity_cmd
    (* 4    12)
    ;; float32[12] _foot_acc_cmd
    (* 4    12)
    ;; float32[4] _contact_target
    (* 4    4)
    ;; float32[12] _foot_force_ff
    (* 4    12)
    ;; float32[12] _foot_kp
    (* 4    12)
    ;; float32[12] _foot_kd
    (* 4    12)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[12] _foot_position_cmd
     (dotimes (i 12)
       (sys::poke (elt _foot_position_cmd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _foot_velocity_cmd
     (dotimes (i 12)
       (sys::poke (elt _foot_velocity_cmd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _foot_acc_cmd
     (dotimes (i 12)
       (sys::poke (elt _foot_acc_cmd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[4] _contact_target
     (dotimes (i 4)
       (sys::poke (elt _contact_target i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _foot_force_ff
     (dotimes (i 12)
       (sys::poke (elt _foot_force_ff i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _foot_kp
     (dotimes (i 12)
       (sys::poke (elt _foot_kp i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _foot_kd
     (dotimes (i 12)
       (sys::poke (elt _foot_kd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[12] _foot_position_cmd
   (dotimes (i (length _foot_position_cmd))
     (setf (elt _foot_position_cmd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _foot_velocity_cmd
   (dotimes (i (length _foot_velocity_cmd))
     (setf (elt _foot_velocity_cmd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _foot_acc_cmd
   (dotimes (i (length _foot_acc_cmd))
     (setf (elt _foot_acc_cmd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[4] _contact_target
   (dotimes (i (length _contact_target))
     (setf (elt _contact_target i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _foot_force_ff
   (dotimes (i (length _foot_force_ff))
     (setf (elt _foot_force_ff i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _foot_kp
   (dotimes (i (length _foot_kp))
     (setf (elt _foot_kp i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _foot_kd
   (dotimes (i (length _foot_kd))
     (setf (elt _foot_kd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get control_msgs::foot_cmd :md5sum-) "adaaf980547d2f8d216cbda68b41af75")
(setf (get control_msgs::foot_cmd :datatype-) "control_msgs/foot_cmd")
(setf (get control_msgs::foot_cmd :definition-)
      "#topic:'motion/foot/cmd'
Header header

float32[12] foot_position_cmd
float32[12] foot_velocity_cmd
float32[12] foot_acc_cmd
float32[4]  contact_target
float32[12] foot_force_ff
float32[12] foot_kp
float32[12] foot_kd


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



(provide :control_msgs/foot_cmd "adaaf980547d2f8d216cbda68b41af75")


