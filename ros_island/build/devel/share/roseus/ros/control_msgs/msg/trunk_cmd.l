;; Auto-generated. Do not edit!


(when (boundp 'control_msgs::trunk_cmd)
  (if (not (find-package "CONTROL_MSGS"))
    (make-package "CONTROL_MSGS"))
  (shadow 'trunk_cmd (find-package "CONTROL_MSGS")))
(unless (find-package "CONTROL_MSGS::TRUNK_CMD")
  (make-package "CONTROL_MSGS::TRUNK_CMD"))

(in-package "ROS")
;;//! \htmlinclude trunk_cmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass control_msgs::trunk_cmd
  :super ros::object
  :slots (_header _trunk_position_cmd _trunk_velocity_cmd _trunk_acc_cmd _trunk_kp _trunk_kd ))

(defmethod control_msgs::trunk_cmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:trunk_position_cmd __trunk_position_cmd) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:trunk_velocity_cmd __trunk_velocity_cmd) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:trunk_acc_cmd __trunk_acc_cmd) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:trunk_kp __trunk_kp) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:trunk_kd __trunk_kd) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _trunk_position_cmd __trunk_position_cmd)
   (setq _trunk_velocity_cmd __trunk_velocity_cmd)
   (setq _trunk_acc_cmd __trunk_acc_cmd)
   (setq _trunk_kp __trunk_kp)
   (setq _trunk_kd __trunk_kd)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:trunk_position_cmd
   (&optional __trunk_position_cmd)
   (if __trunk_position_cmd (setq _trunk_position_cmd __trunk_position_cmd)) _trunk_position_cmd)
  (:trunk_velocity_cmd
   (&optional __trunk_velocity_cmd)
   (if __trunk_velocity_cmd (setq _trunk_velocity_cmd __trunk_velocity_cmd)) _trunk_velocity_cmd)
  (:trunk_acc_cmd
   (&optional __trunk_acc_cmd)
   (if __trunk_acc_cmd (setq _trunk_acc_cmd __trunk_acc_cmd)) _trunk_acc_cmd)
  (:trunk_kp
   (&optional __trunk_kp)
   (if __trunk_kp (setq _trunk_kp __trunk_kp)) _trunk_kp)
  (:trunk_kd
   (&optional __trunk_kd)
   (if __trunk_kd (setq _trunk_kd __trunk_kd)) _trunk_kd)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[6] _trunk_position_cmd
    (* 4    6)
    ;; float32[6] _trunk_velocity_cmd
    (* 4    6)
    ;; float32[6] _trunk_acc_cmd
    (* 4    6)
    ;; float32[6] _trunk_kp
    (* 4    6)
    ;; float32[6] _trunk_kd
    (* 4    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[6] _trunk_position_cmd
     (dotimes (i 6)
       (sys::poke (elt _trunk_position_cmd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[6] _trunk_velocity_cmd
     (dotimes (i 6)
       (sys::poke (elt _trunk_velocity_cmd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[6] _trunk_acc_cmd
     (dotimes (i 6)
       (sys::poke (elt _trunk_acc_cmd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[6] _trunk_kp
     (dotimes (i 6)
       (sys::poke (elt _trunk_kp i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[6] _trunk_kd
     (dotimes (i 6)
       (sys::poke (elt _trunk_kd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[6] _trunk_position_cmd
   (dotimes (i (length _trunk_position_cmd))
     (setf (elt _trunk_position_cmd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[6] _trunk_velocity_cmd
   (dotimes (i (length _trunk_velocity_cmd))
     (setf (elt _trunk_velocity_cmd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[6] _trunk_acc_cmd
   (dotimes (i (length _trunk_acc_cmd))
     (setf (elt _trunk_acc_cmd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[6] _trunk_kp
   (dotimes (i (length _trunk_kp))
     (setf (elt _trunk_kp i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[6] _trunk_kd
   (dotimes (i (length _trunk_kd))
     (setf (elt _trunk_kd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get control_msgs::trunk_cmd :md5sum-) "9c7edd9abee115da7ddc97ab8e181c2e")
(setf (get control_msgs::trunk_cmd :datatype-) "control_msgs/trunk_cmd")
(setf (get control_msgs::trunk_cmd :definition-)
      "#topic:'motion/trunk/cmd'
Header header

float32[6] trunk_position_cmd
float32[6] trunk_velocity_cmd
float32[6] trunk_acc_cmd
float32[6] trunk_kp
float32[6] trunk_kd

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



(provide :control_msgs/trunk_cmd "9c7edd9abee115da7ddc97ab8e181c2e")


