;; Auto-generated. Do not edit!


(when (boundp 'ros_msgs::RobotStates)
  (if (not (find-package "ROS_MSGS"))
    (make-package "ROS_MSGS"))
  (shadow 'RobotStates (find-package "ROS_MSGS")))
(unless (find-package "ROS_MSGS::ROBOTSTATES")
  (make-package "ROS_MSGS::ROBOTSTATES"))

(in-package "ROS")
;;//! \htmlinclude RobotStates.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ros_msgs::RobotStates
  :super ros::object
  :slots (_header _body_position _body_linear_velocity_in_world _body_linear_velocity_in_body _body_acceleration_in_body _body_euler _body_quaternion _body_angular_velocity_in_world _body_angular_velocity_in_body _rotation_matrix_body_to_world _joint_position _joint_velocity _torque_reading _joint_acceleration _foot_contact_force _foot_contact_detected _foot_contact_probability _foot_position_in_body _foot_velocity_in_body _foot_jacobian_in_body ))

(defmethod ros_msgs::RobotStates
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:body_position __body_position) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:body_linear_velocity_in_world __body_linear_velocity_in_world) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:body_linear_velocity_in_body __body_linear_velocity_in_body) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:body_acceleration_in_body __body_acceleration_in_body) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:body_euler __body_euler) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:body_quaternion __body_quaternion) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:body_angular_velocity_in_world __body_angular_velocity_in_world) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:body_angular_velocity_in_body __body_angular_velocity_in_body) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:rotation_matrix_body_to_world __rotation_matrix_body_to_world) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:joint_position __joint_position) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:joint_velocity __joint_velocity) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:torque_reading __torque_reading) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:joint_acceleration __joint_acceleration) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:foot_contact_force __foot_contact_force) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:foot_contact_detected __foot_contact_detected) (make-array 4 :initial-element 0 :element-type :integer))
    ((:foot_contact_probability __foot_contact_probability) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:foot_position_in_body __foot_position_in_body) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:foot_velocity_in_body __foot_velocity_in_body) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:foot_jacobian_in_body __foot_jacobian_in_body) (make-array 144 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _body_position __body_position)
   (setq _body_linear_velocity_in_world __body_linear_velocity_in_world)
   (setq _body_linear_velocity_in_body __body_linear_velocity_in_body)
   (setq _body_acceleration_in_body __body_acceleration_in_body)
   (setq _body_euler __body_euler)
   (setq _body_quaternion __body_quaternion)
   (setq _body_angular_velocity_in_world __body_angular_velocity_in_world)
   (setq _body_angular_velocity_in_body __body_angular_velocity_in_body)
   (setq _rotation_matrix_body_to_world __rotation_matrix_body_to_world)
   (setq _joint_position __joint_position)
   (setq _joint_velocity __joint_velocity)
   (setq _torque_reading __torque_reading)
   (setq _joint_acceleration __joint_acceleration)
   (setq _foot_contact_force __foot_contact_force)
   (setq _foot_contact_detected __foot_contact_detected)
   (setq _foot_contact_probability __foot_contact_probability)
   (setq _foot_position_in_body __foot_position_in_body)
   (setq _foot_velocity_in_body __foot_velocity_in_body)
   (setq _foot_jacobian_in_body __foot_jacobian_in_body)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:body_position
   (&optional __body_position)
   (if __body_position (setq _body_position __body_position)) _body_position)
  (:body_linear_velocity_in_world
   (&optional __body_linear_velocity_in_world)
   (if __body_linear_velocity_in_world (setq _body_linear_velocity_in_world __body_linear_velocity_in_world)) _body_linear_velocity_in_world)
  (:body_linear_velocity_in_body
   (&optional __body_linear_velocity_in_body)
   (if __body_linear_velocity_in_body (setq _body_linear_velocity_in_body __body_linear_velocity_in_body)) _body_linear_velocity_in_body)
  (:body_acceleration_in_body
   (&optional __body_acceleration_in_body)
   (if __body_acceleration_in_body (setq _body_acceleration_in_body __body_acceleration_in_body)) _body_acceleration_in_body)
  (:body_euler
   (&optional __body_euler)
   (if __body_euler (setq _body_euler __body_euler)) _body_euler)
  (:body_quaternion
   (&optional __body_quaternion)
   (if __body_quaternion (setq _body_quaternion __body_quaternion)) _body_quaternion)
  (:body_angular_velocity_in_world
   (&optional __body_angular_velocity_in_world)
   (if __body_angular_velocity_in_world (setq _body_angular_velocity_in_world __body_angular_velocity_in_world)) _body_angular_velocity_in_world)
  (:body_angular_velocity_in_body
   (&optional __body_angular_velocity_in_body)
   (if __body_angular_velocity_in_body (setq _body_angular_velocity_in_body __body_angular_velocity_in_body)) _body_angular_velocity_in_body)
  (:rotation_matrix_body_to_world
   (&optional __rotation_matrix_body_to_world)
   (if __rotation_matrix_body_to_world (setq _rotation_matrix_body_to_world __rotation_matrix_body_to_world)) _rotation_matrix_body_to_world)
  (:joint_position
   (&optional __joint_position)
   (if __joint_position (setq _joint_position __joint_position)) _joint_position)
  (:joint_velocity
   (&optional __joint_velocity)
   (if __joint_velocity (setq _joint_velocity __joint_velocity)) _joint_velocity)
  (:torque_reading
   (&optional __torque_reading)
   (if __torque_reading (setq _torque_reading __torque_reading)) _torque_reading)
  (:joint_acceleration
   (&optional __joint_acceleration)
   (if __joint_acceleration (setq _joint_acceleration __joint_acceleration)) _joint_acceleration)
  (:foot_contact_force
   (&optional __foot_contact_force)
   (if __foot_contact_force (setq _foot_contact_force __foot_contact_force)) _foot_contact_force)
  (:foot_contact_detected
   (&optional __foot_contact_detected)
   (if __foot_contact_detected (setq _foot_contact_detected __foot_contact_detected)) _foot_contact_detected)
  (:foot_contact_probability
   (&optional __foot_contact_probability)
   (if __foot_contact_probability (setq _foot_contact_probability __foot_contact_probability)) _foot_contact_probability)
  (:foot_position_in_body
   (&optional __foot_position_in_body)
   (if __foot_position_in_body (setq _foot_position_in_body __foot_position_in_body)) _foot_position_in_body)
  (:foot_velocity_in_body
   (&optional __foot_velocity_in_body)
   (if __foot_velocity_in_body (setq _foot_velocity_in_body __foot_velocity_in_body)) _foot_velocity_in_body)
  (:foot_jacobian_in_body
   (&optional __foot_jacobian_in_body)
   (if __foot_jacobian_in_body (setq _foot_jacobian_in_body __foot_jacobian_in_body)) _foot_jacobian_in_body)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[3] _body_position
    (* 4    3)
    ;; float32[3] _body_linear_velocity_in_world
    (* 4    3)
    ;; float32[3] _body_linear_velocity_in_body
    (* 4    3)
    ;; float32[3] _body_acceleration_in_body
    (* 4    3)
    ;; float32[3] _body_euler
    (* 4    3)
    ;; float32[4] _body_quaternion
    (* 4    4)
    ;; float32[3] _body_angular_velocity_in_world
    (* 4    3)
    ;; float32[3] _body_angular_velocity_in_body
    (* 4    3)
    ;; float32[9] _rotation_matrix_body_to_world
    (* 4    9)
    ;; float32[12] _joint_position
    (* 4    12)
    ;; float32[12] _joint_velocity
    (* 4    12)
    ;; float32[12] _torque_reading
    (* 4    12)
    ;; float32[12] _joint_acceleration
    (* 4    12)
    ;; float32[4] _foot_contact_force
    (* 4    4)
    ;; int32[4] _foot_contact_detected
    (* 4    4)
    ;; float32[4] _foot_contact_probability
    (* 4    4)
    ;; float32[12] _foot_position_in_body
    (* 4    12)
    ;; float32[12] _foot_velocity_in_body
    (* 4    12)
    ;; float32[144] _foot_jacobian_in_body
    (* 4    144)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[3] _body_position
     (dotimes (i 3)
       (sys::poke (elt _body_position i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _body_linear_velocity_in_world
     (dotimes (i 3)
       (sys::poke (elt _body_linear_velocity_in_world i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _body_linear_velocity_in_body
     (dotimes (i 3)
       (sys::poke (elt _body_linear_velocity_in_body i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _body_acceleration_in_body
     (dotimes (i 3)
       (sys::poke (elt _body_acceleration_in_body i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _body_euler
     (dotimes (i 3)
       (sys::poke (elt _body_euler i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[4] _body_quaternion
     (dotimes (i 4)
       (sys::poke (elt _body_quaternion i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _body_angular_velocity_in_world
     (dotimes (i 3)
       (sys::poke (elt _body_angular_velocity_in_world i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _body_angular_velocity_in_body
     (dotimes (i 3)
       (sys::poke (elt _body_angular_velocity_in_body i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[9] _rotation_matrix_body_to_world
     (dotimes (i 9)
       (sys::poke (elt _rotation_matrix_body_to_world i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _joint_position
     (dotimes (i 12)
       (sys::poke (elt _joint_position i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _joint_velocity
     (dotimes (i 12)
       (sys::poke (elt _joint_velocity i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _torque_reading
     (dotimes (i 12)
       (sys::poke (elt _torque_reading i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _joint_acceleration
     (dotimes (i 12)
       (sys::poke (elt _joint_acceleration i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[4] _foot_contact_force
     (dotimes (i 4)
       (sys::poke (elt _foot_contact_force i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int32[4] _foot_contact_detected
     (dotimes (i 4)
       (write-long (elt _foot_contact_detected i) s)
       )
     ;; float32[4] _foot_contact_probability
     (dotimes (i 4)
       (sys::poke (elt _foot_contact_probability i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _foot_position_in_body
     (dotimes (i 12)
       (sys::poke (elt _foot_position_in_body i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[12] _foot_velocity_in_body
     (dotimes (i 12)
       (sys::poke (elt _foot_velocity_in_body i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[144] _foot_jacobian_in_body
     (dotimes (i 144)
       (sys::poke (elt _foot_jacobian_in_body i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[3] _body_position
   (dotimes (i (length _body_position))
     (setf (elt _body_position i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _body_linear_velocity_in_world
   (dotimes (i (length _body_linear_velocity_in_world))
     (setf (elt _body_linear_velocity_in_world i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _body_linear_velocity_in_body
   (dotimes (i (length _body_linear_velocity_in_body))
     (setf (elt _body_linear_velocity_in_body i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _body_acceleration_in_body
   (dotimes (i (length _body_acceleration_in_body))
     (setf (elt _body_acceleration_in_body i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _body_euler
   (dotimes (i (length _body_euler))
     (setf (elt _body_euler i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[4] _body_quaternion
   (dotimes (i (length _body_quaternion))
     (setf (elt _body_quaternion i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _body_angular_velocity_in_world
   (dotimes (i (length _body_angular_velocity_in_world))
     (setf (elt _body_angular_velocity_in_world i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _body_angular_velocity_in_body
   (dotimes (i (length _body_angular_velocity_in_body))
     (setf (elt _body_angular_velocity_in_body i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[9] _rotation_matrix_body_to_world
   (dotimes (i (length _rotation_matrix_body_to_world))
     (setf (elt _rotation_matrix_body_to_world i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _joint_position
   (dotimes (i (length _joint_position))
     (setf (elt _joint_position i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _joint_velocity
   (dotimes (i (length _joint_velocity))
     (setf (elt _joint_velocity i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _torque_reading
   (dotimes (i (length _torque_reading))
     (setf (elt _torque_reading i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _joint_acceleration
   (dotimes (i (length _joint_acceleration))
     (setf (elt _joint_acceleration i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[4] _foot_contact_force
   (dotimes (i (length _foot_contact_force))
     (setf (elt _foot_contact_force i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; int32[4] _foot_contact_detected
   (dotimes (i (length _foot_contact_detected))
     (setf (elt _foot_contact_detected i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; float32[4] _foot_contact_probability
   (dotimes (i (length _foot_contact_probability))
     (setf (elt _foot_contact_probability i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _foot_position_in_body
   (dotimes (i (length _foot_position_in_body))
     (setf (elt _foot_position_in_body i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[12] _foot_velocity_in_body
   (dotimes (i (length _foot_velocity_in_body))
     (setf (elt _foot_velocity_in_body i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[144] _foot_jacobian_in_body
   (dotimes (i (length _foot_jacobian_in_body))
     (setf (elt _foot_jacobian_in_body i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get ros_msgs::RobotStates :md5sum-) "a8b0d3c2e9f9adf1e85eb4959a9e0667")
(setf (get ros_msgs::RobotStates :datatype-) "ros_msgs/RobotStates")
(setf (get ros_msgs::RobotStates :definition-)
      "Header header
float32[3] body_position
float32[3] body_linear_velocity_in_world
float32[3] body_linear_velocity_in_body
float32[3] body_acceleration_in_body
float32[3] body_euler
float32[4] body_quaternion
float32[3] body_angular_velocity_in_world
float32[3] body_angular_velocity_in_body
float32[9] rotation_matrix_body_to_world
float32[12] joint_position
float32[12] joint_velocity
float32[12] torque_reading
float32[12] joint_acceleration
float32[4] foot_contact_force
int32[4] foot_contact_detected
float32[4] foot_contact_probability
float32[12] foot_position_in_body
float32[12] foot_velocity_in_body
float32[144] foot_jacobian_in_body

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



(provide :ros_msgs/RobotStates "a8b0d3c2e9f9adf1e85eb4959a9e0667")


