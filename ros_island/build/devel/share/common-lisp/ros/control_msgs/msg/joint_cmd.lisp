; Auto-generated. Do not edit!


(cl:in-package control_msgs-msg)


;//! \htmlinclude joint_cmd.msg.html

(cl:defclass <joint_cmd> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (joint_position_cmd
    :reader joint_position_cmd
    :initarg :joint_position_cmd
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0))
   (joint_velocity_cmd
    :reader joint_velocity_cmd
    :initarg :joint_velocity_cmd
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0))
   (torque_ff
    :reader torque_ff
    :initarg :torque_ff
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0))
   (joint_kp
    :reader joint_kp
    :initarg :joint_kp
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0))
   (joint_kd
    :reader joint_kd
    :initarg :joint_kd
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass joint_cmd (<joint_cmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <joint_cmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'joint_cmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_msgs-msg:<joint_cmd> is deprecated: use control_msgs-msg:joint_cmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <joint_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:header-val is deprecated.  Use control_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'joint_position_cmd-val :lambda-list '(m))
(cl:defmethod joint_position_cmd-val ((m <joint_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:joint_position_cmd-val is deprecated.  Use control_msgs-msg:joint_position_cmd instead.")
  (joint_position_cmd m))

(cl:ensure-generic-function 'joint_velocity_cmd-val :lambda-list '(m))
(cl:defmethod joint_velocity_cmd-val ((m <joint_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:joint_velocity_cmd-val is deprecated.  Use control_msgs-msg:joint_velocity_cmd instead.")
  (joint_velocity_cmd m))

(cl:ensure-generic-function 'torque_ff-val :lambda-list '(m))
(cl:defmethod torque_ff-val ((m <joint_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:torque_ff-val is deprecated.  Use control_msgs-msg:torque_ff instead.")
  (torque_ff m))

(cl:ensure-generic-function 'joint_kp-val :lambda-list '(m))
(cl:defmethod joint_kp-val ((m <joint_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:joint_kp-val is deprecated.  Use control_msgs-msg:joint_kp instead.")
  (joint_kp m))

(cl:ensure-generic-function 'joint_kd-val :lambda-list '(m))
(cl:defmethod joint_kd-val ((m <joint_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:joint_kd-val is deprecated.  Use control_msgs-msg:joint_kd instead.")
  (joint_kd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <joint_cmd>) ostream)
  "Serializes a message object of type '<joint_cmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'joint_position_cmd))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'joint_velocity_cmd))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'torque_ff))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'joint_kp))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'joint_kd))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <joint_cmd>) istream)
  "Deserializes a message object of type '<joint_cmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'joint_position_cmd) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'joint_position_cmd)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'joint_velocity_cmd) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'joint_velocity_cmd)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'torque_ff) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'torque_ff)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'joint_kp) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'joint_kp)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'joint_kd) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'joint_kd)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<joint_cmd>)))
  "Returns string type for a message object of type '<joint_cmd>"
  "control_msgs/joint_cmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'joint_cmd)))
  "Returns string type for a message object of type 'joint_cmd"
  "control_msgs/joint_cmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<joint_cmd>)))
  "Returns md5sum for a message object of type '<joint_cmd>"
  "42cbcd630f7428dcf45148f745ff01c3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'joint_cmd)))
  "Returns md5sum for a message object of type 'joint_cmd"
  "42cbcd630f7428dcf45148f745ff01c3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<joint_cmd>)))
  "Returns full string definition for message of type '<joint_cmd>"
  (cl:format cl:nil "#topic:'motion/joint/cmd'~%Header header~%~%float32[12] joint_position_cmd~%float32[12] joint_velocity_cmd~%float32[12] torque_ff~%float32[12] joint_kp~%float32[12] joint_kd~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'joint_cmd)))
  "Returns full string definition for message of type 'joint_cmd"
  (cl:format cl:nil "#topic:'motion/joint/cmd'~%Header header~%~%float32[12] joint_position_cmd~%float32[12] joint_velocity_cmd~%float32[12] torque_ff~%float32[12] joint_kp~%float32[12] joint_kd~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <joint_cmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_position_cmd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_velocity_cmd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'torque_ff) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_kp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_kd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <joint_cmd>))
  "Converts a ROS message object to a list"
  (cl:list 'joint_cmd
    (cl:cons ':header (header msg))
    (cl:cons ':joint_position_cmd (joint_position_cmd msg))
    (cl:cons ':joint_velocity_cmd (joint_velocity_cmd msg))
    (cl:cons ':torque_ff (torque_ff msg))
    (cl:cons ':joint_kp (joint_kp msg))
    (cl:cons ':joint_kd (joint_kd msg))
))
