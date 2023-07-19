; Auto-generated. Do not edit!


(cl:in-package control_msgs-msg)


;//! \htmlinclude foot_cmd.msg.html

(cl:defclass <foot_cmd> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (foot_position_cmd
    :reader foot_position_cmd
    :initarg :foot_position_cmd
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0))
   (foot_velocity_cmd
    :reader foot_velocity_cmd
    :initarg :foot_velocity_cmd
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0))
   (foot_acc_cmd
    :reader foot_acc_cmd
    :initarg :foot_acc_cmd
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0))
   (contact_target
    :reader contact_target
    :initarg :contact_target
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (foot_force_ff
    :reader foot_force_ff
    :initarg :foot_force_ff
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0))
   (foot_kp
    :reader foot_kp
    :initarg :foot_kp
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0))
   (foot_kd
    :reader foot_kd
    :initarg :foot_kd
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass foot_cmd (<foot_cmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <foot_cmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'foot_cmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_msgs-msg:<foot_cmd> is deprecated: use control_msgs-msg:foot_cmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <foot_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:header-val is deprecated.  Use control_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'foot_position_cmd-val :lambda-list '(m))
(cl:defmethod foot_position_cmd-val ((m <foot_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:foot_position_cmd-val is deprecated.  Use control_msgs-msg:foot_position_cmd instead.")
  (foot_position_cmd m))

(cl:ensure-generic-function 'foot_velocity_cmd-val :lambda-list '(m))
(cl:defmethod foot_velocity_cmd-val ((m <foot_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:foot_velocity_cmd-val is deprecated.  Use control_msgs-msg:foot_velocity_cmd instead.")
  (foot_velocity_cmd m))

(cl:ensure-generic-function 'foot_acc_cmd-val :lambda-list '(m))
(cl:defmethod foot_acc_cmd-val ((m <foot_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:foot_acc_cmd-val is deprecated.  Use control_msgs-msg:foot_acc_cmd instead.")
  (foot_acc_cmd m))

(cl:ensure-generic-function 'contact_target-val :lambda-list '(m))
(cl:defmethod contact_target-val ((m <foot_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:contact_target-val is deprecated.  Use control_msgs-msg:contact_target instead.")
  (contact_target m))

(cl:ensure-generic-function 'foot_force_ff-val :lambda-list '(m))
(cl:defmethod foot_force_ff-val ((m <foot_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:foot_force_ff-val is deprecated.  Use control_msgs-msg:foot_force_ff instead.")
  (foot_force_ff m))

(cl:ensure-generic-function 'foot_kp-val :lambda-list '(m))
(cl:defmethod foot_kp-val ((m <foot_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:foot_kp-val is deprecated.  Use control_msgs-msg:foot_kp instead.")
  (foot_kp m))

(cl:ensure-generic-function 'foot_kd-val :lambda-list '(m))
(cl:defmethod foot_kd-val ((m <foot_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:foot_kd-val is deprecated.  Use control_msgs-msg:foot_kd instead.")
  (foot_kd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <foot_cmd>) ostream)
  "Serializes a message object of type '<foot_cmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'foot_position_cmd))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'foot_velocity_cmd))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'foot_acc_cmd))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'contact_target))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'foot_force_ff))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'foot_kp))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'foot_kd))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <foot_cmd>) istream)
  "Deserializes a message object of type '<foot_cmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'foot_position_cmd) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'foot_position_cmd)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'foot_velocity_cmd) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'foot_velocity_cmd)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'foot_acc_cmd) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'foot_acc_cmd)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'contact_target) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'contact_target)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'foot_force_ff) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'foot_force_ff)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'foot_kp) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'foot_kp)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'foot_kd) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'foot_kd)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<foot_cmd>)))
  "Returns string type for a message object of type '<foot_cmd>"
  "control_msgs/foot_cmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'foot_cmd)))
  "Returns string type for a message object of type 'foot_cmd"
  "control_msgs/foot_cmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<foot_cmd>)))
  "Returns md5sum for a message object of type '<foot_cmd>"
  "adaaf980547d2f8d216cbda68b41af75")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'foot_cmd)))
  "Returns md5sum for a message object of type 'foot_cmd"
  "adaaf980547d2f8d216cbda68b41af75")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<foot_cmd>)))
  "Returns full string definition for message of type '<foot_cmd>"
  (cl:format cl:nil "#topic:'motion/foot/cmd'~%Header header~%~%float32[12] foot_position_cmd~%float32[12] foot_velocity_cmd~%float32[12] foot_acc_cmd~%float32[4]  contact_target~%float32[12] foot_force_ff~%float32[12] foot_kp~%float32[12] foot_kd~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'foot_cmd)))
  "Returns full string definition for message of type 'foot_cmd"
  (cl:format cl:nil "#topic:'motion/foot/cmd'~%Header header~%~%float32[12] foot_position_cmd~%float32[12] foot_velocity_cmd~%float32[12] foot_acc_cmd~%float32[4]  contact_target~%float32[12] foot_force_ff~%float32[12] foot_kp~%float32[12] foot_kd~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <foot_cmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'foot_position_cmd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'foot_velocity_cmd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'foot_acc_cmd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'contact_target) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'foot_force_ff) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'foot_kp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'foot_kd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <foot_cmd>))
  "Converts a ROS message object to a list"
  (cl:list 'foot_cmd
    (cl:cons ':header (header msg))
    (cl:cons ':foot_position_cmd (foot_position_cmd msg))
    (cl:cons ':foot_velocity_cmd (foot_velocity_cmd msg))
    (cl:cons ':foot_acc_cmd (foot_acc_cmd msg))
    (cl:cons ':contact_target (contact_target msg))
    (cl:cons ':foot_force_ff (foot_force_ff msg))
    (cl:cons ':foot_kp (foot_kp msg))
    (cl:cons ':foot_kd (foot_kd msg))
))
