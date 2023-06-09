; Auto-generated. Do not edit!


(cl:in-package control_msgs-msg)


;//! \htmlinclude trunk_cmd.msg.html

(cl:defclass <trunk_cmd> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (trunk_position_cmd
    :reader trunk_position_cmd
    :initarg :trunk_position_cmd
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (trunk_velocity_cmd
    :reader trunk_velocity_cmd
    :initarg :trunk_velocity_cmd
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (trunk_acc_cmd
    :reader trunk_acc_cmd
    :initarg :trunk_acc_cmd
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (trunk_kp
    :reader trunk_kp
    :initarg :trunk_kp
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (trunk_kd
    :reader trunk_kd
    :initarg :trunk_kd
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass trunk_cmd (<trunk_cmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trunk_cmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trunk_cmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_msgs-msg:<trunk_cmd> is deprecated: use control_msgs-msg:trunk_cmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <trunk_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:header-val is deprecated.  Use control_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'trunk_position_cmd-val :lambda-list '(m))
(cl:defmethod trunk_position_cmd-val ((m <trunk_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:trunk_position_cmd-val is deprecated.  Use control_msgs-msg:trunk_position_cmd instead.")
  (trunk_position_cmd m))

(cl:ensure-generic-function 'trunk_velocity_cmd-val :lambda-list '(m))
(cl:defmethod trunk_velocity_cmd-val ((m <trunk_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:trunk_velocity_cmd-val is deprecated.  Use control_msgs-msg:trunk_velocity_cmd instead.")
  (trunk_velocity_cmd m))

(cl:ensure-generic-function 'trunk_acc_cmd-val :lambda-list '(m))
(cl:defmethod trunk_acc_cmd-val ((m <trunk_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:trunk_acc_cmd-val is deprecated.  Use control_msgs-msg:trunk_acc_cmd instead.")
  (trunk_acc_cmd m))

(cl:ensure-generic-function 'trunk_kp-val :lambda-list '(m))
(cl:defmethod trunk_kp-val ((m <trunk_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:trunk_kp-val is deprecated.  Use control_msgs-msg:trunk_kp instead.")
  (trunk_kp m))

(cl:ensure-generic-function 'trunk_kd-val :lambda-list '(m))
(cl:defmethod trunk_kd-val ((m <trunk_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:trunk_kd-val is deprecated.  Use control_msgs-msg:trunk_kd instead.")
  (trunk_kd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trunk_cmd>) ostream)
  "Serializes a message object of type '<trunk_cmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'trunk_position_cmd))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'trunk_velocity_cmd))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'trunk_acc_cmd))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'trunk_kp))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'trunk_kd))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trunk_cmd>) istream)
  "Deserializes a message object of type '<trunk_cmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'trunk_position_cmd) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'trunk_position_cmd)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'trunk_velocity_cmd) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'trunk_velocity_cmd)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'trunk_acc_cmd) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'trunk_acc_cmd)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'trunk_kp) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'trunk_kp)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'trunk_kd) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'trunk_kd)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trunk_cmd>)))
  "Returns string type for a message object of type '<trunk_cmd>"
  "control_msgs/trunk_cmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trunk_cmd)))
  "Returns string type for a message object of type 'trunk_cmd"
  "control_msgs/trunk_cmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trunk_cmd>)))
  "Returns md5sum for a message object of type '<trunk_cmd>"
  "9c7edd9abee115da7ddc97ab8e181c2e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trunk_cmd)))
  "Returns md5sum for a message object of type 'trunk_cmd"
  "9c7edd9abee115da7ddc97ab8e181c2e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trunk_cmd>)))
  "Returns full string definition for message of type '<trunk_cmd>"
  (cl:format cl:nil "#topic:'motion/trunk/cmd'~%Header header~%~%float32[6] trunk_position_cmd~%float32[6] trunk_velocity_cmd~%float32[6] trunk_acc_cmd~%float32[6] trunk_kp~%float32[6] trunk_kd~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trunk_cmd)))
  "Returns full string definition for message of type 'trunk_cmd"
  (cl:format cl:nil "#topic:'motion/trunk/cmd'~%Header header~%~%float32[6] trunk_position_cmd~%float32[6] trunk_velocity_cmd~%float32[6] trunk_acc_cmd~%float32[6] trunk_kp~%float32[6] trunk_kd~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trunk_cmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'trunk_position_cmd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'trunk_velocity_cmd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'trunk_acc_cmd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'trunk_kp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'trunk_kd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trunk_cmd>))
  "Converts a ROS message object to a list"
  (cl:list 'trunk_cmd
    (cl:cons ':header (header msg))
    (cl:cons ':trunk_position_cmd (trunk_position_cmd msg))
    (cl:cons ':trunk_velocity_cmd (trunk_velocity_cmd msg))
    (cl:cons ':trunk_acc_cmd (trunk_acc_cmd msg))
    (cl:cons ':trunk_kp (trunk_kp msg))
    (cl:cons ':trunk_kd (trunk_kd msg))
))
