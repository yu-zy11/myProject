; Auto-generated. Do not edit!


(cl:in-package control_msgs-msg)


;//! \htmlinclude trunk_state.msg.html

(cl:defclass <trunk_state> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (trunk_position
    :reader trunk_position
    :initarg :trunk_position
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (trunk_velocity
    :reader trunk_velocity
    :initarg :trunk_velocity
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass trunk_state (<trunk_state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trunk_state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trunk_state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_msgs-msg:<trunk_state> is deprecated: use control_msgs-msg:trunk_state instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <trunk_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:header-val is deprecated.  Use control_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'trunk_position-val :lambda-list '(m))
(cl:defmethod trunk_position-val ((m <trunk_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:trunk_position-val is deprecated.  Use control_msgs-msg:trunk_position instead.")
  (trunk_position m))

(cl:ensure-generic-function 'trunk_velocity-val :lambda-list '(m))
(cl:defmethod trunk_velocity-val ((m <trunk_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:trunk_velocity-val is deprecated.  Use control_msgs-msg:trunk_velocity instead.")
  (trunk_velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trunk_state>) ostream)
  "Serializes a message object of type '<trunk_state>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'trunk_position))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'trunk_velocity))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trunk_state>) istream)
  "Deserializes a message object of type '<trunk_state>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'trunk_position) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'trunk_position)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'trunk_velocity) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'trunk_velocity)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trunk_state>)))
  "Returns string type for a message object of type '<trunk_state>"
  "control_msgs/trunk_state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trunk_state)))
  "Returns string type for a message object of type 'trunk_state"
  "control_msgs/trunk_state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trunk_state>)))
  "Returns md5sum for a message object of type '<trunk_state>"
  "d6da26616d9cb3c34772795c92291a45")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trunk_state)))
  "Returns md5sum for a message object of type 'trunk_state"
  "d6da26616d9cb3c34772795c92291a45")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trunk_state>)))
  "Returns full string definition for message of type '<trunk_state>"
  (cl:format cl:nil "#topic:'motion/trunk/state'~%Header header~%~%float32[6] trunk_position~%float32[6] trunk_velocity~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trunk_state)))
  "Returns full string definition for message of type 'trunk_state"
  (cl:format cl:nil "#topic:'motion/trunk/state'~%Header header~%~%float32[6] trunk_position~%float32[6] trunk_velocity~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trunk_state>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'trunk_position) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'trunk_velocity) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trunk_state>))
  "Converts a ROS message object to a list"
  (cl:list 'trunk_state
    (cl:cons ':header (header msg))
    (cl:cons ':trunk_position (trunk_position msg))
    (cl:cons ':trunk_velocity (trunk_velocity msg))
))
