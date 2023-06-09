; Auto-generated. Do not edit!


(cl:in-package control_msgs-msg)


;//! \htmlinclude foot_state.msg.html

(cl:defclass <foot_state> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (foot_position
    :reader foot_position
    :initarg :foot_position
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0))
   (foot_velocity
    :reader foot_velocity
    :initarg :foot_velocity
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0))
   (contact_state
    :reader contact_state
    :initarg :contact_state
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (foot_force
    :reader foot_force
    :initarg :foot_force
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass foot_state (<foot_state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <foot_state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'foot_state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_msgs-msg:<foot_state> is deprecated: use control_msgs-msg:foot_state instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <foot_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:header-val is deprecated.  Use control_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'foot_position-val :lambda-list '(m))
(cl:defmethod foot_position-val ((m <foot_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:foot_position-val is deprecated.  Use control_msgs-msg:foot_position instead.")
  (foot_position m))

(cl:ensure-generic-function 'foot_velocity-val :lambda-list '(m))
(cl:defmethod foot_velocity-val ((m <foot_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:foot_velocity-val is deprecated.  Use control_msgs-msg:foot_velocity instead.")
  (foot_velocity m))

(cl:ensure-generic-function 'contact_state-val :lambda-list '(m))
(cl:defmethod contact_state-val ((m <foot_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:contact_state-val is deprecated.  Use control_msgs-msg:contact_state instead.")
  (contact_state m))

(cl:ensure-generic-function 'foot_force-val :lambda-list '(m))
(cl:defmethod foot_force-val ((m <foot_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:foot_force-val is deprecated.  Use control_msgs-msg:foot_force instead.")
  (foot_force m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <foot_state>) ostream)
  "Serializes a message object of type '<foot_state>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'foot_position))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'foot_velocity))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'contact_state))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'foot_force))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <foot_state>) istream)
  "Deserializes a message object of type '<foot_state>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'foot_position) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'foot_position)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'foot_velocity) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'foot_velocity)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'contact_state) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'contact_state)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'foot_force) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'foot_force)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<foot_state>)))
  "Returns string type for a message object of type '<foot_state>"
  "control_msgs/foot_state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'foot_state)))
  "Returns string type for a message object of type 'foot_state"
  "control_msgs/foot_state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<foot_state>)))
  "Returns md5sum for a message object of type '<foot_state>"
  "ed2e25b8bdf234bee1b653ec362f2558")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'foot_state)))
  "Returns md5sum for a message object of type 'foot_state"
  "ed2e25b8bdf234bee1b653ec362f2558")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<foot_state>)))
  "Returns full string definition for message of type '<foot_state>"
  (cl:format cl:nil "#topic:'motion/foot/state'~%Header header~%~%float32[12] foot_position~%float32[12] foot_velocity~%float32[4]  contact_state~%float32[12] foot_force~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'foot_state)))
  "Returns full string definition for message of type 'foot_state"
  (cl:format cl:nil "#topic:'motion/foot/state'~%Header header~%~%float32[12] foot_position~%float32[12] foot_velocity~%float32[4]  contact_state~%float32[12] foot_force~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <foot_state>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'foot_position) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'foot_velocity) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'contact_state) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'foot_force) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <foot_state>))
  "Converts a ROS message object to a list"
  (cl:list 'foot_state
    (cl:cons ':header (header msg))
    (cl:cons ':foot_position (foot_position msg))
    (cl:cons ':foot_velocity (foot_velocity msg))
    (cl:cons ':contact_state (contact_state msg))
    (cl:cons ':foot_force (foot_force msg))
))
