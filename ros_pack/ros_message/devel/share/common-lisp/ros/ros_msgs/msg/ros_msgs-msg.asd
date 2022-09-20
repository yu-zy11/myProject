
(cl:in-package :asdf)

(defsystem "ros_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "JointCommand" :depends-on ("_package_JointCommand"))
    (:file "_package_JointCommand" :depends-on ("_package"))
    (:file "RobotStates" :depends-on ("_package_RobotStates"))
    (:file "_package_RobotStates" :depends-on ("_package"))
  ))