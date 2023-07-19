// Auto-generated. Do not edit!

// (in-package control_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class joint_state {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.joint_position = null;
      this.joint_velocity = null;
      this.torque = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('joint_position')) {
        this.joint_position = initObj.joint_position
      }
      else {
        this.joint_position = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('joint_velocity')) {
        this.joint_velocity = initObj.joint_velocity
      }
      else {
        this.joint_velocity = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('torque')) {
        this.torque = initObj.torque
      }
      else {
        this.torque = new Array(12).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type joint_state
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [joint_position] has the right length
    if (obj.joint_position.length !== 12) {
      throw new Error('Unable to serialize array field joint_position - length must be 12')
    }
    // Serialize message field [joint_position]
    bufferOffset = _arraySerializer.float32(obj.joint_position, buffer, bufferOffset, 12);
    // Check that the constant length array field [joint_velocity] has the right length
    if (obj.joint_velocity.length !== 12) {
      throw new Error('Unable to serialize array field joint_velocity - length must be 12')
    }
    // Serialize message field [joint_velocity]
    bufferOffset = _arraySerializer.float32(obj.joint_velocity, buffer, bufferOffset, 12);
    // Check that the constant length array field [torque] has the right length
    if (obj.torque.length !== 12) {
      throw new Error('Unable to serialize array field torque - length must be 12')
    }
    // Serialize message field [torque]
    bufferOffset = _arraySerializer.float32(obj.torque, buffer, bufferOffset, 12);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type joint_state
    let len;
    let data = new joint_state(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint_position]
    data.joint_position = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [joint_velocity]
    data.joint_velocity = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [torque]
    data.torque = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 144;
  }

  static datatype() {
    // Returns string type for a message object
    return 'control_msgs/joint_state';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cee320b1b25101ce84cadf5dd27b382f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #topic:'motion/joint/state'
    Header header
    
    float32[12] joint_position
    float32[12] joint_velocity
    float32[12] torque
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new joint_state(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.joint_position !== undefined) {
      resolved.joint_position = msg.joint_position;
    }
    else {
      resolved.joint_position = new Array(12).fill(0)
    }

    if (msg.joint_velocity !== undefined) {
      resolved.joint_velocity = msg.joint_velocity;
    }
    else {
      resolved.joint_velocity = new Array(12).fill(0)
    }

    if (msg.torque !== undefined) {
      resolved.torque = msg.torque;
    }
    else {
      resolved.torque = new Array(12).fill(0)
    }

    return resolved;
    }
};

module.exports = joint_state;
