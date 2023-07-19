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

class joint_cmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.joint_position_cmd = null;
      this.joint_velocity_cmd = null;
      this.torque_ff = null;
      this.joint_kp = null;
      this.joint_kd = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('joint_position_cmd')) {
        this.joint_position_cmd = initObj.joint_position_cmd
      }
      else {
        this.joint_position_cmd = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('joint_velocity_cmd')) {
        this.joint_velocity_cmd = initObj.joint_velocity_cmd
      }
      else {
        this.joint_velocity_cmd = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('torque_ff')) {
        this.torque_ff = initObj.torque_ff
      }
      else {
        this.torque_ff = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('joint_kp')) {
        this.joint_kp = initObj.joint_kp
      }
      else {
        this.joint_kp = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('joint_kd')) {
        this.joint_kd = initObj.joint_kd
      }
      else {
        this.joint_kd = new Array(12).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type joint_cmd
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [joint_position_cmd] has the right length
    if (obj.joint_position_cmd.length !== 12) {
      throw new Error('Unable to serialize array field joint_position_cmd - length must be 12')
    }
    // Serialize message field [joint_position_cmd]
    bufferOffset = _arraySerializer.float32(obj.joint_position_cmd, buffer, bufferOffset, 12);
    // Check that the constant length array field [joint_velocity_cmd] has the right length
    if (obj.joint_velocity_cmd.length !== 12) {
      throw new Error('Unable to serialize array field joint_velocity_cmd - length must be 12')
    }
    // Serialize message field [joint_velocity_cmd]
    bufferOffset = _arraySerializer.float32(obj.joint_velocity_cmd, buffer, bufferOffset, 12);
    // Check that the constant length array field [torque_ff] has the right length
    if (obj.torque_ff.length !== 12) {
      throw new Error('Unable to serialize array field torque_ff - length must be 12')
    }
    // Serialize message field [torque_ff]
    bufferOffset = _arraySerializer.float32(obj.torque_ff, buffer, bufferOffset, 12);
    // Check that the constant length array field [joint_kp] has the right length
    if (obj.joint_kp.length !== 12) {
      throw new Error('Unable to serialize array field joint_kp - length must be 12')
    }
    // Serialize message field [joint_kp]
    bufferOffset = _arraySerializer.float32(obj.joint_kp, buffer, bufferOffset, 12);
    // Check that the constant length array field [joint_kd] has the right length
    if (obj.joint_kd.length !== 12) {
      throw new Error('Unable to serialize array field joint_kd - length must be 12')
    }
    // Serialize message field [joint_kd]
    bufferOffset = _arraySerializer.float32(obj.joint_kd, buffer, bufferOffset, 12);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type joint_cmd
    let len;
    let data = new joint_cmd(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint_position_cmd]
    data.joint_position_cmd = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [joint_velocity_cmd]
    data.joint_velocity_cmd = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [torque_ff]
    data.torque_ff = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [joint_kp]
    data.joint_kp = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [joint_kd]
    data.joint_kd = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 240;
  }

  static datatype() {
    // Returns string type for a message object
    return 'control_msgs/joint_cmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '42cbcd630f7428dcf45148f745ff01c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #topic:'motion/joint/cmd'
    Header header
    
    float32[12] joint_position_cmd
    float32[12] joint_velocity_cmd
    float32[12] torque_ff
    float32[12] joint_kp
    float32[12] joint_kd
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
    const resolved = new joint_cmd(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.joint_position_cmd !== undefined) {
      resolved.joint_position_cmd = msg.joint_position_cmd;
    }
    else {
      resolved.joint_position_cmd = new Array(12).fill(0)
    }

    if (msg.joint_velocity_cmd !== undefined) {
      resolved.joint_velocity_cmd = msg.joint_velocity_cmd;
    }
    else {
      resolved.joint_velocity_cmd = new Array(12).fill(0)
    }

    if (msg.torque_ff !== undefined) {
      resolved.torque_ff = msg.torque_ff;
    }
    else {
      resolved.torque_ff = new Array(12).fill(0)
    }

    if (msg.joint_kp !== undefined) {
      resolved.joint_kp = msg.joint_kp;
    }
    else {
      resolved.joint_kp = new Array(12).fill(0)
    }

    if (msg.joint_kd !== undefined) {
      resolved.joint_kd = msg.joint_kd;
    }
    else {
      resolved.joint_kd = new Array(12).fill(0)
    }

    return resolved;
    }
};

module.exports = joint_cmd;
