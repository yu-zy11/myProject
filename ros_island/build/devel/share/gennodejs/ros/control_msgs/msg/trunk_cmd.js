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

class trunk_cmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.trunk_position_cmd = null;
      this.trunk_velocity_cmd = null;
      this.trunk_acc_cmd = null;
      this.trunk_kp = null;
      this.trunk_kd = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('trunk_position_cmd')) {
        this.trunk_position_cmd = initObj.trunk_position_cmd
      }
      else {
        this.trunk_position_cmd = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('trunk_velocity_cmd')) {
        this.trunk_velocity_cmd = initObj.trunk_velocity_cmd
      }
      else {
        this.trunk_velocity_cmd = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('trunk_acc_cmd')) {
        this.trunk_acc_cmd = initObj.trunk_acc_cmd
      }
      else {
        this.trunk_acc_cmd = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('trunk_kp')) {
        this.trunk_kp = initObj.trunk_kp
      }
      else {
        this.trunk_kp = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('trunk_kd')) {
        this.trunk_kd = initObj.trunk_kd
      }
      else {
        this.trunk_kd = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type trunk_cmd
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [trunk_position_cmd] has the right length
    if (obj.trunk_position_cmd.length !== 6) {
      throw new Error('Unable to serialize array field trunk_position_cmd - length must be 6')
    }
    // Serialize message field [trunk_position_cmd]
    bufferOffset = _arraySerializer.float32(obj.trunk_position_cmd, buffer, bufferOffset, 6);
    // Check that the constant length array field [trunk_velocity_cmd] has the right length
    if (obj.trunk_velocity_cmd.length !== 6) {
      throw new Error('Unable to serialize array field trunk_velocity_cmd - length must be 6')
    }
    // Serialize message field [trunk_velocity_cmd]
    bufferOffset = _arraySerializer.float32(obj.trunk_velocity_cmd, buffer, bufferOffset, 6);
    // Check that the constant length array field [trunk_acc_cmd] has the right length
    if (obj.trunk_acc_cmd.length !== 6) {
      throw new Error('Unable to serialize array field trunk_acc_cmd - length must be 6')
    }
    // Serialize message field [trunk_acc_cmd]
    bufferOffset = _arraySerializer.float32(obj.trunk_acc_cmd, buffer, bufferOffset, 6);
    // Check that the constant length array field [trunk_kp] has the right length
    if (obj.trunk_kp.length !== 6) {
      throw new Error('Unable to serialize array field trunk_kp - length must be 6')
    }
    // Serialize message field [trunk_kp]
    bufferOffset = _arraySerializer.float32(obj.trunk_kp, buffer, bufferOffset, 6);
    // Check that the constant length array field [trunk_kd] has the right length
    if (obj.trunk_kd.length !== 6) {
      throw new Error('Unable to serialize array field trunk_kd - length must be 6')
    }
    // Serialize message field [trunk_kd]
    bufferOffset = _arraySerializer.float32(obj.trunk_kd, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type trunk_cmd
    let len;
    let data = new trunk_cmd(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [trunk_position_cmd]
    data.trunk_position_cmd = _arrayDeserializer.float32(buffer, bufferOffset, 6)
    // Deserialize message field [trunk_velocity_cmd]
    data.trunk_velocity_cmd = _arrayDeserializer.float32(buffer, bufferOffset, 6)
    // Deserialize message field [trunk_acc_cmd]
    data.trunk_acc_cmd = _arrayDeserializer.float32(buffer, bufferOffset, 6)
    // Deserialize message field [trunk_kp]
    data.trunk_kp = _arrayDeserializer.float32(buffer, bufferOffset, 6)
    // Deserialize message field [trunk_kd]
    data.trunk_kd = _arrayDeserializer.float32(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 120;
  }

  static datatype() {
    // Returns string type for a message object
    return 'control_msgs/trunk_cmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9c7edd9abee115da7ddc97ab8e181c2e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #topic:'motion/trunk/cmd'
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new trunk_cmd(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.trunk_position_cmd !== undefined) {
      resolved.trunk_position_cmd = msg.trunk_position_cmd;
    }
    else {
      resolved.trunk_position_cmd = new Array(6).fill(0)
    }

    if (msg.trunk_velocity_cmd !== undefined) {
      resolved.trunk_velocity_cmd = msg.trunk_velocity_cmd;
    }
    else {
      resolved.trunk_velocity_cmd = new Array(6).fill(0)
    }

    if (msg.trunk_acc_cmd !== undefined) {
      resolved.trunk_acc_cmd = msg.trunk_acc_cmd;
    }
    else {
      resolved.trunk_acc_cmd = new Array(6).fill(0)
    }

    if (msg.trunk_kp !== undefined) {
      resolved.trunk_kp = msg.trunk_kp;
    }
    else {
      resolved.trunk_kp = new Array(6).fill(0)
    }

    if (msg.trunk_kd !== undefined) {
      resolved.trunk_kd = msg.trunk_kd;
    }
    else {
      resolved.trunk_kd = new Array(6).fill(0)
    }

    return resolved;
    }
};

module.exports = trunk_cmd;
