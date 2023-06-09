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

class foot_cmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.foot_position_cmd = null;
      this.foot_velocity_cmd = null;
      this.foot_acc_cmd = null;
      this.contact_target = null;
      this.foot_force_ff = null;
      this.foot_kp = null;
      this.foot_kd = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('foot_position_cmd')) {
        this.foot_position_cmd = initObj.foot_position_cmd
      }
      else {
        this.foot_position_cmd = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('foot_velocity_cmd')) {
        this.foot_velocity_cmd = initObj.foot_velocity_cmd
      }
      else {
        this.foot_velocity_cmd = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('foot_acc_cmd')) {
        this.foot_acc_cmd = initObj.foot_acc_cmd
      }
      else {
        this.foot_acc_cmd = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('contact_target')) {
        this.contact_target = initObj.contact_target
      }
      else {
        this.contact_target = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('foot_force_ff')) {
        this.foot_force_ff = initObj.foot_force_ff
      }
      else {
        this.foot_force_ff = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('foot_kp')) {
        this.foot_kp = initObj.foot_kp
      }
      else {
        this.foot_kp = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('foot_kd')) {
        this.foot_kd = initObj.foot_kd
      }
      else {
        this.foot_kd = new Array(12).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type foot_cmd
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [foot_position_cmd] has the right length
    if (obj.foot_position_cmd.length !== 12) {
      throw new Error('Unable to serialize array field foot_position_cmd - length must be 12')
    }
    // Serialize message field [foot_position_cmd]
    bufferOffset = _arraySerializer.float32(obj.foot_position_cmd, buffer, bufferOffset, 12);
    // Check that the constant length array field [foot_velocity_cmd] has the right length
    if (obj.foot_velocity_cmd.length !== 12) {
      throw new Error('Unable to serialize array field foot_velocity_cmd - length must be 12')
    }
    // Serialize message field [foot_velocity_cmd]
    bufferOffset = _arraySerializer.float32(obj.foot_velocity_cmd, buffer, bufferOffset, 12);
    // Check that the constant length array field [foot_acc_cmd] has the right length
    if (obj.foot_acc_cmd.length !== 12) {
      throw new Error('Unable to serialize array field foot_acc_cmd - length must be 12')
    }
    // Serialize message field [foot_acc_cmd]
    bufferOffset = _arraySerializer.float32(obj.foot_acc_cmd, buffer, bufferOffset, 12);
    // Check that the constant length array field [contact_target] has the right length
    if (obj.contact_target.length !== 4) {
      throw new Error('Unable to serialize array field contact_target - length must be 4')
    }
    // Serialize message field [contact_target]
    bufferOffset = _arraySerializer.float32(obj.contact_target, buffer, bufferOffset, 4);
    // Check that the constant length array field [foot_force_ff] has the right length
    if (obj.foot_force_ff.length !== 12) {
      throw new Error('Unable to serialize array field foot_force_ff - length must be 12')
    }
    // Serialize message field [foot_force_ff]
    bufferOffset = _arraySerializer.float32(obj.foot_force_ff, buffer, bufferOffset, 12);
    // Check that the constant length array field [foot_kp] has the right length
    if (obj.foot_kp.length !== 12) {
      throw new Error('Unable to serialize array field foot_kp - length must be 12')
    }
    // Serialize message field [foot_kp]
    bufferOffset = _arraySerializer.float32(obj.foot_kp, buffer, bufferOffset, 12);
    // Check that the constant length array field [foot_kd] has the right length
    if (obj.foot_kd.length !== 12) {
      throw new Error('Unable to serialize array field foot_kd - length must be 12')
    }
    // Serialize message field [foot_kd]
    bufferOffset = _arraySerializer.float32(obj.foot_kd, buffer, bufferOffset, 12);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type foot_cmd
    let len;
    let data = new foot_cmd(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [foot_position_cmd]
    data.foot_position_cmd = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [foot_velocity_cmd]
    data.foot_velocity_cmd = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [foot_acc_cmd]
    data.foot_acc_cmd = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [contact_target]
    data.contact_target = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [foot_force_ff]
    data.foot_force_ff = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [foot_kp]
    data.foot_kp = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [foot_kd]
    data.foot_kd = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 304;
  }

  static datatype() {
    // Returns string type for a message object
    return 'control_msgs/foot_cmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'adaaf980547d2f8d216cbda68b41af75';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #topic:'motion/foot/cmd'
    Header header
    
    float32[12] foot_position_cmd
    float32[12] foot_velocity_cmd
    float32[12] foot_acc_cmd
    float32[4]  contact_target
    float32[12] foot_force_ff
    float32[12] foot_kp
    float32[12] foot_kd
    
    
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
    const resolved = new foot_cmd(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.foot_position_cmd !== undefined) {
      resolved.foot_position_cmd = msg.foot_position_cmd;
    }
    else {
      resolved.foot_position_cmd = new Array(12).fill(0)
    }

    if (msg.foot_velocity_cmd !== undefined) {
      resolved.foot_velocity_cmd = msg.foot_velocity_cmd;
    }
    else {
      resolved.foot_velocity_cmd = new Array(12).fill(0)
    }

    if (msg.foot_acc_cmd !== undefined) {
      resolved.foot_acc_cmd = msg.foot_acc_cmd;
    }
    else {
      resolved.foot_acc_cmd = new Array(12).fill(0)
    }

    if (msg.contact_target !== undefined) {
      resolved.contact_target = msg.contact_target;
    }
    else {
      resolved.contact_target = new Array(4).fill(0)
    }

    if (msg.foot_force_ff !== undefined) {
      resolved.foot_force_ff = msg.foot_force_ff;
    }
    else {
      resolved.foot_force_ff = new Array(12).fill(0)
    }

    if (msg.foot_kp !== undefined) {
      resolved.foot_kp = msg.foot_kp;
    }
    else {
      resolved.foot_kp = new Array(12).fill(0)
    }

    if (msg.foot_kd !== undefined) {
      resolved.foot_kd = msg.foot_kd;
    }
    else {
      resolved.foot_kd = new Array(12).fill(0)
    }

    return resolved;
    }
};

module.exports = foot_cmd;
