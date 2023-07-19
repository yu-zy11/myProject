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

class trunk_state {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.trunk_position = null;
      this.trunk_velocity = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('trunk_position')) {
        this.trunk_position = initObj.trunk_position
      }
      else {
        this.trunk_position = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('trunk_velocity')) {
        this.trunk_velocity = initObj.trunk_velocity
      }
      else {
        this.trunk_velocity = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type trunk_state
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [trunk_position] has the right length
    if (obj.trunk_position.length !== 6) {
      throw new Error('Unable to serialize array field trunk_position - length must be 6')
    }
    // Serialize message field [trunk_position]
    bufferOffset = _arraySerializer.float32(obj.trunk_position, buffer, bufferOffset, 6);
    // Check that the constant length array field [trunk_velocity] has the right length
    if (obj.trunk_velocity.length !== 6) {
      throw new Error('Unable to serialize array field trunk_velocity - length must be 6')
    }
    // Serialize message field [trunk_velocity]
    bufferOffset = _arraySerializer.float32(obj.trunk_velocity, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type trunk_state
    let len;
    let data = new trunk_state(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [trunk_position]
    data.trunk_position = _arrayDeserializer.float32(buffer, bufferOffset, 6)
    // Deserialize message field [trunk_velocity]
    data.trunk_velocity = _arrayDeserializer.float32(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'control_msgs/trunk_state';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd6da26616d9cb3c34772795c92291a45';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #topic:'motion/trunk/state'
    Header header
    
    float32[6] trunk_position
    float32[6] trunk_velocity
    
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
    const resolved = new trunk_state(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.trunk_position !== undefined) {
      resolved.trunk_position = msg.trunk_position;
    }
    else {
      resolved.trunk_position = new Array(6).fill(0)
    }

    if (msg.trunk_velocity !== undefined) {
      resolved.trunk_velocity = msg.trunk_velocity;
    }
    else {
      resolved.trunk_velocity = new Array(6).fill(0)
    }

    return resolved;
    }
};

module.exports = trunk_state;
