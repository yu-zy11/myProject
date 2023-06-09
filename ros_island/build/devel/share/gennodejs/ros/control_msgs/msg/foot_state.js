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

class foot_state {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.foot_position = null;
      this.foot_velocity = null;
      this.contact_state = null;
      this.foot_force = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('foot_position')) {
        this.foot_position = initObj.foot_position
      }
      else {
        this.foot_position = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('foot_velocity')) {
        this.foot_velocity = initObj.foot_velocity
      }
      else {
        this.foot_velocity = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('contact_state')) {
        this.contact_state = initObj.contact_state
      }
      else {
        this.contact_state = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('foot_force')) {
        this.foot_force = initObj.foot_force
      }
      else {
        this.foot_force = new Array(12).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type foot_state
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [foot_position] has the right length
    if (obj.foot_position.length !== 12) {
      throw new Error('Unable to serialize array field foot_position - length must be 12')
    }
    // Serialize message field [foot_position]
    bufferOffset = _arraySerializer.float32(obj.foot_position, buffer, bufferOffset, 12);
    // Check that the constant length array field [foot_velocity] has the right length
    if (obj.foot_velocity.length !== 12) {
      throw new Error('Unable to serialize array field foot_velocity - length must be 12')
    }
    // Serialize message field [foot_velocity]
    bufferOffset = _arraySerializer.float32(obj.foot_velocity, buffer, bufferOffset, 12);
    // Check that the constant length array field [contact_state] has the right length
    if (obj.contact_state.length !== 4) {
      throw new Error('Unable to serialize array field contact_state - length must be 4')
    }
    // Serialize message field [contact_state]
    bufferOffset = _arraySerializer.float32(obj.contact_state, buffer, bufferOffset, 4);
    // Check that the constant length array field [foot_force] has the right length
    if (obj.foot_force.length !== 12) {
      throw new Error('Unable to serialize array field foot_force - length must be 12')
    }
    // Serialize message field [foot_force]
    bufferOffset = _arraySerializer.float32(obj.foot_force, buffer, bufferOffset, 12);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type foot_state
    let len;
    let data = new foot_state(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [foot_position]
    data.foot_position = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [foot_velocity]
    data.foot_velocity = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [contact_state]
    data.contact_state = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [foot_force]
    data.foot_force = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 160;
  }

  static datatype() {
    // Returns string type for a message object
    return 'control_msgs/foot_state';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ed2e25b8bdf234bee1b653ec362f2558';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #topic:'motion/foot/state'
    Header header
    
    float32[12] foot_position
    float32[12] foot_velocity
    float32[4]  contact_state
    float32[12] foot_force
    
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
    const resolved = new foot_state(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.foot_position !== undefined) {
      resolved.foot_position = msg.foot_position;
    }
    else {
      resolved.foot_position = new Array(12).fill(0)
    }

    if (msg.foot_velocity !== undefined) {
      resolved.foot_velocity = msg.foot_velocity;
    }
    else {
      resolved.foot_velocity = new Array(12).fill(0)
    }

    if (msg.contact_state !== undefined) {
      resolved.contact_state = msg.contact_state;
    }
    else {
      resolved.contact_state = new Array(4).fill(0)
    }

    if (msg.foot_force !== undefined) {
      resolved.foot_force = msg.foot_force;
    }
    else {
      resolved.foot_force = new Array(12).fill(0)
    }

    return resolved;
    }
};

module.exports = foot_state;
