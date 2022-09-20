// Auto-generated. Do not edit!

// (in-package ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RobotStates {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.body_position = null;
      this.body_linear_velocity_in_world = null;
      this.body_linear_velocity_in_body = null;
      this.body_acceleration_in_body = null;
      this.body_euler = null;
      this.body_quaternion = null;
      this.body_angular_velocity_in_world = null;
      this.body_angular_velocity_in_body = null;
      this.rotation_matrix_body_to_world = null;
      this.joint_position = null;
      this.joint_velocity = null;
      this.torque_reading = null;
      this.joint_acceleration = null;
      this.foot_contact_force = null;
      this.foot_contact_detected = null;
      this.foot_contact_probability = null;
      this.foot_position_in_body = null;
      this.foot_velocity_in_body = null;
      this.foot_jacobian_in_body = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('body_position')) {
        this.body_position = initObj.body_position
      }
      else {
        this.body_position = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('body_linear_velocity_in_world')) {
        this.body_linear_velocity_in_world = initObj.body_linear_velocity_in_world
      }
      else {
        this.body_linear_velocity_in_world = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('body_linear_velocity_in_body')) {
        this.body_linear_velocity_in_body = initObj.body_linear_velocity_in_body
      }
      else {
        this.body_linear_velocity_in_body = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('body_acceleration_in_body')) {
        this.body_acceleration_in_body = initObj.body_acceleration_in_body
      }
      else {
        this.body_acceleration_in_body = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('body_euler')) {
        this.body_euler = initObj.body_euler
      }
      else {
        this.body_euler = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('body_quaternion')) {
        this.body_quaternion = initObj.body_quaternion
      }
      else {
        this.body_quaternion = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('body_angular_velocity_in_world')) {
        this.body_angular_velocity_in_world = initObj.body_angular_velocity_in_world
      }
      else {
        this.body_angular_velocity_in_world = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('body_angular_velocity_in_body')) {
        this.body_angular_velocity_in_body = initObj.body_angular_velocity_in_body
      }
      else {
        this.body_angular_velocity_in_body = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('rotation_matrix_body_to_world')) {
        this.rotation_matrix_body_to_world = initObj.rotation_matrix_body_to_world
      }
      else {
        this.rotation_matrix_body_to_world = new Array(9).fill(0);
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
      if (initObj.hasOwnProperty('torque_reading')) {
        this.torque_reading = initObj.torque_reading
      }
      else {
        this.torque_reading = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('joint_acceleration')) {
        this.joint_acceleration = initObj.joint_acceleration
      }
      else {
        this.joint_acceleration = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('foot_contact_force')) {
        this.foot_contact_force = initObj.foot_contact_force
      }
      else {
        this.foot_contact_force = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('foot_contact_detected')) {
        this.foot_contact_detected = initObj.foot_contact_detected
      }
      else {
        this.foot_contact_detected = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('foot_contact_probability')) {
        this.foot_contact_probability = initObj.foot_contact_probability
      }
      else {
        this.foot_contact_probability = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('foot_position_in_body')) {
        this.foot_position_in_body = initObj.foot_position_in_body
      }
      else {
        this.foot_position_in_body = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('foot_velocity_in_body')) {
        this.foot_velocity_in_body = initObj.foot_velocity_in_body
      }
      else {
        this.foot_velocity_in_body = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('foot_jacobian_in_body')) {
        this.foot_jacobian_in_body = initObj.foot_jacobian_in_body
      }
      else {
        this.foot_jacobian_in_body = new Array(144).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotStates
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [body_position] has the right length
    if (obj.body_position.length !== 3) {
      throw new Error('Unable to serialize array field body_position - length must be 3')
    }
    // Serialize message field [body_position]
    bufferOffset = _arraySerializer.float32(obj.body_position, buffer, bufferOffset, 3);
    // Check that the constant length array field [body_linear_velocity_in_world] has the right length
    if (obj.body_linear_velocity_in_world.length !== 3) {
      throw new Error('Unable to serialize array field body_linear_velocity_in_world - length must be 3')
    }
    // Serialize message field [body_linear_velocity_in_world]
    bufferOffset = _arraySerializer.float32(obj.body_linear_velocity_in_world, buffer, bufferOffset, 3);
    // Check that the constant length array field [body_linear_velocity_in_body] has the right length
    if (obj.body_linear_velocity_in_body.length !== 3) {
      throw new Error('Unable to serialize array field body_linear_velocity_in_body - length must be 3')
    }
    // Serialize message field [body_linear_velocity_in_body]
    bufferOffset = _arraySerializer.float32(obj.body_linear_velocity_in_body, buffer, bufferOffset, 3);
    // Check that the constant length array field [body_acceleration_in_body] has the right length
    if (obj.body_acceleration_in_body.length !== 3) {
      throw new Error('Unable to serialize array field body_acceleration_in_body - length must be 3')
    }
    // Serialize message field [body_acceleration_in_body]
    bufferOffset = _arraySerializer.float32(obj.body_acceleration_in_body, buffer, bufferOffset, 3);
    // Check that the constant length array field [body_euler] has the right length
    if (obj.body_euler.length !== 3) {
      throw new Error('Unable to serialize array field body_euler - length must be 3')
    }
    // Serialize message field [body_euler]
    bufferOffset = _arraySerializer.float32(obj.body_euler, buffer, bufferOffset, 3);
    // Check that the constant length array field [body_quaternion] has the right length
    if (obj.body_quaternion.length !== 4) {
      throw new Error('Unable to serialize array field body_quaternion - length must be 4')
    }
    // Serialize message field [body_quaternion]
    bufferOffset = _arraySerializer.float32(obj.body_quaternion, buffer, bufferOffset, 4);
    // Check that the constant length array field [body_angular_velocity_in_world] has the right length
    if (obj.body_angular_velocity_in_world.length !== 3) {
      throw new Error('Unable to serialize array field body_angular_velocity_in_world - length must be 3')
    }
    // Serialize message field [body_angular_velocity_in_world]
    bufferOffset = _arraySerializer.float32(obj.body_angular_velocity_in_world, buffer, bufferOffset, 3);
    // Check that the constant length array field [body_angular_velocity_in_body] has the right length
    if (obj.body_angular_velocity_in_body.length !== 3) {
      throw new Error('Unable to serialize array field body_angular_velocity_in_body - length must be 3')
    }
    // Serialize message field [body_angular_velocity_in_body]
    bufferOffset = _arraySerializer.float32(obj.body_angular_velocity_in_body, buffer, bufferOffset, 3);
    // Check that the constant length array field [rotation_matrix_body_to_world] has the right length
    if (obj.rotation_matrix_body_to_world.length !== 9) {
      throw new Error('Unable to serialize array field rotation_matrix_body_to_world - length must be 9')
    }
    // Serialize message field [rotation_matrix_body_to_world]
    bufferOffset = _arraySerializer.float32(obj.rotation_matrix_body_to_world, buffer, bufferOffset, 9);
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
    // Check that the constant length array field [torque_reading] has the right length
    if (obj.torque_reading.length !== 12) {
      throw new Error('Unable to serialize array field torque_reading - length must be 12')
    }
    // Serialize message field [torque_reading]
    bufferOffset = _arraySerializer.float32(obj.torque_reading, buffer, bufferOffset, 12);
    // Check that the constant length array field [joint_acceleration] has the right length
    if (obj.joint_acceleration.length !== 12) {
      throw new Error('Unable to serialize array field joint_acceleration - length must be 12')
    }
    // Serialize message field [joint_acceleration]
    bufferOffset = _arraySerializer.float32(obj.joint_acceleration, buffer, bufferOffset, 12);
    // Check that the constant length array field [foot_contact_force] has the right length
    if (obj.foot_contact_force.length !== 4) {
      throw new Error('Unable to serialize array field foot_contact_force - length must be 4')
    }
    // Serialize message field [foot_contact_force]
    bufferOffset = _arraySerializer.float32(obj.foot_contact_force, buffer, bufferOffset, 4);
    // Check that the constant length array field [foot_contact_detected] has the right length
    if (obj.foot_contact_detected.length !== 4) {
      throw new Error('Unable to serialize array field foot_contact_detected - length must be 4')
    }
    // Serialize message field [foot_contact_detected]
    bufferOffset = _arraySerializer.int32(obj.foot_contact_detected, buffer, bufferOffset, 4);
    // Check that the constant length array field [foot_contact_probability] has the right length
    if (obj.foot_contact_probability.length !== 4) {
      throw new Error('Unable to serialize array field foot_contact_probability - length must be 4')
    }
    // Serialize message field [foot_contact_probability]
    bufferOffset = _arraySerializer.float32(obj.foot_contact_probability, buffer, bufferOffset, 4);
    // Check that the constant length array field [foot_position_in_body] has the right length
    if (obj.foot_position_in_body.length !== 12) {
      throw new Error('Unable to serialize array field foot_position_in_body - length must be 12')
    }
    // Serialize message field [foot_position_in_body]
    bufferOffset = _arraySerializer.float32(obj.foot_position_in_body, buffer, bufferOffset, 12);
    // Check that the constant length array field [foot_velocity_in_body] has the right length
    if (obj.foot_velocity_in_body.length !== 12) {
      throw new Error('Unable to serialize array field foot_velocity_in_body - length must be 12')
    }
    // Serialize message field [foot_velocity_in_body]
    bufferOffset = _arraySerializer.float32(obj.foot_velocity_in_body, buffer, bufferOffset, 12);
    // Check that the constant length array field [foot_jacobian_in_body] has the right length
    if (obj.foot_jacobian_in_body.length !== 144) {
      throw new Error('Unable to serialize array field foot_jacobian_in_body - length must be 144')
    }
    // Serialize message field [foot_jacobian_in_body]
    bufferOffset = _arraySerializer.float32(obj.foot_jacobian_in_body, buffer, bufferOffset, 144);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotStates
    let len;
    let data = new RobotStates(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [body_position]
    data.body_position = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [body_linear_velocity_in_world]
    data.body_linear_velocity_in_world = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [body_linear_velocity_in_body]
    data.body_linear_velocity_in_body = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [body_acceleration_in_body]
    data.body_acceleration_in_body = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [body_euler]
    data.body_euler = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [body_quaternion]
    data.body_quaternion = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [body_angular_velocity_in_world]
    data.body_angular_velocity_in_world = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [body_angular_velocity_in_body]
    data.body_angular_velocity_in_body = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [rotation_matrix_body_to_world]
    data.rotation_matrix_body_to_world = _arrayDeserializer.float32(buffer, bufferOffset, 9)
    // Deserialize message field [joint_position]
    data.joint_position = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [joint_velocity]
    data.joint_velocity = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [torque_reading]
    data.torque_reading = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [joint_acceleration]
    data.joint_acceleration = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [foot_contact_force]
    data.foot_contact_force = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [foot_contact_detected]
    data.foot_contact_detected = _arrayDeserializer.int32(buffer, bufferOffset, 4)
    // Deserialize message field [foot_contact_probability]
    data.foot_contact_probability = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [foot_position_in_body]
    data.foot_position_in_body = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [foot_velocity_in_body]
    data.foot_velocity_in_body = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [foot_jacobian_in_body]
    data.foot_jacobian_in_body = _arrayDeserializer.float32(buffer, bufferOffset, 144)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 1048;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ros_msgs/RobotStates';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a8b0d3c2e9f9adf1e85eb4959a9e0667';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float32[3] body_position
    float32[3] body_linear_velocity_in_world
    float32[3] body_linear_velocity_in_body
    float32[3] body_acceleration_in_body
    float32[3] body_euler
    float32[4] body_quaternion
    float32[3] body_angular_velocity_in_world
    float32[3] body_angular_velocity_in_body
    float32[9] rotation_matrix_body_to_world
    float32[12] joint_position
    float32[12] joint_velocity
    float32[12] torque_reading
    float32[12] joint_acceleration
    float32[4] foot_contact_force
    int32[4] foot_contact_detected
    float32[4] foot_contact_probability
    float32[12] foot_position_in_body
    float32[12] foot_velocity_in_body
    float32[144] foot_jacobian_in_body
    
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
    const resolved = new RobotStates(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.body_position !== undefined) {
      resolved.body_position = msg.body_position;
    }
    else {
      resolved.body_position = new Array(3).fill(0)
    }

    if (msg.body_linear_velocity_in_world !== undefined) {
      resolved.body_linear_velocity_in_world = msg.body_linear_velocity_in_world;
    }
    else {
      resolved.body_linear_velocity_in_world = new Array(3).fill(0)
    }

    if (msg.body_linear_velocity_in_body !== undefined) {
      resolved.body_linear_velocity_in_body = msg.body_linear_velocity_in_body;
    }
    else {
      resolved.body_linear_velocity_in_body = new Array(3).fill(0)
    }

    if (msg.body_acceleration_in_body !== undefined) {
      resolved.body_acceleration_in_body = msg.body_acceleration_in_body;
    }
    else {
      resolved.body_acceleration_in_body = new Array(3).fill(0)
    }

    if (msg.body_euler !== undefined) {
      resolved.body_euler = msg.body_euler;
    }
    else {
      resolved.body_euler = new Array(3).fill(0)
    }

    if (msg.body_quaternion !== undefined) {
      resolved.body_quaternion = msg.body_quaternion;
    }
    else {
      resolved.body_quaternion = new Array(4).fill(0)
    }

    if (msg.body_angular_velocity_in_world !== undefined) {
      resolved.body_angular_velocity_in_world = msg.body_angular_velocity_in_world;
    }
    else {
      resolved.body_angular_velocity_in_world = new Array(3).fill(0)
    }

    if (msg.body_angular_velocity_in_body !== undefined) {
      resolved.body_angular_velocity_in_body = msg.body_angular_velocity_in_body;
    }
    else {
      resolved.body_angular_velocity_in_body = new Array(3).fill(0)
    }

    if (msg.rotation_matrix_body_to_world !== undefined) {
      resolved.rotation_matrix_body_to_world = msg.rotation_matrix_body_to_world;
    }
    else {
      resolved.rotation_matrix_body_to_world = new Array(9).fill(0)
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

    if (msg.torque_reading !== undefined) {
      resolved.torque_reading = msg.torque_reading;
    }
    else {
      resolved.torque_reading = new Array(12).fill(0)
    }

    if (msg.joint_acceleration !== undefined) {
      resolved.joint_acceleration = msg.joint_acceleration;
    }
    else {
      resolved.joint_acceleration = new Array(12).fill(0)
    }

    if (msg.foot_contact_force !== undefined) {
      resolved.foot_contact_force = msg.foot_contact_force;
    }
    else {
      resolved.foot_contact_force = new Array(4).fill(0)
    }

    if (msg.foot_contact_detected !== undefined) {
      resolved.foot_contact_detected = msg.foot_contact_detected;
    }
    else {
      resolved.foot_contact_detected = new Array(4).fill(0)
    }

    if (msg.foot_contact_probability !== undefined) {
      resolved.foot_contact_probability = msg.foot_contact_probability;
    }
    else {
      resolved.foot_contact_probability = new Array(4).fill(0)
    }

    if (msg.foot_position_in_body !== undefined) {
      resolved.foot_position_in_body = msg.foot_position_in_body;
    }
    else {
      resolved.foot_position_in_body = new Array(12).fill(0)
    }

    if (msg.foot_velocity_in_body !== undefined) {
      resolved.foot_velocity_in_body = msg.foot_velocity_in_body;
    }
    else {
      resolved.foot_velocity_in_body = new Array(12).fill(0)
    }

    if (msg.foot_jacobian_in_body !== undefined) {
      resolved.foot_jacobian_in_body = msg.foot_jacobian_in_body;
    }
    else {
      resolved.foot_jacobian_in_body = new Array(144).fill(0)
    }

    return resolved;
    }
};

module.exports = RobotStates;
