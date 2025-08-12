// Auto-generated. Do not edit!

// (in-package movemaster_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint = null;
      this.setpoint = null;
      this.pulse_count = null;
      this.error = null;
      this.output = null;
      this.control_loop = null;
      this.IsDone = null;
    }
    else {
      if (initObj.hasOwnProperty('joint')) {
        this.joint = initObj.joint
      }
      else {
        this.joint = '';
      }
      if (initObj.hasOwnProperty('setpoint')) {
        this.setpoint = initObj.setpoint
      }
      else {
        this.setpoint = 0.0;
      }
      if (initObj.hasOwnProperty('pulse_count')) {
        this.pulse_count = initObj.pulse_count
      }
      else {
        this.pulse_count = 0.0;
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = 0.0;
      }
      if (initObj.hasOwnProperty('output')) {
        this.output = initObj.output
      }
      else {
        this.output = 0.0;
      }
      if (initObj.hasOwnProperty('control_loop')) {
        this.control_loop = initObj.control_loop
      }
      else {
        this.control_loop = 0.0;
      }
      if (initObj.hasOwnProperty('IsDone')) {
        this.IsDone = initObj.IsDone
      }
      else {
        this.IsDone = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type status
    // Serialize message field [joint]
    bufferOffset = _serializer.string(obj.joint, buffer, bufferOffset);
    // Serialize message field [setpoint]
    bufferOffset = _serializer.float32(obj.setpoint, buffer, bufferOffset);
    // Serialize message field [pulse_count]
    bufferOffset = _serializer.float32(obj.pulse_count, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.float32(obj.error, buffer, bufferOffset);
    // Serialize message field [output]
    bufferOffset = _serializer.float32(obj.output, buffer, bufferOffset);
    // Serialize message field [control_loop]
    bufferOffset = _serializer.float32(obj.control_loop, buffer, bufferOffset);
    // Serialize message field [IsDone]
    bufferOffset = _serializer.bool(obj.IsDone, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type status
    let len;
    let data = new status(null);
    // Deserialize message field [joint]
    data.joint = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [setpoint]
    data.setpoint = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pulse_count]
    data.pulse_count = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [output]
    data.output = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [control_loop]
    data.control_loop = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [IsDone]
    data.IsDone = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.joint);
    return length + 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'movemaster_msg/status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '283025d1e58370fa7a43c23c572de06b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string joint
    float32 setpoint
    float32 pulse_count
    float32 error
    float32 output
    float32 control_loop
    bool IsDone
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new status(null);
    if (msg.joint !== undefined) {
      resolved.joint = msg.joint;
    }
    else {
      resolved.joint = ''
    }

    if (msg.setpoint !== undefined) {
      resolved.setpoint = msg.setpoint;
    }
    else {
      resolved.setpoint = 0.0
    }

    if (msg.pulse_count !== undefined) {
      resolved.pulse_count = msg.pulse_count;
    }
    else {
      resolved.pulse_count = 0.0
    }

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = 0.0
    }

    if (msg.output !== undefined) {
      resolved.output = msg.output;
    }
    else {
      resolved.output = 0.0
    }

    if (msg.control_loop !== undefined) {
      resolved.control_loop = msg.control_loop;
    }
    else {
      resolved.control_loop = 0.0
    }

    if (msg.IsDone !== undefined) {
      resolved.IsDone = msg.IsDone;
    }
    else {
      resolved.IsDone = false
    }

    return resolved;
    }
};

module.exports = status;
