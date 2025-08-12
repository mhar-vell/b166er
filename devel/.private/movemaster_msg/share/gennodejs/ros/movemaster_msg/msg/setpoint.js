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

class setpoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.set_1 = null;
      this.set_2 = null;
      this.set_3 = null;
      this.set_4 = null;
      this.set_5 = null;
      this.set_GRIP = null;
      this.emergency_stop = null;
      this.GoHome = null;
    }
    else {
      if (initObj.hasOwnProperty('set_1')) {
        this.set_1 = initObj.set_1
      }
      else {
        this.set_1 = 0.0;
      }
      if (initObj.hasOwnProperty('set_2')) {
        this.set_2 = initObj.set_2
      }
      else {
        this.set_2 = 0.0;
      }
      if (initObj.hasOwnProperty('set_3')) {
        this.set_3 = initObj.set_3
      }
      else {
        this.set_3 = 0.0;
      }
      if (initObj.hasOwnProperty('set_4')) {
        this.set_4 = initObj.set_4
      }
      else {
        this.set_4 = 0.0;
      }
      if (initObj.hasOwnProperty('set_5')) {
        this.set_5 = initObj.set_5
      }
      else {
        this.set_5 = 0.0;
      }
      if (initObj.hasOwnProperty('set_GRIP')) {
        this.set_GRIP = initObj.set_GRIP
      }
      else {
        this.set_GRIP = false;
      }
      if (initObj.hasOwnProperty('emergency_stop')) {
        this.emergency_stop = initObj.emergency_stop
      }
      else {
        this.emergency_stop = false;
      }
      if (initObj.hasOwnProperty('GoHome')) {
        this.GoHome = initObj.GoHome
      }
      else {
        this.GoHome = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type setpoint
    // Serialize message field [set_1]
    bufferOffset = _serializer.float32(obj.set_1, buffer, bufferOffset);
    // Serialize message field [set_2]
    bufferOffset = _serializer.float32(obj.set_2, buffer, bufferOffset);
    // Serialize message field [set_3]
    bufferOffset = _serializer.float32(obj.set_3, buffer, bufferOffset);
    // Serialize message field [set_4]
    bufferOffset = _serializer.float32(obj.set_4, buffer, bufferOffset);
    // Serialize message field [set_5]
    bufferOffset = _serializer.float32(obj.set_5, buffer, bufferOffset);
    // Serialize message field [set_GRIP]
    bufferOffset = _serializer.bool(obj.set_GRIP, buffer, bufferOffset);
    // Serialize message field [emergency_stop]
    bufferOffset = _serializer.bool(obj.emergency_stop, buffer, bufferOffset);
    // Serialize message field [GoHome]
    bufferOffset = _serializer.int8(obj.GoHome, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type setpoint
    let len;
    let data = new setpoint(null);
    // Deserialize message field [set_1]
    data.set_1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [set_2]
    data.set_2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [set_3]
    data.set_3 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [set_4]
    data.set_4 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [set_5]
    data.set_5 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [set_GRIP]
    data.set_GRIP = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [emergency_stop]
    data.emergency_stop = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [GoHome]
    data.GoHome = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 23;
  }

  static datatype() {
    // Returns string type for a message object
    return 'movemaster_msg/setpoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aaa39b4215432c08ac9b36247ded9610';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 set_1
    float32 set_2
    float32 set_3
    float32 set_4
    float32 set_5
    bool 	set_GRIP
    bool 	emergency_stop
    int8 	GoHome
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new setpoint(null);
    if (msg.set_1 !== undefined) {
      resolved.set_1 = msg.set_1;
    }
    else {
      resolved.set_1 = 0.0
    }

    if (msg.set_2 !== undefined) {
      resolved.set_2 = msg.set_2;
    }
    else {
      resolved.set_2 = 0.0
    }

    if (msg.set_3 !== undefined) {
      resolved.set_3 = msg.set_3;
    }
    else {
      resolved.set_3 = 0.0
    }

    if (msg.set_4 !== undefined) {
      resolved.set_4 = msg.set_4;
    }
    else {
      resolved.set_4 = 0.0
    }

    if (msg.set_5 !== undefined) {
      resolved.set_5 = msg.set_5;
    }
    else {
      resolved.set_5 = 0.0
    }

    if (msg.set_GRIP !== undefined) {
      resolved.set_GRIP = msg.set_GRIP;
    }
    else {
      resolved.set_GRIP = false
    }

    if (msg.emergency_stop !== undefined) {
      resolved.emergency_stop = msg.emergency_stop;
    }
    else {
      resolved.emergency_stop = false
    }

    if (msg.GoHome !== undefined) {
      resolved.GoHome = msg.GoHome;
    }
    else {
      resolved.GoHome = 0
    }

    return resolved;
    }
};

module.exports = setpoint;
