// Auto-generated. Do not edit!

// (in-package custom_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class commands {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.arm = null;
      this.mode = null;
      this.forward = null;
      this.lateral = null;
      this.thrust = null;
      this.pitch = null;
      this.roll = null;
      this.yaw = null;
      this.servo1 = null;
      this.servo2 = null;
    }
    else {
      if (initObj.hasOwnProperty('arm')) {
        this.arm = initObj.arm
      }
      else {
        this.arm = false;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = '';
      }
      if (initObj.hasOwnProperty('forward')) {
        this.forward = initObj.forward
      }
      else {
        this.forward = 0;
      }
      if (initObj.hasOwnProperty('lateral')) {
        this.lateral = initObj.lateral
      }
      else {
        this.lateral = 0;
      }
      if (initObj.hasOwnProperty('thrust')) {
        this.thrust = initObj.thrust
      }
      else {
        this.thrust = 0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0;
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0;
      }
      if (initObj.hasOwnProperty('servo1')) {
        this.servo1 = initObj.servo1
      }
      else {
        this.servo1 = 0;
      }
      if (initObj.hasOwnProperty('servo2')) {
        this.servo2 = initObj.servo2
      }
      else {
        this.servo2 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type commands
    // Serialize message field [arm]
    bufferOffset = _serializer.bool(obj.arm, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.string(obj.mode, buffer, bufferOffset);
    // Serialize message field [forward]
    bufferOffset = _serializer.int16(obj.forward, buffer, bufferOffset);
    // Serialize message field [lateral]
    bufferOffset = _serializer.int16(obj.lateral, buffer, bufferOffset);
    // Serialize message field [thrust]
    bufferOffset = _serializer.int16(obj.thrust, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.int16(obj.pitch, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.int16(obj.roll, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.int16(obj.yaw, buffer, bufferOffset);
    // Serialize message field [servo1]
    bufferOffset = _serializer.int16(obj.servo1, buffer, bufferOffset);
    // Serialize message field [servo2]
    bufferOffset = _serializer.int16(obj.servo2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type commands
    let len;
    let data = new commands(null);
    // Deserialize message field [arm]
    data.arm = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [forward]
    data.forward = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [lateral]
    data.lateral = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [thrust]
    data.thrust = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [servo1]
    data.servo1 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [servo2]
    data.servo2 = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.mode);
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'custom_msgs/commands';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fd03263c66e74cb889ba23a832fbae12';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool arm
    string mode
    int16 forward
    int16 lateral
    int16 thrust
    int16 pitch
    int16 roll
    int16 yaw
    int16 servo1
    int16 servo2
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new commands(null);
    if (msg.arm !== undefined) {
      resolved.arm = msg.arm;
    }
    else {
      resolved.arm = false
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = ''
    }

    if (msg.forward !== undefined) {
      resolved.forward = msg.forward;
    }
    else {
      resolved.forward = 0
    }

    if (msg.lateral !== undefined) {
      resolved.lateral = msg.lateral;
    }
    else {
      resolved.lateral = 0
    }

    if (msg.thrust !== undefined) {
      resolved.thrust = msg.thrust;
    }
    else {
      resolved.thrust = 0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0
    }

    if (msg.servo1 !== undefined) {
      resolved.servo1 = msg.servo1;
    }
    else {
      resolved.servo1 = 0
    }

    if (msg.servo2 !== undefined) {
      resolved.servo2 = msg.servo2;
    }
    else {
      resolved.servo2 = 0
    }

    return resolved;
    }
};

module.exports = commands;
