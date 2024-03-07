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

class telemetry {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timestamp = null;
      this.imu_gyro_x = null;
      this.imu_gyro_y = null;
      this.imu_gyro_z = null;
      this.imu_gyro_compass_x = null;
      this.imu_gyro_compass_y = null;
      this.imu_gyro_compass_z = null;
      this.q1 = null;
      this.q2 = null;
      this.q3 = null;
      this.q4 = null;
      this.rollspeed = null;
      this.pitchspeed = null;
      this.yawspeed = null;
      this.internal_pressure = null;
      this.external_pressure = null;
      this.heading = null;
    }
    else {
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0;
      }
      if (initObj.hasOwnProperty('imu_gyro_x')) {
        this.imu_gyro_x = initObj.imu_gyro_x
      }
      else {
        this.imu_gyro_x = 0;
      }
      if (initObj.hasOwnProperty('imu_gyro_y')) {
        this.imu_gyro_y = initObj.imu_gyro_y
      }
      else {
        this.imu_gyro_y = 0;
      }
      if (initObj.hasOwnProperty('imu_gyro_z')) {
        this.imu_gyro_z = initObj.imu_gyro_z
      }
      else {
        this.imu_gyro_z = 0;
      }
      if (initObj.hasOwnProperty('imu_gyro_compass_x')) {
        this.imu_gyro_compass_x = initObj.imu_gyro_compass_x
      }
      else {
        this.imu_gyro_compass_x = 0;
      }
      if (initObj.hasOwnProperty('imu_gyro_compass_y')) {
        this.imu_gyro_compass_y = initObj.imu_gyro_compass_y
      }
      else {
        this.imu_gyro_compass_y = 0;
      }
      if (initObj.hasOwnProperty('imu_gyro_compass_z')) {
        this.imu_gyro_compass_z = initObj.imu_gyro_compass_z
      }
      else {
        this.imu_gyro_compass_z = 0;
      }
      if (initObj.hasOwnProperty('q1')) {
        this.q1 = initObj.q1
      }
      else {
        this.q1 = 0.0;
      }
      if (initObj.hasOwnProperty('q2')) {
        this.q2 = initObj.q2
      }
      else {
        this.q2 = 0.0;
      }
      if (initObj.hasOwnProperty('q3')) {
        this.q3 = initObj.q3
      }
      else {
        this.q3 = 0.0;
      }
      if (initObj.hasOwnProperty('q4')) {
        this.q4 = initObj.q4
      }
      else {
        this.q4 = 0.0;
      }
      if (initObj.hasOwnProperty('rollspeed')) {
        this.rollspeed = initObj.rollspeed
      }
      else {
        this.rollspeed = 0.0;
      }
      if (initObj.hasOwnProperty('pitchspeed')) {
        this.pitchspeed = initObj.pitchspeed
      }
      else {
        this.pitchspeed = 0.0;
      }
      if (initObj.hasOwnProperty('yawspeed')) {
        this.yawspeed = initObj.yawspeed
      }
      else {
        this.yawspeed = 0.0;
      }
      if (initObj.hasOwnProperty('internal_pressure')) {
        this.internal_pressure = initObj.internal_pressure
      }
      else {
        this.internal_pressure = 0.0;
      }
      if (initObj.hasOwnProperty('external_pressure')) {
        this.external_pressure = initObj.external_pressure
      }
      else {
        this.external_pressure = 0.0;
      }
      if (initObj.hasOwnProperty('heading')) {
        this.heading = initObj.heading
      }
      else {
        this.heading = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type telemetry
    // Serialize message field [timestamp]
    bufferOffset = _serializer.int32(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [imu_gyro_x]
    bufferOffset = _serializer.int32(obj.imu_gyro_x, buffer, bufferOffset);
    // Serialize message field [imu_gyro_y]
    bufferOffset = _serializer.int32(obj.imu_gyro_y, buffer, bufferOffset);
    // Serialize message field [imu_gyro_z]
    bufferOffset = _serializer.int32(obj.imu_gyro_z, buffer, bufferOffset);
    // Serialize message field [imu_gyro_compass_x]
    bufferOffset = _serializer.int32(obj.imu_gyro_compass_x, buffer, bufferOffset);
    // Serialize message field [imu_gyro_compass_y]
    bufferOffset = _serializer.int32(obj.imu_gyro_compass_y, buffer, bufferOffset);
    // Serialize message field [imu_gyro_compass_z]
    bufferOffset = _serializer.int32(obj.imu_gyro_compass_z, buffer, bufferOffset);
    // Serialize message field [q1]
    bufferOffset = _serializer.float32(obj.q1, buffer, bufferOffset);
    // Serialize message field [q2]
    bufferOffset = _serializer.float32(obj.q2, buffer, bufferOffset);
    // Serialize message field [q3]
    bufferOffset = _serializer.float32(obj.q3, buffer, bufferOffset);
    // Serialize message field [q4]
    bufferOffset = _serializer.float32(obj.q4, buffer, bufferOffset);
    // Serialize message field [rollspeed]
    bufferOffset = _serializer.float32(obj.rollspeed, buffer, bufferOffset);
    // Serialize message field [pitchspeed]
    bufferOffset = _serializer.float32(obj.pitchspeed, buffer, bufferOffset);
    // Serialize message field [yawspeed]
    bufferOffset = _serializer.float32(obj.yawspeed, buffer, bufferOffset);
    // Serialize message field [internal_pressure]
    bufferOffset = _serializer.float32(obj.internal_pressure, buffer, bufferOffset);
    // Serialize message field [external_pressure]
    bufferOffset = _serializer.float32(obj.external_pressure, buffer, bufferOffset);
    // Serialize message field [heading]
    bufferOffset = _serializer.int32(obj.heading, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type telemetry
    let len;
    let data = new telemetry(null);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [imu_gyro_x]
    data.imu_gyro_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [imu_gyro_y]
    data.imu_gyro_y = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [imu_gyro_z]
    data.imu_gyro_z = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [imu_gyro_compass_x]
    data.imu_gyro_compass_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [imu_gyro_compass_y]
    data.imu_gyro_compass_y = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [imu_gyro_compass_z]
    data.imu_gyro_compass_z = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [q1]
    data.q1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [q2]
    data.q2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [q3]
    data.q3 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [q4]
    data.q4 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rollspeed]
    data.rollspeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitchspeed]
    data.pitchspeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yawspeed]
    data.yawspeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [internal_pressure]
    data.internal_pressure = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [external_pressure]
    data.external_pressure = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading]
    data.heading = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 68;
  }

  static datatype() {
    // Returns string type for a message object
    return 'custom_msgs/telemetry';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '287f88c9716cc47a884bba700bcc2292';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 timestamp
    
    int32 imu_gyro_x
    int32 imu_gyro_y
    int32 imu_gyro_z
    
    
    
    int32 imu_gyro_compass_x
    int32 imu_gyro_compass_y
    int32 imu_gyro_compass_z
    
    
    float32 q1
    float32 q2
    float32 q3
    float32 q4
    float32 rollspeed
    float32 pitchspeed
    float32 yawspeed
    
    
    
    float32 internal_pressure
    float32 external_pressure
    int32 heading
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new telemetry(null);
    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0
    }

    if (msg.imu_gyro_x !== undefined) {
      resolved.imu_gyro_x = msg.imu_gyro_x;
    }
    else {
      resolved.imu_gyro_x = 0
    }

    if (msg.imu_gyro_y !== undefined) {
      resolved.imu_gyro_y = msg.imu_gyro_y;
    }
    else {
      resolved.imu_gyro_y = 0
    }

    if (msg.imu_gyro_z !== undefined) {
      resolved.imu_gyro_z = msg.imu_gyro_z;
    }
    else {
      resolved.imu_gyro_z = 0
    }

    if (msg.imu_gyro_compass_x !== undefined) {
      resolved.imu_gyro_compass_x = msg.imu_gyro_compass_x;
    }
    else {
      resolved.imu_gyro_compass_x = 0
    }

    if (msg.imu_gyro_compass_y !== undefined) {
      resolved.imu_gyro_compass_y = msg.imu_gyro_compass_y;
    }
    else {
      resolved.imu_gyro_compass_y = 0
    }

    if (msg.imu_gyro_compass_z !== undefined) {
      resolved.imu_gyro_compass_z = msg.imu_gyro_compass_z;
    }
    else {
      resolved.imu_gyro_compass_z = 0
    }

    if (msg.q1 !== undefined) {
      resolved.q1 = msg.q1;
    }
    else {
      resolved.q1 = 0.0
    }

    if (msg.q2 !== undefined) {
      resolved.q2 = msg.q2;
    }
    else {
      resolved.q2 = 0.0
    }

    if (msg.q3 !== undefined) {
      resolved.q3 = msg.q3;
    }
    else {
      resolved.q3 = 0.0
    }

    if (msg.q4 !== undefined) {
      resolved.q4 = msg.q4;
    }
    else {
      resolved.q4 = 0.0
    }

    if (msg.rollspeed !== undefined) {
      resolved.rollspeed = msg.rollspeed;
    }
    else {
      resolved.rollspeed = 0.0
    }

    if (msg.pitchspeed !== undefined) {
      resolved.pitchspeed = msg.pitchspeed;
    }
    else {
      resolved.pitchspeed = 0.0
    }

    if (msg.yawspeed !== undefined) {
      resolved.yawspeed = msg.yawspeed;
    }
    else {
      resolved.yawspeed = 0.0
    }

    if (msg.internal_pressure !== undefined) {
      resolved.internal_pressure = msg.internal_pressure;
    }
    else {
      resolved.internal_pressure = 0.0
    }

    if (msg.external_pressure !== undefined) {
      resolved.external_pressure = msg.external_pressure;
    }
    else {
      resolved.external_pressure = 0.0
    }

    if (msg.heading !== undefined) {
      resolved.heading = msg.heading;
    }
    else {
      resolved.heading = 0
    }

    return resolved;
    }
};

module.exports = telemetry;
