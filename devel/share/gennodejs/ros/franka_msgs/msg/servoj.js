// Auto-generated. Do not edit!

// (in-package franka_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class servoj {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.keepalive = null;
      this.cmd_q = null;
    }
    else {
      if (initObj.hasOwnProperty('keepalive')) {
        this.keepalive = initObj.keepalive
      }
      else {
        this.keepalive = 0;
      }
      if (initObj.hasOwnProperty('cmd_q')) {
        this.cmd_q = initObj.cmd_q
      }
      else {
        this.cmd_q = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type servoj
    // Serialize message field [keepalive]
    bufferOffset = _serializer.int64(obj.keepalive, buffer, bufferOffset);
    // Serialize message field [cmd_q]
    bufferOffset = _arraySerializer.float64(obj.cmd_q, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type servoj
    let len;
    let data = new servoj(null);
    // Deserialize message field [keepalive]
    data.keepalive = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [cmd_q]
    data.cmd_q = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.cmd_q.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'franka_msgs/servoj';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '961076d13ed378a92e75de5ec5b3a69b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 keepalive
    float64[] cmd_q
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new servoj(null);
    if (msg.keepalive !== undefined) {
      resolved.keepalive = msg.keepalive;
    }
    else {
      resolved.keepalive = 0
    }

    if (msg.cmd_q !== undefined) {
      resolved.cmd_q = msg.cmd_q;
    }
    else {
      resolved.cmd_q = []
    }

    return resolved;
    }
};

module.exports = servoj;
