// Auto-generated. Do not edit!

// (in-package marker_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class marker {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ids = null;
      this.xErrors = null;
      this.yaws = null;
    }
    else {
      if (initObj.hasOwnProperty('ids')) {
        this.ids = initObj.ids
      }
      else {
        this.ids = new std_msgs.msg.Int16MultiArray();
      }
      if (initObj.hasOwnProperty('xErrors')) {
        this.xErrors = initObj.xErrors
      }
      else {
        this.xErrors = new std_msgs.msg.Float32MultiArray();
      }
      if (initObj.hasOwnProperty('yaws')) {
        this.yaws = initObj.yaws
      }
      else {
        this.yaws = new std_msgs.msg.Float32MultiArray();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type marker
    // Serialize message field [ids]
    bufferOffset = std_msgs.msg.Int16MultiArray.serialize(obj.ids, buffer, bufferOffset);
    // Serialize message field [xErrors]
    bufferOffset = std_msgs.msg.Float32MultiArray.serialize(obj.xErrors, buffer, bufferOffset);
    // Serialize message field [yaws]
    bufferOffset = std_msgs.msg.Float32MultiArray.serialize(obj.yaws, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type marker
    let len;
    let data = new marker(null);
    // Deserialize message field [ids]
    data.ids = std_msgs.msg.Int16MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [xErrors]
    data.xErrors = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [yaws]
    data.yaws = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Int16MultiArray.getMessageSize(object.ids);
    length += std_msgs.msg.Float32MultiArray.getMessageSize(object.xErrors);
    length += std_msgs.msg.Float32MultiArray.getMessageSize(object.yaws);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'marker_msgs/marker';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '29b5307f7b0077b567c8b87adcaca5dd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Int16MultiArray ids
    std_msgs/Float32MultiArray xErrors
    std_msgs/Float32MultiArray yaws
    ================================================================================
    MSG: std_msgs/Int16MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    int16[]           data          # array of data
    
    
    ================================================================================
    MSG: std_msgs/MultiArrayLayout
    # The multiarray declares a generic multi-dimensional array of a
    # particular data type.  Dimensions are ordered from outer most
    # to inner most.
    
    MultiArrayDimension[] dim # Array of dimension properties
    uint32 data_offset        # padding elements at front of data
    
    # Accessors should ALWAYS be written in terms of dimension stride
    # and specified outer-most dimension first.
    # 
    # multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    #
    # A standard, 3-channel 640x480 image with interleaved color channels
    # would be specified as:
    #
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
    # dim[2].size   = 3
    # dim[2].stride = 3
    #
    # multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
    
    ================================================================================
    MSG: std_msgs/MultiArrayDimension
    string label   # label of given dimension
    uint32 size    # size of given dimension (in type units)
    uint32 stride  # stride of given dimension
    ================================================================================
    MSG: std_msgs/Float32MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    float32[]         data          # array of data
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new marker(null);
    if (msg.ids !== undefined) {
      resolved.ids = std_msgs.msg.Int16MultiArray.Resolve(msg.ids)
    }
    else {
      resolved.ids = new std_msgs.msg.Int16MultiArray()
    }

    if (msg.xErrors !== undefined) {
      resolved.xErrors = std_msgs.msg.Float32MultiArray.Resolve(msg.xErrors)
    }
    else {
      resolved.xErrors = new std_msgs.msg.Float32MultiArray()
    }

    if (msg.yaws !== undefined) {
      resolved.yaws = std_msgs.msg.Float32MultiArray.Resolve(msg.yaws)
    }
    else {
      resolved.yaws = new std_msgs.msg.Float32MultiArray()
    }

    return resolved;
    }
};

module.exports = marker;
