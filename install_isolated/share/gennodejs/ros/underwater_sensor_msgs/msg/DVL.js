// Auto-generated. Do not edit!

// (in-package underwater_sensor_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class DVL {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.header_dvl = null;
      this.date = null;
      this.salinity = null;
      this.temperature = null;
      this.depth = null;
      this.sound_speed = null;
      this.test = null;
      this.wi_x_axis = null;
      this.wi_y_axis = null;
      this.wi_z_axis = null;
      this.wi_error = null;
      this.wi_status = null;
      this.bi_x_axis = null;
      this.bi_y_axis = null;
      this.bi_z_axis = null;
      this.bi_error = null;
      this.bi_status = null;
      this.ws_transverse = null;
      this.ws_longitudinal = null;
      this.ws_normal = null;
      this.ws_status = null;
      this.bs_transverse = null;
      this.bs_longitudinal = null;
      this.bs_normal = null;
      this.bs_status = null;
      this.we_east = null;
      this.we_north = null;
      this.we_upwards = null;
      this.we_status = null;
      this.be_east = null;
      this.be_north = null;
      this.be_upwards = null;
      this.be_status = null;
      this.wd_east = null;
      this.wd_north = null;
      this.wd_upwards = null;
      this.wd_range = null;
      this.wd_time = null;
      this.bd_east = null;
      this.bd_north = null;
      this.bd_upwards = null;
      this.bd_range = null;
      this.bd_time = null;
      this.raw_data = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('header_dvl')) {
        this.header_dvl = initObj.header_dvl
      }
      else {
        this.header_dvl = '';
      }
      if (initObj.hasOwnProperty('date')) {
        this.date = initObj.date
      }
      else {
        this.date = '';
      }
      if (initObj.hasOwnProperty('salinity')) {
        this.salinity = initObj.salinity
      }
      else {
        this.salinity = 0.0;
      }
      if (initObj.hasOwnProperty('temperature')) {
        this.temperature = initObj.temperature
      }
      else {
        this.temperature = 0.0;
      }
      if (initObj.hasOwnProperty('depth')) {
        this.depth = initObj.depth
      }
      else {
        this.depth = 0.0;
      }
      if (initObj.hasOwnProperty('sound_speed')) {
        this.sound_speed = initObj.sound_speed
      }
      else {
        this.sound_speed = 0.0;
      }
      if (initObj.hasOwnProperty('test')) {
        this.test = initObj.test
      }
      else {
        this.test = 0;
      }
      if (initObj.hasOwnProperty('wi_x_axis')) {
        this.wi_x_axis = initObj.wi_x_axis
      }
      else {
        this.wi_x_axis = 0.0;
      }
      if (initObj.hasOwnProperty('wi_y_axis')) {
        this.wi_y_axis = initObj.wi_y_axis
      }
      else {
        this.wi_y_axis = 0.0;
      }
      if (initObj.hasOwnProperty('wi_z_axis')) {
        this.wi_z_axis = initObj.wi_z_axis
      }
      else {
        this.wi_z_axis = 0.0;
      }
      if (initObj.hasOwnProperty('wi_error')) {
        this.wi_error = initObj.wi_error
      }
      else {
        this.wi_error = 0.0;
      }
      if (initObj.hasOwnProperty('wi_status')) {
        this.wi_status = initObj.wi_status
      }
      else {
        this.wi_status = '';
      }
      if (initObj.hasOwnProperty('bi_x_axis')) {
        this.bi_x_axis = initObj.bi_x_axis
      }
      else {
        this.bi_x_axis = 0.0;
      }
      if (initObj.hasOwnProperty('bi_y_axis')) {
        this.bi_y_axis = initObj.bi_y_axis
      }
      else {
        this.bi_y_axis = 0.0;
      }
      if (initObj.hasOwnProperty('bi_z_axis')) {
        this.bi_z_axis = initObj.bi_z_axis
      }
      else {
        this.bi_z_axis = 0.0;
      }
      if (initObj.hasOwnProperty('bi_error')) {
        this.bi_error = initObj.bi_error
      }
      else {
        this.bi_error = 0.0;
      }
      if (initObj.hasOwnProperty('bi_status')) {
        this.bi_status = initObj.bi_status
      }
      else {
        this.bi_status = '';
      }
      if (initObj.hasOwnProperty('ws_transverse')) {
        this.ws_transverse = initObj.ws_transverse
      }
      else {
        this.ws_transverse = 0.0;
      }
      if (initObj.hasOwnProperty('ws_longitudinal')) {
        this.ws_longitudinal = initObj.ws_longitudinal
      }
      else {
        this.ws_longitudinal = 0.0;
      }
      if (initObj.hasOwnProperty('ws_normal')) {
        this.ws_normal = initObj.ws_normal
      }
      else {
        this.ws_normal = 0.0;
      }
      if (initObj.hasOwnProperty('ws_status')) {
        this.ws_status = initObj.ws_status
      }
      else {
        this.ws_status = '';
      }
      if (initObj.hasOwnProperty('bs_transverse')) {
        this.bs_transverse = initObj.bs_transverse
      }
      else {
        this.bs_transverse = 0.0;
      }
      if (initObj.hasOwnProperty('bs_longitudinal')) {
        this.bs_longitudinal = initObj.bs_longitudinal
      }
      else {
        this.bs_longitudinal = 0.0;
      }
      if (initObj.hasOwnProperty('bs_normal')) {
        this.bs_normal = initObj.bs_normal
      }
      else {
        this.bs_normal = 0.0;
      }
      if (initObj.hasOwnProperty('bs_status')) {
        this.bs_status = initObj.bs_status
      }
      else {
        this.bs_status = '';
      }
      if (initObj.hasOwnProperty('we_east')) {
        this.we_east = initObj.we_east
      }
      else {
        this.we_east = 0.0;
      }
      if (initObj.hasOwnProperty('we_north')) {
        this.we_north = initObj.we_north
      }
      else {
        this.we_north = 0.0;
      }
      if (initObj.hasOwnProperty('we_upwards')) {
        this.we_upwards = initObj.we_upwards
      }
      else {
        this.we_upwards = 0.0;
      }
      if (initObj.hasOwnProperty('we_status')) {
        this.we_status = initObj.we_status
      }
      else {
        this.we_status = '';
      }
      if (initObj.hasOwnProperty('be_east')) {
        this.be_east = initObj.be_east
      }
      else {
        this.be_east = 0.0;
      }
      if (initObj.hasOwnProperty('be_north')) {
        this.be_north = initObj.be_north
      }
      else {
        this.be_north = 0.0;
      }
      if (initObj.hasOwnProperty('be_upwards')) {
        this.be_upwards = initObj.be_upwards
      }
      else {
        this.be_upwards = 0.0;
      }
      if (initObj.hasOwnProperty('be_status')) {
        this.be_status = initObj.be_status
      }
      else {
        this.be_status = '';
      }
      if (initObj.hasOwnProperty('wd_east')) {
        this.wd_east = initObj.wd_east
      }
      else {
        this.wd_east = 0.0;
      }
      if (initObj.hasOwnProperty('wd_north')) {
        this.wd_north = initObj.wd_north
      }
      else {
        this.wd_north = 0.0;
      }
      if (initObj.hasOwnProperty('wd_upwards')) {
        this.wd_upwards = initObj.wd_upwards
      }
      else {
        this.wd_upwards = 0.0;
      }
      if (initObj.hasOwnProperty('wd_range')) {
        this.wd_range = initObj.wd_range
      }
      else {
        this.wd_range = 0.0;
      }
      if (initObj.hasOwnProperty('wd_time')) {
        this.wd_time = initObj.wd_time
      }
      else {
        this.wd_time = 0.0;
      }
      if (initObj.hasOwnProperty('bd_east')) {
        this.bd_east = initObj.bd_east
      }
      else {
        this.bd_east = 0.0;
      }
      if (initObj.hasOwnProperty('bd_north')) {
        this.bd_north = initObj.bd_north
      }
      else {
        this.bd_north = 0.0;
      }
      if (initObj.hasOwnProperty('bd_upwards')) {
        this.bd_upwards = initObj.bd_upwards
      }
      else {
        this.bd_upwards = 0.0;
      }
      if (initObj.hasOwnProperty('bd_range')) {
        this.bd_range = initObj.bd_range
      }
      else {
        this.bd_range = 0.0;
      }
      if (initObj.hasOwnProperty('bd_time')) {
        this.bd_time = initObj.bd_time
      }
      else {
        this.bd_time = 0.0;
      }
      if (initObj.hasOwnProperty('raw_data')) {
        this.raw_data = initObj.raw_data
      }
      else {
        this.raw_data = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DVL
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [header_dvl]
    bufferOffset = _serializer.string(obj.header_dvl, buffer, bufferOffset);
    // Serialize message field [date]
    bufferOffset = _serializer.string(obj.date, buffer, bufferOffset);
    // Serialize message field [salinity]
    bufferOffset = _serializer.float64(obj.salinity, buffer, bufferOffset);
    // Serialize message field [temperature]
    bufferOffset = _serializer.float64(obj.temperature, buffer, bufferOffset);
    // Serialize message field [depth]
    bufferOffset = _serializer.float64(obj.depth, buffer, bufferOffset);
    // Serialize message field [sound_speed]
    bufferOffset = _serializer.float64(obj.sound_speed, buffer, bufferOffset);
    // Serialize message field [test]
    bufferOffset = _serializer.int32(obj.test, buffer, bufferOffset);
    // Serialize message field [wi_x_axis]
    bufferOffset = _serializer.float64(obj.wi_x_axis, buffer, bufferOffset);
    // Serialize message field [wi_y_axis]
    bufferOffset = _serializer.float64(obj.wi_y_axis, buffer, bufferOffset);
    // Serialize message field [wi_z_axis]
    bufferOffset = _serializer.float64(obj.wi_z_axis, buffer, bufferOffset);
    // Serialize message field [wi_error]
    bufferOffset = _serializer.float64(obj.wi_error, buffer, bufferOffset);
    // Serialize message field [wi_status]
    bufferOffset = _serializer.string(obj.wi_status, buffer, bufferOffset);
    // Serialize message field [bi_x_axis]
    bufferOffset = _serializer.float64(obj.bi_x_axis, buffer, bufferOffset);
    // Serialize message field [bi_y_axis]
    bufferOffset = _serializer.float64(obj.bi_y_axis, buffer, bufferOffset);
    // Serialize message field [bi_z_axis]
    bufferOffset = _serializer.float64(obj.bi_z_axis, buffer, bufferOffset);
    // Serialize message field [bi_error]
    bufferOffset = _serializer.float64(obj.bi_error, buffer, bufferOffset);
    // Serialize message field [bi_status]
    bufferOffset = _serializer.string(obj.bi_status, buffer, bufferOffset);
    // Serialize message field [ws_transverse]
    bufferOffset = _serializer.float64(obj.ws_transverse, buffer, bufferOffset);
    // Serialize message field [ws_longitudinal]
    bufferOffset = _serializer.float64(obj.ws_longitudinal, buffer, bufferOffset);
    // Serialize message field [ws_normal]
    bufferOffset = _serializer.float64(obj.ws_normal, buffer, bufferOffset);
    // Serialize message field [ws_status]
    bufferOffset = _serializer.string(obj.ws_status, buffer, bufferOffset);
    // Serialize message field [bs_transverse]
    bufferOffset = _serializer.float64(obj.bs_transverse, buffer, bufferOffset);
    // Serialize message field [bs_longitudinal]
    bufferOffset = _serializer.float64(obj.bs_longitudinal, buffer, bufferOffset);
    // Serialize message field [bs_normal]
    bufferOffset = _serializer.float64(obj.bs_normal, buffer, bufferOffset);
    // Serialize message field [bs_status]
    bufferOffset = _serializer.string(obj.bs_status, buffer, bufferOffset);
    // Serialize message field [we_east]
    bufferOffset = _serializer.float64(obj.we_east, buffer, bufferOffset);
    // Serialize message field [we_north]
    bufferOffset = _serializer.float64(obj.we_north, buffer, bufferOffset);
    // Serialize message field [we_upwards]
    bufferOffset = _serializer.float64(obj.we_upwards, buffer, bufferOffset);
    // Serialize message field [we_status]
    bufferOffset = _serializer.string(obj.we_status, buffer, bufferOffset);
    // Serialize message field [be_east]
    bufferOffset = _serializer.float64(obj.be_east, buffer, bufferOffset);
    // Serialize message field [be_north]
    bufferOffset = _serializer.float64(obj.be_north, buffer, bufferOffset);
    // Serialize message field [be_upwards]
    bufferOffset = _serializer.float64(obj.be_upwards, buffer, bufferOffset);
    // Serialize message field [be_status]
    bufferOffset = _serializer.string(obj.be_status, buffer, bufferOffset);
    // Serialize message field [wd_east]
    bufferOffset = _serializer.float64(obj.wd_east, buffer, bufferOffset);
    // Serialize message field [wd_north]
    bufferOffset = _serializer.float64(obj.wd_north, buffer, bufferOffset);
    // Serialize message field [wd_upwards]
    bufferOffset = _serializer.float64(obj.wd_upwards, buffer, bufferOffset);
    // Serialize message field [wd_range]
    bufferOffset = _serializer.float64(obj.wd_range, buffer, bufferOffset);
    // Serialize message field [wd_time]
    bufferOffset = _serializer.float64(obj.wd_time, buffer, bufferOffset);
    // Serialize message field [bd_east]
    bufferOffset = _serializer.float64(obj.bd_east, buffer, bufferOffset);
    // Serialize message field [bd_north]
    bufferOffset = _serializer.float64(obj.bd_north, buffer, bufferOffset);
    // Serialize message field [bd_upwards]
    bufferOffset = _serializer.float64(obj.bd_upwards, buffer, bufferOffset);
    // Serialize message field [bd_range]
    bufferOffset = _serializer.float64(obj.bd_range, buffer, bufferOffset);
    // Serialize message field [bd_time]
    bufferOffset = _serializer.float64(obj.bd_time, buffer, bufferOffset);
    // Serialize message field [raw_data]
    bufferOffset = _serializer.string(obj.raw_data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DVL
    let len;
    let data = new DVL(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [header_dvl]
    data.header_dvl = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [date]
    data.date = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [salinity]
    data.salinity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [temperature]
    data.temperature = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [depth]
    data.depth = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [sound_speed]
    data.sound_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [test]
    data.test = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [wi_x_axis]
    data.wi_x_axis = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [wi_y_axis]
    data.wi_y_axis = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [wi_z_axis]
    data.wi_z_axis = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [wi_error]
    data.wi_error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [wi_status]
    data.wi_status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [bi_x_axis]
    data.bi_x_axis = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bi_y_axis]
    data.bi_y_axis = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bi_z_axis]
    data.bi_z_axis = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bi_error]
    data.bi_error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bi_status]
    data.bi_status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [ws_transverse]
    data.ws_transverse = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ws_longitudinal]
    data.ws_longitudinal = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ws_normal]
    data.ws_normal = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ws_status]
    data.ws_status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [bs_transverse]
    data.bs_transverse = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bs_longitudinal]
    data.bs_longitudinal = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bs_normal]
    data.bs_normal = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bs_status]
    data.bs_status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [we_east]
    data.we_east = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [we_north]
    data.we_north = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [we_upwards]
    data.we_upwards = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [we_status]
    data.we_status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [be_east]
    data.be_east = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [be_north]
    data.be_north = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [be_upwards]
    data.be_upwards = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [be_status]
    data.be_status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [wd_east]
    data.wd_east = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [wd_north]
    data.wd_north = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [wd_upwards]
    data.wd_upwards = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [wd_range]
    data.wd_range = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [wd_time]
    data.wd_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bd_east]
    data.bd_east = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bd_north]
    data.bd_north = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bd_upwards]
    data.bd_upwards = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bd_range]
    data.bd_range = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bd_time]
    data.bd_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [raw_data]
    data.raw_data = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.header_dvl.length;
    length += object.date.length;
    length += object.wi_status.length;
    length += object.bi_status.length;
    length += object.ws_status.length;
    length += object.bs_status.length;
    length += object.we_status.length;
    length += object.be_status.length;
    length += object.raw_data.length;
    return length + 312;
  }

  static datatype() {
    // Returns string type for a message object
    return 'underwater_sensor_msgs/DVL';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c455f331872552e620e26fc7caad335d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    string header_dvl
    string date 
    float64 salinity 
    float64 temperature   	# TODO::ARNAU ha de ser generic ?
    float64 depth  		# TODO::ARNAU ha de ser generic
    float64 sound_speed 
    int32 test
    #WATER-MASS, INSTRUMENT-REFERENCED VELOCITY DATA
    float64 wi_x_axis 
    float64 wi_y_axis 
    float64 wi_z_axis 
    float64 wi_error 
    string wi_status 
    #BOTTOM-TRACK, INSTRUMENT-REFERENCED VELOCITY DATA
    float64 bi_x_axis 
    float64 bi_y_axis 
    float64 bi_z_axis 
    float64 bi_error 
    string bi_status 
    #WATER-MASS, SHIP-REFERENCED VELOCITY DATA
    float64 ws_transverse 
    float64 ws_longitudinal 
    float64 ws_normal 
    string ws_status 
    #BOTTOM-TRACK, SHIP-REFERENCED VELOCITY DATA
    float64 bs_transverse 
    float64 bs_longitudinal 
    float64 bs_normal 
    string bs_status 
    #WATER-MASS, EARTH-REFERENCED VELOCITY DATA
    float64 we_east 
    float64 we_north 
    float64 we_upwards 
    string  we_status 
    #BOTTOM-TRACK, EARTH-REFERENCED VELOCITY DATA
    float64 be_east 
    float64 be_north 
    float64 be_upwards 
    string  be_status 
    #  WATER-MASS, EARTH-REFERENCED DISTANCE DATA
    float64 wd_east 
    float64 wd_north 
    float64 wd_upwards 
    float64 wd_range 
    float64 wd_time 
    #BOTTOM-TRACK, EARTH-REFERENCED DISTANCE DATA
    float64 bd_east 
    float64 bd_north 
    float64 bd_upwards 
    float64 bd_range 
    float64 bd_time
    #RAW DATA 
    string raw_data
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DVL(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.header_dvl !== undefined) {
      resolved.header_dvl = msg.header_dvl;
    }
    else {
      resolved.header_dvl = ''
    }

    if (msg.date !== undefined) {
      resolved.date = msg.date;
    }
    else {
      resolved.date = ''
    }

    if (msg.salinity !== undefined) {
      resolved.salinity = msg.salinity;
    }
    else {
      resolved.salinity = 0.0
    }

    if (msg.temperature !== undefined) {
      resolved.temperature = msg.temperature;
    }
    else {
      resolved.temperature = 0.0
    }

    if (msg.depth !== undefined) {
      resolved.depth = msg.depth;
    }
    else {
      resolved.depth = 0.0
    }

    if (msg.sound_speed !== undefined) {
      resolved.sound_speed = msg.sound_speed;
    }
    else {
      resolved.sound_speed = 0.0
    }

    if (msg.test !== undefined) {
      resolved.test = msg.test;
    }
    else {
      resolved.test = 0
    }

    if (msg.wi_x_axis !== undefined) {
      resolved.wi_x_axis = msg.wi_x_axis;
    }
    else {
      resolved.wi_x_axis = 0.0
    }

    if (msg.wi_y_axis !== undefined) {
      resolved.wi_y_axis = msg.wi_y_axis;
    }
    else {
      resolved.wi_y_axis = 0.0
    }

    if (msg.wi_z_axis !== undefined) {
      resolved.wi_z_axis = msg.wi_z_axis;
    }
    else {
      resolved.wi_z_axis = 0.0
    }

    if (msg.wi_error !== undefined) {
      resolved.wi_error = msg.wi_error;
    }
    else {
      resolved.wi_error = 0.0
    }

    if (msg.wi_status !== undefined) {
      resolved.wi_status = msg.wi_status;
    }
    else {
      resolved.wi_status = ''
    }

    if (msg.bi_x_axis !== undefined) {
      resolved.bi_x_axis = msg.bi_x_axis;
    }
    else {
      resolved.bi_x_axis = 0.0
    }

    if (msg.bi_y_axis !== undefined) {
      resolved.bi_y_axis = msg.bi_y_axis;
    }
    else {
      resolved.bi_y_axis = 0.0
    }

    if (msg.bi_z_axis !== undefined) {
      resolved.bi_z_axis = msg.bi_z_axis;
    }
    else {
      resolved.bi_z_axis = 0.0
    }

    if (msg.bi_error !== undefined) {
      resolved.bi_error = msg.bi_error;
    }
    else {
      resolved.bi_error = 0.0
    }

    if (msg.bi_status !== undefined) {
      resolved.bi_status = msg.bi_status;
    }
    else {
      resolved.bi_status = ''
    }

    if (msg.ws_transverse !== undefined) {
      resolved.ws_transverse = msg.ws_transverse;
    }
    else {
      resolved.ws_transverse = 0.0
    }

    if (msg.ws_longitudinal !== undefined) {
      resolved.ws_longitudinal = msg.ws_longitudinal;
    }
    else {
      resolved.ws_longitudinal = 0.0
    }

    if (msg.ws_normal !== undefined) {
      resolved.ws_normal = msg.ws_normal;
    }
    else {
      resolved.ws_normal = 0.0
    }

    if (msg.ws_status !== undefined) {
      resolved.ws_status = msg.ws_status;
    }
    else {
      resolved.ws_status = ''
    }

    if (msg.bs_transverse !== undefined) {
      resolved.bs_transverse = msg.bs_transverse;
    }
    else {
      resolved.bs_transverse = 0.0
    }

    if (msg.bs_longitudinal !== undefined) {
      resolved.bs_longitudinal = msg.bs_longitudinal;
    }
    else {
      resolved.bs_longitudinal = 0.0
    }

    if (msg.bs_normal !== undefined) {
      resolved.bs_normal = msg.bs_normal;
    }
    else {
      resolved.bs_normal = 0.0
    }

    if (msg.bs_status !== undefined) {
      resolved.bs_status = msg.bs_status;
    }
    else {
      resolved.bs_status = ''
    }

    if (msg.we_east !== undefined) {
      resolved.we_east = msg.we_east;
    }
    else {
      resolved.we_east = 0.0
    }

    if (msg.we_north !== undefined) {
      resolved.we_north = msg.we_north;
    }
    else {
      resolved.we_north = 0.0
    }

    if (msg.we_upwards !== undefined) {
      resolved.we_upwards = msg.we_upwards;
    }
    else {
      resolved.we_upwards = 0.0
    }

    if (msg.we_status !== undefined) {
      resolved.we_status = msg.we_status;
    }
    else {
      resolved.we_status = ''
    }

    if (msg.be_east !== undefined) {
      resolved.be_east = msg.be_east;
    }
    else {
      resolved.be_east = 0.0
    }

    if (msg.be_north !== undefined) {
      resolved.be_north = msg.be_north;
    }
    else {
      resolved.be_north = 0.0
    }

    if (msg.be_upwards !== undefined) {
      resolved.be_upwards = msg.be_upwards;
    }
    else {
      resolved.be_upwards = 0.0
    }

    if (msg.be_status !== undefined) {
      resolved.be_status = msg.be_status;
    }
    else {
      resolved.be_status = ''
    }

    if (msg.wd_east !== undefined) {
      resolved.wd_east = msg.wd_east;
    }
    else {
      resolved.wd_east = 0.0
    }

    if (msg.wd_north !== undefined) {
      resolved.wd_north = msg.wd_north;
    }
    else {
      resolved.wd_north = 0.0
    }

    if (msg.wd_upwards !== undefined) {
      resolved.wd_upwards = msg.wd_upwards;
    }
    else {
      resolved.wd_upwards = 0.0
    }

    if (msg.wd_range !== undefined) {
      resolved.wd_range = msg.wd_range;
    }
    else {
      resolved.wd_range = 0.0
    }

    if (msg.wd_time !== undefined) {
      resolved.wd_time = msg.wd_time;
    }
    else {
      resolved.wd_time = 0.0
    }

    if (msg.bd_east !== undefined) {
      resolved.bd_east = msg.bd_east;
    }
    else {
      resolved.bd_east = 0.0
    }

    if (msg.bd_north !== undefined) {
      resolved.bd_north = msg.bd_north;
    }
    else {
      resolved.bd_north = 0.0
    }

    if (msg.bd_upwards !== undefined) {
      resolved.bd_upwards = msg.bd_upwards;
    }
    else {
      resolved.bd_upwards = 0.0
    }

    if (msg.bd_range !== undefined) {
      resolved.bd_range = msg.bd_range;
    }
    else {
      resolved.bd_range = 0.0
    }

    if (msg.bd_time !== undefined) {
      resolved.bd_time = msg.bd_time;
    }
    else {
      resolved.bd_time = 0.0
    }

    if (msg.raw_data !== undefined) {
      resolved.raw_data = msg.raw_data;
    }
    else {
      resolved.raw_data = ''
    }

    return resolved;
    }
};

module.exports = DVL;
