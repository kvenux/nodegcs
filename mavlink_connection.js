var SerialPort = require('serialport').SerialPort;
var mavlink = require('mavlink');
var net = require('net');
var EventEmitter = require("events").EventEmitter;
var emitter = new EventEmitter();

var connection_type = 3;
var connection_path = '';
var connection_port = 5760;
var connection_baud = 57600;

var mavheader = require('./mavheader.js')

var serial = require("serialport");
function get_port_list() {
  /*serial = require("serialport");*/
  serial.list(function (err, ports) {
    emitter.emit('serial_list', ports);
  });
}
get_port_list();

// data stream definition
var MAV_DATA_STREAM = {
  MAV_DATA_STREAM_ALL: 0,
  MAV_DATA_STREAM_RAW_SENSORS: 1,
  MAV_DATA_STREAM_EXTENDED_STATUS: 2,
  MAV_DATA_STREAM_RC_CHANNELS: 3,
  MAV_DATA_STREAM_RAW_CONTROLLER: 4,
  MAV_DATA_STREAM_POSITION: 6,
  MAV_DATA_STREAM_EXTRA1: 10,
  MAV_DATA_STREAM_EXTRA2: 11,
  MAV_DATA_STREAM_EXTRA3: 12,
  MAV_DATA_STREAM_ENUM_END: 13
}

var APM_MODE = {
  STABILIZE: 0,  // manual airframe angle with manual throttle
  ACRO: 1,  // manual body-frame angular rate with manual throttle
  ALT_HOLD: 2,  // manual airframe angle with automatic throttle
  AUTO: 3,  // fully automatic waypoint control using mission commands
  GUIDED: 4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
  LOITER: 5,  // automatic horizontal acceleration with automatic throttle
  RTL: 6,  // automatic return to launching point
  CIRCLE: 7,  // automatic circular flight with automatic throttle
  LAND: 9,  // automatic landing with horizontal position control
  DRIFT: 11,  // semi-automous position, yaw and throttle control
  SPORT: 13,  // manual earth-frame angular rate control with manual throttle
  FLIP: 14,  // automatically flip the vehicle on the roll axis
  AUTOTUNE: 15,  // automatically tune the vehicle's roll and pitch gains
  POSHOLD: 16,  // automatic position hold with manual override, with automatic throttle
  BRAKE: 17,  // full-brake using inertial/GPS system, no pilot input
  THROW: 18   // throw to launch mode using inertial/GPS system, no pilot input
};

var APM_MODE_STR = [
  'STABILIZE',
  'ACRO',
  'ALT_HOLD',
  'AUTO',
  'GUIDED',
  'LOITER',
  'RTL',
  'CIRCLE',
  'LAND',
  'LAND',
  'DRIFT',
  'SPORT',
  'FLIP',
  'AUTOTUNE',
  'POSHOLD',
  'BRAKE',
  'THROW'
];

var mav_port = null;
var mav_parser = null;

var system_ID = 1;
var component_ID = 1;
var heartbeat_enabled = true;
var heartbeat_timer;
var datastream_timer;

var gcsDate = new Date();
var timeout_interval_heartbeat = 5000;
var last_heartbeat = 0;
var connection_lost = true;
var connection_losstime = 0;
// true if mode was retrieved from current connection to UAS
var received_mode = false;

var send_default_stream_period = 20000;

var base_status = {
  system_is_armed: false,
  armed: 0,
  custom_mode: 0,
  base_mode: 0,
  frame_type: 0,
  autopilot_type: 0,
  system_status: 0
}

var sys_status = {
  cpu_load: 0,
  sensors_present: 0,
  sensors_enabled: 0,
  sensors_health: 0,
  battery_voltage: 0,
  battery_current: 0,
  battery_remaining: 0
}

var attitude = {
  time_boot_ms: 0,
  roll: 0,
  pitch: 0,
  yaw: 0,
  rollspeed: 0,
  pitchspeed: 0,
  yawspeed: 0
}

var global_position = {
  lat: 0,
  lon: 0,
  alt: 0,
  relative_alt: 0,
  vx: 0,
  xy: 0,
  vz: 0,
  hdg: 0
}

var gps_raw_int = {
  time_usec: 0,
  lat: 0,
  lon: 0,
  alt: 0,
  eph: 9999,
  epv: 65535,
  vel: 0,
  cog: 0,
  fix_type: 0,
  satellites_visible: 0
}


var current_mission_seq = 0;
var mission_list = [];
var mission_len = 0;

var MISSION_TRANS_STATE_IDLE = 0;
var MISSION_TRANS_STATE_TRANS = 1;
var mission_trans_state = MISSION_TRANS_STATE_IDLE;
var current_trans_seq = -1;
var mission_trans_start_time = 0;
var mission_trans_end_time = 0;

var DOWNLOADING_NUM_AT_A_TIME = 2;
// to measure downloading and uploading speed using radio
var measurement_period = 1000;
var down_measurement_time = 0;
var down_bytes_per_sec = 0;
var up_measurement_time = 0;
var up_bytes_per_sec = 0;

var seq_cur_cnt = 0;
var seq_last = -1;

function open_serial() {
  emitter.emit('connect_procedure', 1 / 3);
  mav_parser = new mavlink(system_ID, 1);

  //When mavlink is ready, assign some listeners
  mav_parser.on('ready', function () {
    emitter.emit('connect_procedure', 2 / 3);
    emitter.emit('serial_opened');
    mav_port.on('data', function (data) {
      if(data[0]!= 0 && data[1]!= 7){
        var radio_status = {
          rssi: parseInt(data[2]),
        };
      }
      mav_parser.parse(data);
    });

    mav_parser.on('message', function (message, fields) {
      if (message.id == 26) {
        emitter.emit('status_text', message);
      }
      var seq = message.sequence;
      if (seq_last != -1) {
        var gap = seq - seq_last;
        if (gap < 0) {
          seq_cur_cnt = 0;
        }
        else {
          seq_cur_cnt += gap - 1;
        }
      }
      seq_last = seq;
    });

    mav_parser.on('RADIO', function (message, fields) {
    });
    mav_parser.on('RADIO_STATUS', function (message, fields) {
    });


    // HEARTBEAT message
    mav_parser.on('HEARTBEAT', function (message, fields) {
      gcsDate = new Date();
      /*emitter.emit('status_text', gcsDate.getTime() - last_heartbeat);*/
      last_heartbeat = gcsDate.getTime();
      curently_armed = Boolean(fields.base_mode & mavheader.MAV_MODE_FLAG_DECODE_POSITION_SAFETY);
      if (curently_armed != base_status.system_is_armed) {
        // here emit arming changed message
        //base_status.armed = curently_armed;
        base_status.system_is_armed = curently_armed;
        if (curently_armed) {
          // is armed
          emitter.emit('arming');
        }
        else {
          // is dis armed
          emitter.emit('disarming');
        }
      }
      if (base_status.custom_mode != fields.custom_mode) {
        // emit custom mode changed message
        base_status.custom_mode = fields.custom_mode;
      }
      received_mode = true;
      base_status.base_mode = fields.base_mode;
      base_status.frame_type = fields.type;
      base_status.autopilot_type = fields.autopilot;
      base_status.system_status = fields.system_status;
      /*emitter.emit('status_text', base_status);*/
      /*emitter.emit('status_text', "Cur Mode: " +base_status.custom_mode);*/
      /*emitter.emit('status_text', "Arming State: " +base_status.system_is_armed);*/
      // emit heartbeat message
      emitter.emit('heartbeat', base_status);
    });

    // SYSTEM_STATUS message
    mav_parser.on('SYS_STATUS', function (message, fields) {
      /*emitter.emit('status_text', "Roll is " + fields.roll + "\nPitch is " + fields.pitch);*/
      /*emitter.emit('status_text', fields);*/
      sys_status.cpu_load = fields.load * 10.0;
      sys_status.sensors_present = fields.onboard_control_sensors_present;
      sys_status.sensors_enabled = fields.onboard_control_sensors_enabled;
      sys_status.sensors_health = fields.onboard_control_sensors_health;
      sys_status.battery_voltage = fields.voltage_battery;
      sys_status.battery_current = fields.current_battery;
      sys_status.battery_remaining = fields.battery_remaining;
      emitter.emit('sys_status', sys_status);
    });

    // ATTITUDE message
    mav_parser.on('ATTITUDE', function (message, fields) {
      attitude.roll = fields.roll;
      attitude.pitch = fields.pitch;
      attitude.yaw = fields.yaw;
      attitude.rollspeed = fields.rollspeed;
      attitude.pitchspeed = fields.pitchspeed;
      attitude.yawspeed = fields.yawspeed;
      // here emit attitude changed message
      emitter.emit('attitude', fields);
    });

    // ATTITUDE message
    mav_parser.on('LOCAL_POSITION', function (message, fields) {
      emitter.emit('status_text', fields);
    });

    // GLOBAL_POSITION_INT message
    mav_parser.on('GLOBAL_POSITION_INT', function (message, fields) {
      global_position.lat = fields.lat;
      global_position.lon = fields.lon;
      global_position.alt = fields.alt;
      global_position.relative_alt = fields.relative_alt;
      global_position.vx = fields.vx;
      global_position.vy = fields.vy;
      global_position.vz = fields.vz;
      global_position.hdg = fields.hdg;
      emitter.emit('global_postion', fields);
    });

    // GPS_RAW_INT message
    mav_parser.on('GPS_RAW_INT', function (message, fields) {
      gps_raw_int = fields;
      emitter.emit('gps_raw_int', gps_raw_int);
    });

    // RC_CHANNELS_RAW message
    mav_parser.on('RC_CHANNELS_RAW', function (message, fields) {
    });

    // RC_CHANNELS_RAW message
    mav_parser.on('RC_CHANNELS_SCALED', function (message, fields) {
      //emitter.emit('rc_channels', fields);
    });

    // SERVO_OUTPUT_RAW message
    mav_parser.on('SERVO_OUTPUT_RAW', function (message, fields) {
      emitter.emit('servo_output', fields);
    });

    // STATUS_TEXT message
    mav_parser.on('STATUSTEXT', function (message, fields) {
      var status_text = fields.text;
      var ARMING = 'ARMING MOTORS';
      var DISARMING = 'DISARMING MOTORS';
      var PRE_ARM = 'PreArm';
      var REACHED = 'Reached Command #';
      var LOW_BAT = 'Low battery';
      emitter.emit('status_text', status_text);
      if (status_text.indexOf('DISARMING') >= 0) {
        emitter.emit('disarming', status_text);
      }
      else if (status_text.indexOf('ARMING') >= 0) {
        emitter.emit('arming', status_text);
      }
      if (status_text.indexOf(PRE_ARM) >= 0) {
        emitter.emit('prearm', status_text);
      }
      if (status_text.indexOf(REACHED) >= 0) {
        var reached_index = parseInt(status_text.substring(REACHED.length));
        emitter.emit('reached', get_gcs_seq(reached_index));
      }
      if (status_text.indexOf(LOW_BAT) >= 0) {
        emitter.emit('low_battery', status_text);
      }
    });

    //  message
    mav_parser.on('RAW_IMU', function (message, fields) {
      emitter.emit('raw_imu', fields);
    });

    //  message
    mav_parser.on('SCALED_IMU2', function (message, fields) {
      emitter.emit('raw_imu2', fields);
    });

    // NAV_CONTROLLER_OUTPUT message
    mav_parser.on('NAV_CONTROLLER_OUTPUT', function (message, fields) {
    });

    // MEMINFO message
    mav_parser.on('MEMINFO', function (message, fields) {
    });

    // MISSION_CURRENT message
    mav_parser.on('MISSION_CURRENT', function (message, fields) {
      if (current_mission_seq != fields.seq && fields.seq < mission_list.length) {
        if (current_mission_seq < fields.seq) {
          emitter.emit('current_cmd', mission_list[fields.seq]);
        }
      }
      current_mission_seq = fields.seq;
      var gcs_seq = get_gcs_seq(fields.seq);
      var ret_fields = {
        seq: gcs_seq
      }
      emitter.emit('current_mission', ret_fields);
      if (fields.seq == mission_list.length - 1) {
        //emitter.emit('mission_finished');
      }
    });

    // VFR_HUD message
    mav_parser.on('VFR_HUD', function (message, fields) {
    });

    // AHRS message
    mav_parser.on('AHRS', function (message, fields) {
    });

    // SYSTEM_TIME message
    mav_parser.on('SYSTEM_TIME', function (message, fields) {
    });

    // HWSTATUS message
    mav_parser.on('HWSTATUS', function (message, fields) {
    });

    // VIBRATION message
    mav_parser.on('VIBRATION', function (message, fields) {
    });

    // COMMAND_ACK message
    mav_parser.on('COMMAND_ACK', function (message, fields) {
      emitter.emit('command_ack', fields);
    });

    // AUTOPILOT_VERSION message
    mav_parser.on('AUTOPILOT_VERSION', function (message, fields) {
    });

    // MISSION_ITEM message
    mav_parser.on('MISSION_ITEM', function (message, fields) {
      var mission_index = fields.seq;
      if (mission_index >= 0 && mission_index < mission_len) {
        emitter.emit('reading_next', mission_index);
        mission_list[mission_index] = fields;
        emitter.emit('reading_single_mission', fields);
        if (mission_index == mission_len - 1) {
          emitter.emit('status_text', 'INFO: Mission reading finished');
          var mission_procedure = {
            index: mission_len,
            len: mission_len
          };
          emitter.emit('downloading_procedure', mission_procedure);
          emitter.emit('reading_completed', mission_to_gcs());
        }
        else {
          var mission_procedure = {
            index: mission_index + 1,
            len: mission_len
          };
          emitter.emit('downloading_procedure', mission_procedure);
          if (mission_index + DOWNLOADING_NUM_AT_A_TIME < mission_len) {
            mission_request_individual(mission_index + DOWNLOADING_NUM_AT_A_TIME);
          }
        }
      }
      else {
        emitter.emit('status_text', 'ERROR: Receive seq not in regular range');
      }
    });

    // MISSION_COUNT message
    mav_parser.on('MISSION_COUNT', function (message, fields) {
      mission_len = fields.count;
      mission_get_all();
    });

    // MISSION_ACK message
    mav_parser.on('MISSION_ACK', function (message, fields) {
      if (mission_trans_state == MISSION_TRANS_STATE_TRANS) {
        mission_trans_state = MISSION_TRANS_STATE_IDLE;
        emitter.emit('status_text', 'Mission uploading finished');
        gcsDate = new Date();
        mission_trans_end_time = gcsDate.getTime();
        var mission_trans_time = mission_trans_end_time - mission_trans_start_time;
        emitter.emit('status_text', 'Mission trans time using ' + mission_trans_time);
        var mission_procedure = {
          index: mission_len,
          len: mission_len
        };
        emitter.emit('uploading_procedure', mission_procedure);
        emitter.emit('uploading_finished', fields);
      }
      else {
        emitter.emit('status_text', 'Unexpected MISSION_ACK');
      }
    });

    // MISSION_ACK message
    mav_parser.on('MISSION_REQUEST', function (message, fields) {
      emitter.emit('status_text', fields);
      var mission_index = fields.seq;
      var mission_procedure = {
        index: mission_index + 1,
        len: mission_len
      };
      if (mission_index < 0 && mission_index > mission_len) {
        emitter.emit('status_text', 'ERROR: Receive seq not in regular range');
        return;
      }
      if (mission_trans_state == MISSION_TRANS_STATE_TRANS) {
        mission_write_waypoint(mission_list[mission_index]);
        emitter.emit('uploading_procedure', mission_procedure);
      }
    });

    //  message
    mav_parser.on('RADIO', function (message, fields) {
      emitter.emit('status_text', fields);
    });

    //  message
    mav_parser.on('PARAM_VALUE', function (message, fields) {
      emitter.emit('status_text', fields);
    });

  });
}

function close_serial() {
  mav_port.close();
}

function serial_closed_callback() {
  emitter.emit('serial_closed');
  emitter.emit('status_text', 'port closed callback');
}

function show_error_callback(error) {
  if (error.message.indexOf('Port is opening') >= 0) {
    emitter.emit('status_text', error + ' --which can be ignored');
    return;
  }
  emitter.emit('status_text', 'serial port error: ' + error)
  clearInterval(heartbeat_timer);
  clearInterval(datastream_timer);
  clearInterval(update_heartbeat_timer);
  //mav_port.pause();
  mav_port.close();
  emitter.emit('serial_error', error);
  if (error.message.indexOf('Access denied') >= 0) {
    clearInterval(heartbeat_timer);
    clearInterval(datastream_timer);
    clearInterval(update_heartbeat_timer);
    mav_port.close();
  }
  else {
    clearInterval(heartbeat_timer);
    clearInterval(datastream_timer);
    clearInterval(update_heartbeat_timer);
    mav_port.close();
  }
}

function connect_serial() {
  //port configuration
  var usb_name = "/dev/cu.usbmodem1"
  var usb_baudrate = 115200;
  var radio1_name = "/dev/cu.usbserial-A503UTCT"
  var radio2_name = "/dev/cu.usbserial-AJ032N5Q"
  var radio_baudrate = 57600;

  // via usb
  if (connection_type == 0) {
    mav_port = new SerialPort(connection_path, {
      baudrate: connection_baud
    });
  }
  // via radio
  else if (connection_type == 1) {
    mav_port = new net.Socket();
    mav_port.connect(connection_port, connection_path, function () {
      emitter.emit('status_text', 'Connected to sitl via tcp:' + connection_path + ':' + connection_port);
    });
  }
  else {
    return;
  }

  if (connection_type == 1) {
    setTimeout(open_serial, 500);
  }
  else {
    emitter.emit('status_text', 'Connected to uav via serial:' + connection_path + ':' + connection_baud);
    if (mav_port.isOpen()) {
      mav_port.close();
    }
    mav_port.open(open_serial);
    /*mav_port.on('open', open_serial);*/
    mav_port.on('error', show_error_callback);
    mav_port.on('close', serial_closed_callback);
  }
}

/**
 * Update the heartbeat
 */
function update_heartbeat() {
  gcsDate = new Date();
  var heartbeat_interval = gcsDate.getTime() - last_heartbeat;
  // check if heartbeat time out
  if (!connection_lost && (heartbeat_interval > timeout_interval_heartbeat)) {
    connection_lost = true;
    received_mode = false;
    // here to emit connection lost message
    emitter.emit('status_text', "Connection lost");
    emitter.emit('connection_lost');
  }

  // update connection loss time
  if (connection_lost && (heartbeat_interval > timeout_interval_heartbeat)) {
    connection_losstime = heartbeat_interval;
    /*emitter.emit('status_text', "Conection has last for " + connection_losstime/1000 + " seconds");*/
  }

  // connection regained
  if (connection_lost && (heartbeat_interval < timeout_interval_heartbeat)) {
    connection_lost = false;
    connection_losstime = 0;
    // here to emit link regained message
    emitter.emit('connection_regained');
    emitter.emit('connect_procedure', 1);
  }

}

function port_send_message(message) {
  if (mav_port == null) {
    emitter.emit('status_text', 'ERROR: port not initializded!');
    return;
  }
  if (connection_type == 1) {
    mav_port.write(message.buffer);
    return;
  }
  if (mav_port.isOpen()) {
    mav_port.write(message.buffer);
    var now_date = new Date();
    var new_measurement_time = parseInt(now_date.getTime() / measurement_period);
    if (new_measurement_time != up_measurement_time) {
      emitter.emit('up_speed', up_bytes_per_sec / (new_measurement_time - up_measurement_time));
      up_bytes_per_sec = 0;
    }
    else {
      up_bytes_per_sec += message.buffer.length;
    }
    up_measurement_time = new_measurement_time;
  }
  else {
    /*emitter.emit('status_text', 'ERROR: sending msg error on '+ message.id);*/
    emitter.emit('status_text', 'ERROR: Port not open! On msg id ' + message.id);
  }
}

function port_send_customed_control(control_msg) {
  if (mav_port == null) {
    emitter.emit('status_text', 'ERROR: port not initializded!');
    return;
  }
  var msg_buf = new Buffer(20);
  msg_buf[0] = parseInt('fd', 16);
  msg_buf[1] = 16;
  msg_buf[2] = parseInt('ff', 16);
  msg_buf[3] = parseInt('01', 16);
  msg_buf[4] = control_msg.chan1 / 256;
  msg_buf[5] = control_msg.chan1 % 256;
  msg_buf[6] = control_msg.chan2 / 256;
  msg_buf[7] = control_msg.chan2 % 256;
  msg_buf[8] = control_msg.chan3 / 256;
  msg_buf[9] = control_msg.chan3 % 256;
  msg_buf[10] = control_msg.chan4 / 256;
  msg_buf[11] = control_msg.chan4 % 256;
  msg_buf[12] = control_msg.chan5 / 256;
  msg_buf[13] = control_msg.chan5 % 256;
  msg_buf[14] = control_msg.chan6 / 256;
  msg_buf[15] = control_msg.chan6 % 256;
  msg_buf[16] = control_msg.chan7 / 256;
  msg_buf[17] = control_msg.chan7 % 256;
  msg_buf[18] = control_msg.chan8 / 256;
  msg_buf[19] = control_msg.chan8 % 256;
  if (connection_type == 1) {
    mav_port.write(msg_buf);
    return;
  }
  if (mav_port.isOpen()) {
    mav_port.write(msg_buf);
  }
  else {
    /*emitter.emit('status_text', 'ERROR: sending msg error on '+ message.id);*/
    emitter.emit('status_text', 'ERROR: Port not open! On msg id ' + message.id);
  }
}

function port_send_switch_control(id, command) {
  if (mav_port == null) {
    emitter.emit('status_text', 'ERROR: port not initializded!');
    return;
  }
  var msg_buf = new Buffer(6);
  msg_buf[0] = parseInt('fd', 16);
  msg_buf[1] = 2;
  msg_buf[2] = parseInt('ff', 16);
  msg_buf[3] = parseInt('02', 16);
  msg_buf[4] = id;
  msg_buf[5] = command;
  if (connection_type == 1) {
    mav_port.write(msg_buf);
    return;
  }
  if (mav_port.isOpen()) {
    mav_port.write(msg_buf);
  }
  else {
    /*emitter.emit('status_text', 'ERROR: sending msg error on '+ message.id);*/
    emitter.emit('status_text', 'ERROR: Port not open! On msg id ' + message.id);
  }
}

var heartbeat_paused = false;
function send_heartbeat() {
  if (heartbeat_paused) return;
  if (heartbeat_enabled && mav_parser != undefined) {
    mav_parser.createMessage("HEARTBEAT", {
      'custom_mode': 0,
      'type': mavheader.MAV_TYPE_GCS,
      'autopilot': mavheader.MAV_AUTOPILOT_INVALID,
      'base_mode': mavheader.MAV_MODE_MANUAL_ARMED,
      'system_status': mavheader.MAV_STATE_ACTIVE,
      'mavlink_version': 3
    },
      function (message) {
        port_send_message(message);
        return;
      });
  }
}

function heartbeat_pause(params) {
  heartbeat_paused = true;
}

function heartbeat_resume(params) {
  heartbeat_paused = false;
}

function set_datastream_rate(id, rate) {
  mav_parser.createMessage("REQUEST_DATA_STREAM", {
    'target_system': system_ID,
    'target_component': 1,
    'req_stream_id': id,
    'req_message_rate': rate,
    'start_stop': 1
  },
    function (message) {
      port_send_message(message);
      return;
    });
}

var mag_cal_datastream = false;
var rates_raw_sensors = 1;
var rates_extended_status = 5;
var rates_rc_channels = 2;
var rates_raw_controller = 1;
var rates_position = 2;
var rates_extra1 = 5;
var rates_extra2 = 0;
var rates_extra3 = 0;

function send_default_datastream() {
  if (mav_parser == undefined)
    return;
  //set_datastream_rate(MAV_DATA_STREAM.MAV_DATA_STREAM_ALL, 0);
  //if(mav_parser != undefined)
  //return;
  if (mag_cal_datastream) {
    set_datastream_rate(MAV_DATA_STREAM.MAV_DATA_STREAM_RAW_SENSORS, 50);
    set_datastream_rate(MAV_DATA_STREAM.MAV_DATA_STREAM_EXTENDED_STATUS, 0);
    set_datastream_rate(MAV_DATA_STREAM.MAV_DATA_STREAM_RC_CHANNELS, 0);
    set_datastream_rate(MAV_DATA_STREAM.MAV_DATA_STREAM_RAW_CONTROLLER, 0);
    set_datastream_rate(MAV_DATA_STREAM.MAV_DATA_STREAM_POSITION, 0);
    set_datastream_rate(MAV_DATA_STREAM.MAV_DATA_STREAM_EXTRA1, 0);
    set_datastream_rate(MAV_DATA_STREAM.MAV_DATA_STREAM_EXTRA2, 0);
    set_datastream_rate(MAV_DATA_STREAM.MAV_DATA_STREAM_EXTRA3, 0);
  }
  else {
    //	Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
    // 5
    setTimeout(function(){
      set_datastream_rate(MAV_DATA_STREAM.MAV_DATA_STREAM_RAW_SENSORS, rates_raw_sensors);
    }, 200);
    // Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
    // 5
    setTimeout(function(){
      set_datastream_rate(MAV_DATA_STREAM.MAV_DATA_STREAM_EXTENDED_STATUS, rates_extended_status);
    }, 200);
    // Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
    // 5
    setTimeout(function(){
      set_datastream_rate(MAV_DATA_STREAM.MAV_DATA_STREAM_RC_CHANNELS, rates_rc_channels);
    }, 200);
    // Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
    // 2
    setTimeout(function(){
      set_datastream_rate(MAV_DATA_STREAM.MAV_DATA_STREAM_RAW_CONTROLLER, rates_raw_controller);
    }, 200);
    // 	Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
    // 3
    setTimeout(function(){
      set_datastream_rate(MAV_DATA_STREAM.MAV_DATA_STREAM_POSITION, rates_position);
    }, 200);
    // ATTITUDE
    // 10
    setTimeout(function(){
      set_datastream_rate(MAV_DATA_STREAM.MAV_DATA_STREAM_EXTRA1, rates_extra1);
    }, 200);
    // 1
    setTimeout(function(){
      set_datastream_rate(MAV_DATA_STREAM.MAV_DATA_STREAM_EXTRA2, rates_extra2);
    }, 200);
    // 2
    setTimeout(function(){
      set_datastream_rate(MAV_DATA_STREAM.MAV_DATA_STREAM_EXTRA3, rates_extra3);
    }, 200);
  }
}

function set_default_stream_rates(rate_0, rate_1, rate_2, rate_3, rate_4, rate_5, rate_6, rate_7) {
  rates_raw_sensors = rate_0;
  rates_extended_status = rate_1;
  rates_rc_channels = rate_2;
  rates_raw_controller = rate_3;
  rates_position = rate_4;
  rates_extra1 = rate_5;
  rates_extra2 = rate_6;
  rates_extra2 = rate_7;
  send_default_datastream();
}

function set_manual_stream_rates() {
  set_default_stream_rates(0, 2, 1, 0, 2, 1, 0, 0);
}

function set_auto_stream_rates() {
  set_default_stream_rates(1, 3, 2, 0, 1, 3, 0, 0);
}

function set_mag_cal_stream(yes_or_no) {
  mag_cal_datastream = yes_or_no;
  send_default_datastream();
}

function send_version() {
  mav_parser.createMessage("AUTOPILOT_VERSION_REQUEST", {
    'target_system': system_ID,
    'target_component': 1
  },
    function (message) {
      port_send_message(message);
      return;
    });
}

function send_command_long(param1, param2, param3, param4, param5, param6, param7, command, confirmation) {
  mav_parser.createMessage("COMMAND_LONG", {
    'param1': param1,
    'param2': param2,
    'param3': param3,
    'param4': param4,
    'param5': param5,
    'param6': param6,
    'param7': param7,
    'command': command,
    'target_system': 1,
    'target_component': 1,
    'confirmation': confirmation
  },
    function (message) {
      port_send_message(message);
      return;
    });
}

function send_banner() {
  send_command_long(0, 0, 0, 0, 0, 0, 0, 42428, 1);
}
//debug area


/*connection_type = 1;*/
/*connection_path = '127.0.0.1';*/
/*connection_port = 5760;*/

/*init_UAS();*/

/*setTimeout(close_serial, 20000);*/
function set_mode_arm(new_base_mode, new_custom_mode) {
  if (received_mode) {
    //send mavlink msg to set base_mode and custom_mode
    mav_parser.createMessage("SET_MODE", {
      'target_system': system_ID,
      'custom_mode': new_custom_mode,
      'base_mode': new_base_mode
    },
      function (message) {
        port_send_message(message);
        return;
      });
  }
}

function set_custom_mode(new_custom_mode) {
  set_mode_arm(base_status.base_mode, new_custom_mode);
}

function set_arming() {
  /*set_mode_arm(base_status.base_mode | mavheader.MAV_MODE_FLAG_SAFETY_ARMED, base_status.custom_mode);*/
  send_command_long(1, 0, 0, 0, 0, 0, 0, mavheader.MAV_CMD_COMPONENT_ARM_DISARM, 1);
}

function set_disarming() {
  /*set_mode_arm(base_status.base_mode & ~mavheader.MAV_MODE_FLAG_SAFETY_ARMED, base_status.custom_mode);*/
  send_command_long(0, 0, 0, 0, 0, 0, 0, mavheader.MAV_CMD_COMPONENT_ARM_DISARM, 1);
}

function get_attitude() {
  return attitude;
}

function get_base_status() {
  return base_status;
}

function get_mode() {
  return base_status.custom_mode;
}

function get_mode_str() {
  return APM_MODE_STR[base_status.custom_mode];
}

function get_gps_raw_int() {
  return gps_raw_int;
}

function get_global_pos() {
  return global_position;
}

function set_connection(type, path, baud_or_port) {
  connection_type = type;
  connection_path = path;
  connection_port = baud_or_port;
  connection_baud = baud_or_port;
  init_UAS();
}

function resume_serial() {
  heartbeat_timer = setInterval(send_heartbeat, 1000);
  datastream_timer = setInterval(send_default_datastream, send_default_stream_period);
  update_heartbeat_timer = setInterval(update_heartbeat, 500);
  last_heartbeat = 0;
  connection_lost = true;
  connection_losstime = 0;
  current_mission_seq = 0;
  mission_list = [];
  mission_len = 0;
  mission_trans_state = MISSION_TRANS_STATE_IDLE;
  current_trans_seq = -1;
  mission_trans_start_time = 0;
  mission_trans_end_time = 0;
  //mav_port.resume();
  if (connection_type == 0) {
    if (mav_port.isOpen()) {
      mav_port.close();
    }
  }
  mav_port.open(open_serial);
  /*mav_port.on('open', open_serial);*/
  mav_port.on('error', show_error_callback);
  mav_port.on('close', serial_closed_callback);
}

function pause_serial() {
  clearInterval(heartbeat_timer);
  clearInterval(datastream_timer);
  clearInterval(update_heartbeat_timer);
  //mav_port.pause();
  mav_port.close();
}

function start_serial() {
  /*connect_serial();*/
  heartbeat_timer = setInterval(send_heartbeat, 1000);
  datastream_timer = setInterval(send_default_datastream, send_default_stream_period);
  update_heartbeat_timer = setInterval(update_heartbeat, 500);
  last_heartbeat = 0;
  connection_lost = true;
  connection_losstime = 0;
  current_mission_seq = 0;
  mission_list = [];
  mission_len = 0;
  mission_trans_state = MISSION_TRANS_STATE_IDLE;
  current_trans_seq = -1;
  mission_trans_start_time = 0;
  mission_trans_end_time = 0;
  if (connection_type == 0) {
    if (mav_port.isOpen()) {
      mav_port.close();
    }
  }
  mav_port.open(open_serial);
  /*mav_port.on('open', open_serial);*/
  mav_port.on('error', show_error_callback);
  mav_port.on('close', serial_closed_callback);
}

function stop_serial() {
  /*close_serial();*/
  connection_destroy();
}

function check_serial() {
  if (mav_port == null) {
    return false;
  }
  if (connection_type == 1) {
    return true;
  }
  return mav_port.isOpen();
}


// mission related
function mission_request_individual(seq) {
  // get mission at seq on UAV
  mav_parser.createMessage("MISSION_REQUEST", {
    'seq': seq,
    'target_system': system_ID,
    'target_component': 1
  },
    function (message) {
      port_send_message(message);
      return;
    });
}

function mission_request_list() {
  // get mission count of UAV
  mission_len = -1;
  mav_parser.createMessage("MISSION_REQUEST_LIST", {
    'target_system': system_ID,
    'target_component': 1
  },
    function (message) {
      port_send_message(message);
      return;
    });
  setTimeout(mission_request_list_check, 30000);
}

function mission_request_list_check() {
  if (mission_len < 0) {
    emitter.emit('status_text', 'ERROR: Mission request list get count not legal');
    mission_len = 0;
  }
}

function mission_clear_all() {
  mav_parser.createMessage("MISSION_CLEAR_ALL", {
    'target_system': system_ID,
    'target_component': 1
  },
    function (message) {
      port_send_message(message);
      return;
    });
}

function mission_set_current(seq) {
  mav_parser.createMessage("MISSION_SET_CURRENT", {
    'seq': seq,
    'target_system': system_ID,
    'target_component': 1
  },
    function (message) {
      port_send_message(message);
      return;
    });
}

function get_current_mission() {
  return current_mission_seq;
}

/*function mission_write_waypoint(seq, hold_time, radis, param3, yaw, lat, lon, alt){*/
function mission_write_waypoint(fields) {
  mav_parser.createMessage("MISSION_ITEM", {
    'param1': fields.param1,    //hold_time
    'param2': fields.param2,    //radis
    'param3': fields.param3,    //param3
    'param4': fields.param4,    //yaw
    'x': fields.x,               //lat
    'y': fields.y,              //lon
    'z': fields.z,               //alt
    'seq': fields.seq,
    /*'command': mavheader.MAV_CMD_NAV_WAYPOINT,*/
    'command': fields.command,
    'target_system': system_ID,
    'target_component': 1,
    'frame': 3,
    'current': 0,
    'autocontinue': 1
  },
    function (message) {
      port_send_message(message);
      return;
    });
}

function mission_write_rtl(seq, lat, lon, alt) {
  mav_parser.createMessage("MISSION_ITEM", {
    'param1': 0,
    'param2': 0,
    'param3': 0,
    'param4': 0,
    'x': lat,
    'y': lon,
    'z': alt,
    'seq': seq,
    'command': mavheader.MAV_CMD_NAV_RETURN_TO_LAUNCH,
    'target_system': system_ID,
    'target_component': 1,
    'frame': 3,
    'current': 0,
    'autocontinue': 1
  },
    function (message) {
      port_send_message(message);
      return;
    });
}

// transmiting mission at #seq
function mission_trans(seq) {
  mission = mission_list[current_trans_seq];
  /*if(mission.seq != current_mission_seq){
    return;
  }*/
  mission_write_waypoint(
    mission.seq,
    mission.hold_time,
    mission.radis,
    mission.param3,
    mission.yaw,
    mission.lat,
    mission.lon,
    mission.alt
  );
}

function init_mission_trans() {
  // starting mission transmit
  if (mission_len <= 0) {
    emitter.emit('status_text', 'ERROR: Start mission transmiting, mission length not legal');
    return;
  }
  if (mission_trans_state == MISSION_TRANS_STATE_IDLE) {
    mission_trans_state = MISSION_TRANS_STATE_TRANS;
  }
  else {
    emitter.emit('status_text', 'Are we initting transmision during a transmision procedure?');
    return;
  }

  gcsDate = new Date();
  mission_trans_start_time = gcsDate.getTime();

  mav_parser.createMessage("MISSION_COUNT", {
    'count': mission_len,
    'target_system': system_ID,
    'target_component': 1
  },
    function (message) {
      port_send_message(message);
      return;
    });
  // timeout for several seconds wait state go back to 0
  // if not, commit an error
  setTimeout(mission_ack_check, 1000 * mission_len);
}

function mission_ack_check() {
  if (mission_trans_state != MISSION_TRANS_STATE_IDLE) {
    emitter.emit('status_text', 'ERROR: Uploading mission not finish');
  }
  mission_trans_state = MISSION_TRANS_STATE_IDLE;
}

function mission_get_all() {
  emitter.emit('status_text', 'Requesting all mission');
  if (mission_len <= 0) {
    emitter.emit('status_text', 'Warning: No mission on UAV');
    emitter.emit('reading_completed', []);
    return;
  }
  //mission_request_individual(0);
  for (var i = 0; i < mission_len && i < DOWNLOADING_NUM_AT_A_TIME; i++) {
    setTimeout(function(i){
      mission_request_individual(i);
    }, 200);
  }
}

function mission_print() {
  emitter.emit('status_text', mission_list);
  for (var i = 0; i < mission_len; i++) {
    emitter.emit('status_text', JSON.stringify(mission_list[i]));
  }
}

function mission_update(fields) {
  if (fields.seq >= 0 && fields.seq < mission_len) {
    mission_list[fields.seq] = fields;
  }
  else {
    emitter.emit('status_text', 'ERROR: Mission seq not in legal range');
  }
}

function mission_update_alt(seq, alt) {
  if (seq >= 0 && seq < mission_len) {
    if (mission_list[seq].command == mavheader.MAV_CMD_NAV_WAYPOINT) {
      mission_list[seq].z = alt;
    }
    else {
      emitter.emit('status_text', 'ERROR: Reject Alt change, WAYPOINT needed');
    }
  }
  else {
    emitter.emit('status_text', 'ERROR: Mission seq not in legal range');
  }
}

// uav mission item
function MissionItem(seq, command, param1, param2, param3, param4, x, y, z) {
  this.target_system = 1;
  this.target_component = 1;
  this.seq = seq;
  this.frame = 3;
  this.command = command;
  this.current = 0;
  this.autocontinue = 1;
  this.param1 = param1;
  this.param2 = param2;
  this.param3 = param3;
  this.param4 = param4;
  this.x = x;
  this.y = y;
  this.z = z;
}
// gcs mission format
function WayPoint(lat, lng, num, yaw, altitude, spd, hld_time, do_action) {
  this.lat = lat;
  this.lng = lng;
  this.yaw = yaw;
  this.index = num;
  this.alt = altitude;
  this.speed = spd;
  this.hold_time = hld_time;
  this.action = do_action;
}

// this is our customed version of mission points
// all actions are append on their preceed mission
// these actions' seq filed is meaningless
// we just skip those seq
var saved_gcs_mission_list = [];

function mission_to_gcs() {
  var gcs_mission_list = [];
  var gcs_mission_index = 0;
  var last_speed = 0;
  maplist = [];
  for (var i = 0; i < mission_len; i++) {
    var cur_mission = mission_list[i];
    switch (cur_mission.command) {
      case mavheader.MAV_CMD_NAV_WAYPOINT:
        var gcs_mission = new WayPoint(cur_mission.x, cur_mission.y, gcs_mission_index, cur_mission.param4, cur_mission.z, 0, cur_mission.param1, 0);
        if (last_speed > 0) {
          gcs_mission.speed = last_speed
        }
        gcs_mission_list.push(gcs_mission);
        gcs_mission_index += 1;
        maplist.push(i);
        break;
      case mavheader.MAV_CMD_DO_CHANGE_SPEED:
        if (gcs_mission_index > 0) {
          // if(cur_mission.param2 > 0 && cur_mission.param2 <= 20){
          //   if(last_speed != cur_mission.param2){
          //     gcs_mission_list[gcs_mission_index-1].speed = cur_mission.param2;
          //   }
          // }
          // else{
          //   emitter.emit('status_text', "ERROR: do change speed rejected on seq"+ cur_mission.param2);
          // }
          if (cur_mission.param2 > 0 && cur_mission.param2 <= 20) {
            last_speed = cur_mission.param2;
          }
        }
        break;
      case mavheader.MAV_CMD_NAV_LAND:
        var gcs_mission = new WayPoint(cur_mission.x, cur_mission.y, gcs_mission_index, cur_mission.param4, cur_mission.z, 0, 0, 1);
        gcs_mission_list.push(gcs_mission);
        gcs_mission_index += 1;
        maplist.push(i);
        break;
      case mavheader.MAV_CMD_NAV_RETURN_TO_LAUNCH:
        if (gcs_mission_index > 0) {
          gcs_mission_list[gcs_mission_index - 1].action = 2;
        }
        break;
      case mavheader.MAV_CMD_DO_JUMP:
        emitter.emit('loop_checked');
        break;
      default:
        break;
    }
  }
  save_gcslist(gcs_mission_list);
  return gcs_mission_list;
}

var maplist = [];

function get_gcs_seq(uas_seq) {
  if (maplist == [] || maplist.length < 1) return -1;
  if (uas_seq < 0) return 0;
  var i = 0;
  while (uas_seq > maplist[i] && i < maplist.length) {
    i += 1;
  }
  return i;
}

function mission_from_gcs(gcs_mission_list, takeoff_alt, is_loop) {
  emitter.emit('status_text', "is loop" + is_loop);
  mission_len = 0;
  mission_list = [];
  var mis_seq = 0;
  var last_speed = 0;
  maplist = [];
  for (var i = 0; i < gcs_mission_list.length; i++) {
    var gcs_mission = gcs_mission_list[i];
    switch (gcs_mission.action) {
      case 0://WAYPOINT
        if (i == 1 || mission_list.length > 0 && gcs_mission.speed > 0 && gcs_mission.speed != last_speed) {
          var mission = new MissionItem(
            mis_seq++,
            mavheader.MAV_CMD_DO_CHANGE_SPEED,
            0,
            gcs_mission.speed,
            0, 0, 0, 0, 0
          );
          mission_list.push(mission);
        }
        var mission = new MissionItem(
          mis_seq++,
          mavheader.MAV_CMD_NAV_WAYPOINT,
          gcs_mission.hold_time,
          0,
          0,
          gcs_mission.yaw,
          gcs_mission.lat,
          gcs_mission.lng,
          gcs_mission.alt
        )
        mission_list.push(mission);
        maplist.push(mission_list.length - 1);
        // here to take off
        if (mission_list.length == 1) {// right after home point
          var mission = new MissionItem(
            mis_seq++,
            mavheader.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0,
            takeoff_alt
          );
          mission_list.push(mission);
        }
        last_speed = gcs_mission.speed;
        break;
      case 1://LAND
        var mission = new MissionItem(
          mis_seq++,
          mavheader.MAV_CMD_NAV_LAND,
          0,
          0,
          0,
          0,
          gcs_mission.lat,
          gcs_mission.lng,
          gcs_mission.alt
        )
        mission_list.push(mission);
        maplist.push(mission_list.length - 1);
        break;
      case 2:
        var mission = new MissionItem(
          mis_seq++,
          mavheader.MAV_CMD_NAV_WAYPOINT,
          gcs_mission.hold_time,
          0,
          0,
          gcs_mission.yaw,
          gcs_mission.lat,
          gcs_mission.lng,
          gcs_mission.alt
        )
        mission_list.push(mission);
        maplist.push(mission_list.length - 1);

        var mission = new MissionItem(
          mis_seq++,
          mavheader.MAV_CMD_NAV_RETURN_TO_LAUNCH,
          0, 0, 0, 0, 0, 0, 0
        )
        mission_list.push(mission);

        break;
      default:
        emitter.emit('status_text', 'ERROR: Unsupported gcs mission')
        break;
    }
  }
  if (is_loop) {
    var mission = new MissionItem(
      mis_seq++,
      mavheader.MAV_CMD_DO_JUMP,
      3, -1, 0, 0, 0, 0, 0
    )
    mission_list.push(mission);
  }

  if (mis_seq != mission_list.length) {
    emitter.emit('status_text', 'ERROR: mission length illeagal');
  }
  mission_len = mission_list.length;
  save_gcslist(gcs_mission_list);
}

function save_gcslist(gcs_list) {
  saved_gcs_mission_list = [];
  for (var i = 0; i < gcs_list.length; i++) {
    var cur_wp = gcs_list[i];
    var gcs_mission = new WayPoint(
      cur_wp.lat,
      cur_wp.lng,
      cur_wp.index,
      cur_wp.yaw,
      cur_wp.alt,
      cur_wp.speed,
      cur_wp.hold_time,
      cur_wp.action
    );
    saved_gcs_mission_list.push(gcs_mission);
  }
}

function get_save_gcs_misstion_list() {
  return saved_gcs_mission_list;
}

function mission_cmd_mission_start() {
  send_command_long(0, 0, 0, 0, 0, 0, 0, mavheader.MAV_CMD_MISSION_START, 1);
}

// takeoff
function mission_cmd_takeoff(takeoff_alt) {
  send_command_long(0, 0, 0, 0, 0, 0, takeoff_alt, mavheader.MAV_CMD_NAV_TAKEOFF, 1);
}

// set mode to rtl
function mission_cmd_rtl() {
  send_command_long(0, 0, 0, 0, 0, 0, 0, mavheader.MAV_CMD_NAV_RETURN_TO_LAUNCH, 1);
}

// set mode to land
function mission_cmd_land() {
  send_command_long(0, 0, 0, 0, 0, 0, 0, mavheader.MAV_CMD_NAV_LAND, 1);
}

// set mode to land
function mission_do_jump() {
  send_command_long(1, -1, 0, 0, 0, 0, 0, mavheader.MAV_CMD_DO_JUMP, 1);
}

function get_emitter() {
  return emitter;
}

function init_UAS() {
  setTimeout(connect_serial, 100);
  heartbeat_timer = setInterval(send_heartbeat, 1000);
  datastream_timer = setInterval(send_default_datastream, send_default_stream_period);
  mav_parser = null;
  update_heartbeat_timer = setInterval(update_heartbeat, 500);
  last_heartbeat = 0;
  connection_lost = true;
  connection_losstime = 0;
  current_mission_seq = 0;
  mission_list = [];
  mission_len = 0;
  mission_trans_state = MISSION_TRANS_STATE_IDLE;
  current_trans_seq = -1;
  mission_trans_start_time = 0;
  mission_trans_end_time = 0;
}

function connection_destroy() {
  /*mav_port.destroy();*/
  mav_port.close(function () {
    emitter.emit('status_text', 'Connection closed on ' + connection_path);
  });
  /*mav_port = null;*/
  /*mav_parser = null;*/
  clearInterval(heartbeat_timer);
  clearInterval(datastream_timer);
  clearInterval(update_heartbeat_timer);
}

function send_rc_override(gcs_joystick) {
  if (connection_lost) return;
  mav_parser.createMessage("RC_CHANNELS_OVERRIDE", {
    'chan1_raw': gcs_joystick.chan1,
    'chan2_raw': gcs_joystick.chan2,
    'chan3_raw': gcs_joystick.chan3,
    'chan4_raw': gcs_joystick.chan4,
    'chan5_raw': gcs_joystick.chan5,
    'chan6_raw': gcs_joystick.chan6,
    'chan7_raw': gcs_joystick.chan7,
    'chan8_raw': gcs_joystick.chan8,
    'target_system': system_ID,
    'target_component': 1
  },
    function (message) {
      port_send_message(message);
      return;
    });
}

function send_offboard_control(joystick) {
  if (connection_lost) return;
  mav_parser.createMessage("OFFBOARD_CONTROL", {
    'chan1_raw': joystick.chan1,
    'chan2_raw': joystick.chan2,
    'chan3_raw': joystick.chan3,
    'chan4_raw': joystick.chan4,
    'chan5_raw': joystick.chan5,
    'chan6_raw': joystick.chan6,
    'chan7_raw': joystick.chan7,
    'chan8_raw': joystick.chan8,
    'target_system': system_ID,
    'target_component': 1
  },
    function (message) {
      port_send_message(message);
      return;
    });
}

function param_request_single(param_id) {
  mav_parser.createMessage("PARAM_REQUEST_READ", {
    'param_id': param_id,
    'param_index': -1,
    'target_system': system_ID,
    'target_component': 1
  },
    function (message) {
      port_send_message(message);
      return;
    });
}

function param_set_value(param_id, param_value, param_type) {
  mav_parser.createMessage("PARAM_SET", {
    'param_id': param_id,
    'param_value': param_value,
    'param_type': param_type,
    'target_system': system_ID,
    'target_component': 1
  },
    function (message) {
      port_send_message(message);
      return;
    });
}

function guide_set_pos(x, y, z) {
  mav_parser.createMessage("SET_POSITION_TARGET_LOCAL_NED", {
    'time_boot_ms': 0,
    'target_system': system_ID,
    'target_component': 1,
    'coordinate_frame': 9,
    'type_mask': 1 << 3 | 1 << 4 | 1 << 5 | 1 << 6 | 1 << 7 | 1 << 8,
    'x': x,
    'y': y,
    'z': z,
    'vx': 0,
    'vy': 0,
    'vz': 0,
    'afx': 0,
    'afy': 0,
    'afz': 0,
    'yaw': 0,
    'yaw_rate': 0
  },
    function (message) {
      port_send_message(message);
      return;
    });
}

function guide_set_vel(vx, vy, vz) {
  mav_parser.createMessage("SET_POSITION_TARGET_LOCAL_NED", {
    'time_boot_ms': 0,
    'target_system': system_ID,
    'target_component': 1,
    'coordinate_frame': 9,
    'type_mask': 1 << 0 | 1 << 1 | 1 << 2 | 1 << 6 | 1 << 7 | 1 << 8,
    'x': 0,
    'y': 0,
    'z': 0,
    'vx': vx,
    'vy': vy,
    'vz': vz,
    'afx': 0,
    'afy': 0,
    'afz': 0,
    'yaw': 0,
    'yaw_rate': 0
  },
    function (message) {
      port_send_message(message);
      return;
    });
}

function set_servo(id, pwm) {
  send_command_long(id, pwm, 0, 0, 0, 0, 0, mavheader.MAV_CMD_DO_SET_SERVO, 1);
}

function print_msg_hex(data) {
  var hex_string = '';
  for (var i = 0; i < data.length; i++) {
    hex_string = hex_string + data[i].toString(16) + ' ';
  }
  return hex_string;
}

module.exports = {
  get_attitude: get_attitude,
  get_base_status: get_base_status,
  get_mode: get_mode,
  get_mode_str: get_mode_str,
  set_connection: set_connection,
  start_serial: start_serial,
  stop_serial: stop_serial,
  check_serial: check_serial,
  set_custom_mode: set_custom_mode,
  send_banner: send_banner,
  send_version: send_version,
  set_arming: set_arming,
  set_disarming: set_disarming,
  mission_request_individual: mission_request_individual,
  mission_request_list: mission_request_list,
  mission_clear_all: mission_clear_all,
  mission_set_current: mission_set_current,
  get_current_mission: get_current_mission,
  mission_write_waypoint: mission_write_waypoint,
  mission_write_rtl: mission_write_rtl,
  init_mission_trans: init_mission_trans,
  mission_get_all: mission_get_all,
  mission_print: mission_print,
  mission_update: mission_update,
  mission_update_alt: mission_update_alt,
  mission_cmd_mission_start: mission_cmd_mission_start,
  mission_cmd_takeoff: mission_cmd_takeoff,
  mission_cmd_rtl: mission_cmd_rtl,
  mission_cmd_land: mission_cmd_land,
  get_emitter: get_emitter,
  get_gps_raw_int: get_gps_raw_int,
  get_global_pos: get_global_pos,
  get_port_list: get_port_list,
  pause_serial: pause_serial,
  resume_serial: resume_serial,
  send_rc_override: send_rc_override,
  param_set_value: param_set_value,
  param_request_single: param_request_single,
  set_datastream_rate: set_datastream_rate,
  send_default_datastream: send_default_datastream,
  set_mag_cal_stream: set_mag_cal_stream,
  get_save_gcs_misstion_list: get_save_gcs_misstion_list,
  heartbeat_pause: heartbeat_pause,
  heartbeat_resume: heartbeat_resume,
  set_default_stream_rates: set_default_stream_rates,
  set_manual_stream_rates: set_manual_stream_rates,
  set_auto_stream_rates: set_auto_stream_rates,
  guide_set_pos: guide_set_pos,
  guide_set_vel: guide_set_vel,
  set_servo: set_servo,
  mission_do_jump: mission_do_jump,
  port_send_customed_control: port_send_customed_control,
  port_send_switch_control:port_send_switch_control,
  send_offboard_control:send_offboard_control,
  mission_from_gcs: mission_from_gcs
}
