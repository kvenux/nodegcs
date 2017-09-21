var colors = require('colors');
// var argv = require('optimist')
// .usage('nodejs based Ground Control Station\nUsage: $0')
// .alias('t', 'type')
// .describe('t', 'Specity the type of expected connection: serial or tcp, default is serial')
// .default('t', 'serial')
// .alias('p', 'path')
// .describe('p', 'Specify the path of serial or tcp connection, e.g. /dev/cu.usbmodem1 or 127.0.0.1:5760')
// .demand('path')
// .alias('b', 'baud')
// .describe('b', 'Specify baudrate of that serial port, default is 57600')
// .default('b', 57600)
// .argv
// ;

var uas = require('./mavlink_connection.js');
var emitter = uas.get_emitter();

/*console.log(colors.rainbow(argv.path));
*black
red
green
yellow
blue
magenta
cyan
white
gray
grey
* */
var stdin = process.openStdin();

function custom_prompt(){
    process.stdout.write('➜ nodegcs '.cyan.bold);
    if(uas.get_mode_str() != undefined){
      process.stdout.write(uas.get_mode_str().magenta);
      process.stdout.write(' '.cyan);
    }
}

var connected = false;
var mode_str = 'STABILIZE';
var arm_status = 'Disarm'
var sat_num = 0;
var HDOP = 0;
var VDOP = 0;
var pitch = 0.0;
var roll = 0.0;
var yaw = 0.0;
var lat = 0.0;
var lng = 0.0;
var vx = 0.0;
var vy = 0.0;
var alt = 0.0;
var curwp = 0;
var bat = 0.0;

function refresh_cmdline() {
  process.stdout.write('\033c');
  if(connected){
    process.stdout.write('Connected\t'.green.bold);
  }
  else{
    process.stdout.write('Disconnected\t'.red.bold);
  }
  process.stdout.write(uas.get_mode_str().magenta);
  process.stdout.write('\t'.cyan);
  if(arm_status){
    process.stdout.write('ARM\t'.yellow);
  }
  else{
    process.stdout.write('DISARM\t'.blue);
  }
  var tmp_str = 'Sat'+sat_num+'\t';
  process.stdout.write(tmp_str.cyan);
  tmp_str = 'HDOP:'+HDOP+'\t';
  process.stdout.write(tmp_str.cyan);
  tmp_str = 'VDOP:'+VDOP+'\t';
  process.stdout.write(tmp_str.cyan);
  process.stdout.write('\n'.cyan);

  tmp_str = 'Pitch:'+pitch+'\t';
  process.stdout.write(tmp_str.cyan);
  tmp_str = 'Roll:'+roll+'\t';
  process.stdout.write(tmp_str.cyan);
  tmp_str = 'Yaw:'+yaw+'\t';
  process.stdout.write(tmp_str.cyan);
  tmp_str = 'Lat:'+lat+'\t';
  process.stdout.write(tmp_str.cyan);
  tmp_str = 'Lng:'+lng+'\t';
  process.stdout.write(tmp_str.cyan);
  process.stdout.write('\n'.cyan);

  tmp_str = 'X-Speed:'+vx+'\t';
  process.stdout.write(tmp_str.cyan);
  tmp_str = 'Y-Speed:'+vy+'\t';
  process.stdout.write(tmp_str.cyan);
  tmp_str = 'Height:'+alt+'\t';
  process.stdout.write(tmp_str.cyan);
  tmp_str = 'Current-Waypoint:'+curwp+'\t';
  process.stdout.write(tmp_str.cyan);

  process.stdout.write('➜ nodegcs '.cyan.bold);
}

function is_tcp_connection(path){
  return path.indexOf(':') > -1;
}

function start_connection(){
  if(is_tcp_connection(port_path)){
    var ip_addr = port_path.split(':')[0];
    var ip_port = port_path.split(':')[1];
    uas.set_connection(1, ip_addr, parseInt(ip_port));
  }
  else if(!uas.check_serial()){
    console.log('Build new')
    uas.set_connection(0, port_path, 57600);
  }
  else{
    uas.resume_serial();
  }
  uas.set_default_stream_rates(1,3,3,1,3,3,0,0); 
  uas.set_custom_mode(6);
}

var connected = false;
// start_connection();
// custom_prompt();
refresh_cmdline();

stdin.on('data', function(d) {
  var cmd_str = d.toString().trim();
  /*console.log(colors.blue(cmd_str));*/
  var cmd_list = cmd_str.split(' ')
  switch(cmd_list[0]){
    case 'start':
      uas.set_connection(1, '127.0.0.1', 5760);
    break;
    case 'stop':
      uas.pause_serial();
    break;
    case 'check':
      // console.log(uas.check_serial());
      process.stdout.write('\033c');
    break;
    case 'setmode':
      uas.set_custom_mode(cmd_list[1]);
    break;
    case 'getatt':
      console.log(uas.get_attitude());
    break;
    case 'getmode':
      console.log(uas.get_mode_str());
    break;
    case 'getbs':
      console.log(uas.get_base_status());
    break;
    case 'banner':
      uas.send_banner();
    break;
    case 'version':
      uas.send_version();
    break;
    case 'arm':
      uas.set_arming();
    break;
    case 'disarm':
      uas.set_disarming();
    break;
    case 'misat':
      uas.mission_request_individual(cmd_list[1]);
    break;
    case 'miscnt':
      uas.mission_request_list();
    break;
    case 'setcur':
      uas.mission_set_current(cmd_list[1]);
    break;
    case 'getcur':
      console.log(uas.get_current_mission());
    break;
    case 'writewp':
      as.mission_write_waypoint(cmd_list[1], cmd_list[2], cmd_list[3], cmd_list[4], cmd_list[5], cmd_list[6], cmd_list[7], cmd_list[8]);
    break;
    case 'writertl':
      uas.mission_write_rtl(cmd_list[1], cmd_list[2], cmd_list[3], cmd_list[4]);
    break;
    case 'upmis':
      uas.init_mission_trans();
    break;
    case 'misall':
      uas.mission_get_all();
    break;
    case 'misprint':
      uas.mission_print();
    break;
    case 'misupalt':
      /*var fields = JSON.parse(cmd_list[1]);*/
      uas.mission_update_alt(cmd_list[1], cmd_list[2]);
    break;
    case 'misstart':
      uas.mission_cmd_mission_start();
    break;
    case 'takeoff':
      uas.mission_cmd_takeoff(cmd_list[1]);
    break;
    case 'rtl':
      uas.mission_cmd_rtl();
    break;
    case 'land':
      uas.mission_cmd_land();
    break;
    case 'getp':
      uas.param_request_single(cmd_list[1]);
    break;
    case 'setp':
      uas.param_set_value(cmd_list[1], cmd_list[2], cmd_list[3]);
    break;
    case '':
      /*console.log(uas.get_base_status());*/
    break;
    default:
      console.log('command not supported');
  }
  custom_prompt();
});

emitter.on('connection_regained', function(){
  process.stdout.write('connection regained\n'.yellow);
  custom_prompt();
});

emitter.on('command_ack', function(field){
  // custom_prompt();
  if(field.result == 0){
    process.stdout.clearLine();  // clear current text
    process.stdout.cursorTo(0);  // move cursor to beginning of line
    process.stdout.write('command success'.yellow);
  }
});