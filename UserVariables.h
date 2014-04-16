/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES
//////////////////////////////////
// static variables and defines //
//////////////////////////////////
// These were originally in ArduFire but I moved them here because of scope
#define MOTOR_CLASS AP_MotorsQuad

static MOTOR_CLASS motors(&g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4);
static AP_BattMonitor battery;

//////////////// STARTS
////////////////////////////////////////////////////////////////////////////////
// GPS variables
////////////////////////////////////////////////////////////////////////////////
// This is used to scale GPS values for EEPROM storage
// 10^7 times Decimal GPS means  1 == 1cm
// This approximation makes calculations integer and it's easy to read
static const float t7 = 10000000.0;
// We use atan2 and other trig techniques to calaculate angles
// We need to scale the longitude up to make these calcs work
// to account for decreasing distance between lines of longitude away from the equator
static float scaleLongUp = 1;
// Sometimes we need to remove the scaling for distance calcs
static float scaleLongDown = 1;

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// This is the angle from the copter to the next waypoint in centi-degrees
static int32_t wp_bearing;
// The original bearing to the next waypoint.  used to point the nose of the copter at the next waypoint
static int32_t original_wp_bearing;
// The location of home in relation to the copter in centi-degrees
static int32_t home_bearing;
// distance between plane and home in cm
static int32_t home_distance;
// distance between plane and next waypoint in cm.
static uint32_t wp_distance;
// navigation mode - options include NAV_NONE, NAV_LOITER, NAV_CIRCLE, NAV_WP
static uint8_t nav_mode;
// Register containing the index of the current navigation command in the mission script
static int16_t command_nav_index;
// Register containing the index of the previous navigation command in the mission script
// Used to manage the execution of conditional commands
static uint8_t prev_nav_index;
// Register containing the index of the current conditional command in the mission script
static uint8_t command_cond_index;
// Used to track the required WP navigation information
// options include
// NAV_ALTITUDE - have we reached the desired altitude?
// NAV_LOCATION - have we reached the desired location?
// NAV_DELAY    - have we waited at the waypoint the desired time?
static float lon_error, lat_error;      // Used to report how many cm we are from the next waypoint or loiter target position
static int16_t control_roll;            // desired roll angle of copter in centi-degrees
static int16_t control_pitch;           // desired pitch angle of copter in centi-degrees
static uint8_t rtl_state;               // records state of rtl (initial climb, returning home, etc)
static uint8_t land_state;              // records state of land (flying to location, descending)

////////////////////////////////////////////////////////////////////////////////
// Orientation
////////////////////////////////////////////////////////////////////////////////
// Convienience accessors for commonly used trig functions. These values are generated
// by the DCM through a few simple equations. They are used throughout the code where cos and sin
// would normally be used.
// The cos values are defaulted to 1 to get a decent initial value for a level state
static float cos_roll_x         = 1.0;
static float cos_pitch_x        = 1.0;
static float cos_yaw            = 1.0;
static float sin_yaw;
static float sin_roll;
static float sin_pitch;

////////////////////////////////////////////////////////////////////////////////
// SIMPLE Mode
////////////////////////////////////////////////////////////////////////////////
// Used to track the orientation of the copter for Simple mode. This value is reset at each arming
// or in SuperSimple mode when the copter leaves a 20m radius from home.
static float simple_cos_yaw = 1.0;
static float simple_sin_yaw;
static int32_t super_simple_last_bearing;
static float super_simple_cos_yaw = 1.0;
static float super_simple_sin_yaw;


// Stores initial bearing when armed - initial simple bearing is modified in super simple mode so not suitable
static int32_t initial_armed_bearing;


////////////////////////////////////////////////////////////////////////////////
// Rate contoller targets
////////////////////////////////////////////////////////////////////////////////
// Moved to user space

////////////////////////////////////////////////////////////////////////////////
// Throttle variables
////////////////////////////////////////////////////////////////////////////////
static int16_t throttle_accel_target_ef;    // earth frame throttle acceleration target
static bool throttle_accel_controller_active;   // true when accel based throttle controller is active, false when higher level throttle controllers are providing throttle output directly
static float throttle_avg;                  // g.throttle_cruise as a float
static int16_t desired_climb_rate;          // pilot desired climb rate - for logging purposes only
static float target_alt_for_reporting;      // target altitude in cm for reporting (logs and ground station)


////////////////////////////////////////////////////////////////////////////////
// ACRO Mode
////////////////////////////////////////////////////////////////////////////////
// Used to control Axis lock
static int32_t acro_roll;                   // desired roll angle while sport mode
static int32_t acro_roll_rate;              // desired roll rate while in acro mode
static int32_t acro_pitch;                  // desired pitch angle while sport mode
static int32_t acro_pitch_rate;             // desired pitch rate while acro mode
static int32_t acro_yaw_rate;               // desired yaw rate while acro mode
static float acro_level_mix;                // scales back roll, pitch and yaw inversely proportional to input from pilot

////////////////////////////////////////////////////////////////////////////////
// Circle Mode / Loiter control
////////////////////////////////////////////////////////////////////////////////
Vector3f circle_center;     // circle position expressed in cm from home location.  x = lat, y = lon
// angle from the circle center to the copter's desired location.  Incremented at circle_rate / second
static float circle_angle;
// the total angle (in radians) travelled
static float circle_angle_total;
// deg : how many times to circle as specified by mission command
static uint8_t circle_desired_rotations;
static float circle_angular_acceleration;       // circle mode's angular acceleration
static float circle_angular_velocity;           // circle mode's angular velocity
static float circle_angular_velocity_max;       // circle mode's max angular velocity
// How long we should stay in Loiter Mode for mission scripting (time in seconds)
static uint16_t loiter_time_max;
// How long have we been loitering - The start time in millis
static uint32_t loiter_time;


////////////////////////////////////////////////////////////////////////////////
// CH7 and CH8 save waypoint control
////////////////////////////////////////////////////////////////////////////////
// This register tracks the current Mission Command index when writing
// a mission using Ch7 or Ch8 aux switches in flight
static int8_t aux_switch_wp_index;


////////////////////////////////////////////////////////////////////////////////
// Battery Sensors
////////////////////////////////////////////////////////////////////////////////
// Moved to user Variables



////////////////////////////////////////////////////////////////////////////////
// Altitude
////////////////////////////////////////////////////////////////////////////////
// The (throttle) controller desired altitude in cm
static float controller_desired_alt;
// The cm we are off in altitude from next_WP.alt – Positive value means we are below the WP
static int32_t altitude_error;
// The cm/s we are moving up or down based on filtered data - Positive = UP
static int16_t climb_rate;
// The altitude as reported by Sonar in cm – Values are 20 to 700 generally.
static int16_t sonar_alt;
static uint8_t sonar_alt_health;   // true if we can trust the altitude from the sonar
static float target_sonar_alt;      // desired altitude in cm above the ground
// The altitude as reported by Baro in cm – Values can be quite high
static int32_t baro_alt;


////////////////////////////////////////////////////////////////////////////////
// flight modes
////////////////////////////////////////////////////////////////////////////////
// Flight modes are combinations of Roll/Pitch, Yaw and Throttle control modes
// Each Flight mode is a unique combination of these modes
//
// The current desired control scheme for Yaw
static uint8_t yaw_mode = STABILIZE_YAW;
// The current desired control scheme for roll and pitch / navigation
static uint8_t roll_pitch_mode = STABILIZE_RP;
// The current desired control scheme for altitude hold
static uint8_t throttle_mode = STABILIZE_THR;


////////////////////////////////////////////////////////////////////////////////
// flight specific
////////////////////////////////////////////////////////////////////////////////
// An additional throttle added to keep the copter at the same altitude when banking
static int16_t angle_boost;
// counter to verify landings
static uint16_t land_detector;


////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
////////////////////////////////////////////////////////////////////////////////
// home location is stored when we have a good GPS lock and arm the copter
// Can be reset each the copter is re-armed
static struct   Location home;
// Current location of the copter
static struct   Location current_loc;
// Holds the current loaded command from the EEPROM for navigation
static struct   Location command_nav_queue;
// Holds the current loaded command from the EEPROM for conditional scripts
static struct   Location command_cond_queue;


////////////////////////////////////////////////////////////////////////////////
// Navigation Roll/Pitch functions
////////////////////////////////////////////////////////////////////////////////
// The Commanded ROll from the autopilot based on optical flow sensor.
static int32_t of_roll;
// The Commanded pitch from the autopilot based on optical flow sensor. negative Pitch means go forward.
static int32_t of_pitch;


////////////////////////////////////////////////////////////////////////////////
// Navigation Throttle control
////////////////////////////////////////////////////////////////////////////////
// The Commanded Throttle from the autopilot.
static int16_t nav_throttle;    // 0-1000 for throttle control
// This is a simple counter to track the amount of throttle used during flight
// This could be useful later in determining and debuging current usage and predicting battery life
static uint32_t throttle_integrator;


////////////////////////////////////////////////////////////////////////////////
// Navigation Yaw control
////////////////////////////////////////////////////////////////////////////////
// The Commanded Yaw from the autopilot.
static int32_t control_yaw;
// Yaw will point at this location if yaw_mode is set to YAW_LOOK_AT_LOCATION
static Vector3f yaw_look_at_WP;
// bearing from current location to the yaw_look_at_WP
static int32_t yaw_look_at_WP_bearing;
// yaw used for YAW_LOOK_AT_HEADING yaw_mode
static int32_t yaw_look_at_heading;
// Deg/s we should turn
static int16_t yaw_look_at_heading_slew;



////////////////////////////////////////////////////////////////////////////////
// Delay Mission Scripting Command
////////////////////////////////////////////////////////////////////////////////
static int32_t condition_value;  // used in condition commands (eg delay, change alt, etc.)
static uint32_t condition_start;


////////////////////////////////////////////////////////////////////////////////
// IMU variables
////////////////////////////////////////////////////////////////////////////////
// Integration time (in seconds) for the gyros (DCM algorithm)
// Updated with the fast loop
static float G_Dt = 0.02;

////////////////////////////////////////////////////////////////////////////////
// Inertial Navigation
////////////////////////////////////////////////////////////////////////////////
static AP_InertialNav inertial_nav(&ahrs, &barometer, g_gps, gps_glitch);

////////////////////////////////////////////////////////////////////////////////
// Waypoint navigation object
// To-Do: move inertial nav up or other navigation variables down here
////////////////////////////////////////////////////////////////////////////////
static AC_WPNav wp_nav(&inertial_nav, &ahrs, &g.pi_loiter_lat, &g.pi_loiter_lon, &g.pid_loiter_rate_lat, &g.pid_loiter_rate_lon);
//////////////////////////////////// ENDS


static uint8_t rate_targets_frame = EARTH_FRAME;    // indicates whether rate targets provided in earth or body frame
static int32_t roll_rate_target_ef;
static int32_t pitch_rate_target_ef;
static int32_t yaw_rate_target_ef;
static int32_t roll_rate_target_bf;     // body frame roll rate target
static int32_t pitch_rate_target_bf;    // body frame pitch rate target
static int32_t yaw_rate_target_bf;      // body frame yaw rate target


// LED PINS
#define AN5 59 // Fly mode. One means user
#define AN6 60 // armed
#define AN7 61 // BBB communication
#define AN8 62
#define OUTPUT GPIO_OUTPUT
#define INPUT GPIO_INPUT
#define HIGH 1
#define LOW 0

#define THROTTLEMAX 500
#define PITCHMAX 50
#define ROLLMAX 50
#define YAWMAX 50



//////////////////////////
// Small Util functions //
//////////////////////////
int isNumber(char s){
  if(s >= '0' && s <= '9') return 1;
  else return 0;
}

int isValid(char data){
  if((data >= 'a' && data <= 'z') || (data >= '0' && data <= '9')) return 1;
  else return 0;
} 

//////////////
// Fly Mode //
//////////////
// Fly mode, enum, global variable and update
enum FlyMode {
    auto_mode = 0,
    user_mode = 1
};

FlyMode flymode = user_mode; // default, global variable

void update_mode(){
  if(g.rc_5.radio_in > 1400) flymode = user_mode;
  else flymode = auto_mode;

  if(flymode == user_mode){
    hal.gpio->write(AN5,HIGH);
    failsafe_disable();
    set_throttle_mode(AUTO_THR);
    set_auto_armed(true);
  } else {
    hal.gpio->write(AN5,LOW);
    failsafe_enable();
    set_throttle_mode(STABILIZE_THR);
    set_auto_armed(false);
  }

  /*
  // auto_mode can only be set when the motors are disarmed
  if(motors.armed())
    if(flymode == user_mode) return;
    else if(g.rc_5.radio_in > 1400) flymode = user_mode;
  else
    if(g.rc_5.radio_in > 1400) flymode = user_mode;
    else flymode = auto_mode;*/
}

//////////////////////////
// uartB communications //
//////////////////////////

//* Read info from BBB */
static struct {
  int armMotors;
  int pitch;
  int yaw;
  int roll;
  int throttle;
  int targetHeight;
  int powerOff;
} receivedCommands;

void prepareUartB(){
  hal.uartB->begin(9600);
  receivedCommands.armMotors = 0;
  receivedCommands.pitch = 0;
  receivedCommands.yaw = 0;
  receivedCommands.throttle = 0;
  receivedCommands.powerOff = 0;
  receivedCommands.targetHeight = 0;
}

// Process the command in a switch statement. Returns 0 if processing fails 
int processCommand(char * command){
  char key = command[0];
  int value = (command[1]-'0') * 1000 + (command[2]-'0') * 100 + (command[3]-'0') * 10 + (command[4]-'0') * 1;
  
  switch(key){
    case 'a':
    case 'A':
      //hal.uartB->printf("Changing armMotors to %d\n", value);
      receivedCommands.armMotors = value;
      if(flymode == auto_mode) {
        if(value != 0) { 
          receivedCommands.pitch = 0;
          receivedCommands.yaw = 0;
          receivedCommands.throttle = 0;
          receivedCommands.targetHeight = 0;
          receivedCommands.powerOff = 0;
          init_arm_motors();  
          set_land_complete(false);
          wp_nav.set_destination(Vector3f(0,0,40));
        } else { init_disarm_motors(); reset_land_detector(); }
      }
      return 1;
      break;
    case 'h':
    case 'H':
      receivedCommands.targetHeight = value;
      wp_nav.set_destination(Vector3f(0,0,value));
      break;
    case 'p':
    case 'P':
      //hal.uartB->printf("Changing Pitch to %d\n", value);
      if(value > PITCHMAX || value < -PITCHMAX) value = 0;
      receivedCommands.pitch = value;
      return 1;
      break;
    case 'r':
    case 'R':
      //hal.uartB->printf("Changing Roll to %d\n", value);
      if(value > ROLLMAX || value < -ROLLMAX) value = 0;
      receivedCommands.roll = value;
      return 1;
      break;
    case 'y':
    case 'Y':
      //hal.uartB->printf("Changing Yaw to %d\n", value);
      if(value > YAWMAX || value < -YAWMAX) value = 0;
      receivedCommands.yaw = value;
      return 1;
      break;
    case 't':
    case 'T': 
      //hal.uartB->printf("Changing Throttle to %d\n", value);
      if(value > THROTTLEMAX) value = 0;
      receivedCommands.throttle = value;
      return 1;
      break;
    case 'z':
    case 'Z': 
      //hal.uartB->printf("Changing PowerOff to %d\n", value);
      // TODO
      receivedCommands.powerOff = value;
      if(value != 0 && flymode == auto_mode) { 
        receivedCommands.pitch = 0;
        receivedCommands.yaw = 0;
        receivedCommands.throttle = 0;
        receivedCommands.armMotors = 0;
        init_disarm_motors(); 
      } 
      return 1;
      break;
    default:
      hal.uartB->printf("Unknown command %c %d\n", key, value);
      return 0;
      break;
  }
  return 1;
}

/* Receives a message in the format S03 T1000 Y0000 R1234 */
#define WHILELOOPFAILSAFE_MAX 10000 // error check stuff
void receiveMessage(void){
  char start[3];
  char command[5];
  uint8_t data;
  int num_commands = 0;
  int whileLoopFailsafe = 0;
  //hal.uartB->flush();
  for(int ii = 0; ii < 5; ii++){
    command[ii] = 'a';
  }
  //hal.uartB->printf("Message Received \n");
  //hal.gpio->write(AN7,HIGH);
  for(int ii = 0; ii < 3; ii++){
    data = hal.uartB->read();
    while(!isValid( (char) data) && (whileLoopFailsafe++) <= WHILELOOPFAILSAFE_MAX){ data = hal.uartB->read(); } 
    start[ii] = (char) data;
  }
  
  if(start[0] == 's'){
    if(isNumber(start[1])) num_commands = (start[1] - '0') * 10;
    else { return; }
    if(isNumber(start[2])) num_commands += (start[2] - '0'); 
    else { return; }
  } else { return; }
  
/*  if(start[0] == 's'){
    if(isNumber(start[1])) num_commands = (start[1] - '0') * 10;
    else { hal.uartB->printf("Error in first command [1] \n"); return; }
    if(isNumber(start[2])) num_commands += (start[2] - '0'); 
    else { hal.uartB->printf("Error in first command [2] \n"); return; }
    //hal.uartB->printf("Ready to receive %d commands. \n", num_commands);
  } else {
    hal.uartB->printf("Error in first command [0]. Got %s\n", start);
    return;
  }*/
  
  // Receve the rest of the commands
  while((num_commands--) && (whileLoopFailsafe++) <= WHILELOOPFAILSAFE_MAX){
    while(hal.uartB->available() == 0 && (whileLoopFailsafe++) <= WHILELOOPFAILSAFE_MAX) {} // wait for new data. TODO - add exit strategy to avoid infinit loops
    for(int ii = 0; ii < 5; ii++){
      data = hal.uartB->read();
      while(!isValid( (char) data) && (whileLoopFailsafe++) <= WHILELOOPFAILSAFE_MAX) { data = hal.uartB->read(); }
        command[ii] = (char) data;
    } 
    //hal.uartB->printf("Command %d: %s \n", num_commands, command);
    //hal.uartA->printf("Command %d: %s \n", num_commands, command);
    if(processCommand(command) == 0) return; //hal.uartB->printf("Error parsing command!\n");
  }
}

void sendMessageReply(void){
//  float b_voltage = battery.voltage();
//  float b_current = battery.current_amps();
//  float b_current_mah = battery.current_total_mah();
//  hal.uartB->printf("bv%f,bc%f,bm%f,\n", b_voltage, b_current, b_current_mah); 
//  Vector3f gyro = ins.get_gyro();  
//  hal.uartB->printf("gx%f,gy%f,gz%f,\n",gyro.x, gyro.y, gyro.z);
//  Vector3f accel = ins.get_accel();  
//  hal.uartB->printf("ax%f,ay%f,az%f,\n",accel.x, accel.y, accel.z);
//  const Vector3f &mag_offsets = compass.get_offsets();  
//   hal.uartB->printf("cox%f,coy%f,coz%f\n",mag_offsets.x, mag_offsets.y, mag_offsets.z);
//  const Vector3f &compass_field = compass.get_field(); 
//  hal.uartB->printf("cx%f,cy%f,cz%f,\n", compass_field.x, compass_field.y, compass_field.z);
//  float yaw_target_body = yaw_rate_target_bf;
//  float yaw_target_earth = yaw_rate_target_ef;
//  hal.uartB->printf("yb%f,ye%f\n", yaw_target_body, yaw_target_earth);
//  hal.uartB->printf("mo%d,ar%d,\n",flymode,motors.armed());
    hal.uartB->printf("Target Height %d\n",receivedCommands.targetHeight);
}
  
int send_next_status = 0;
Vector3f temp_vec3;
void sendMessageStatus(void){
  switch(send_next_status++ %5) {
    case 0:
      hal.uartB->printf("mo%d,ar%d,\n",flymode,motors.armed());
    break;
    case 1:
      hal.uartB->printf("bv%f,bc%f,bm%f,\n", battery.voltage(), battery.current_amps(), battery.current_total_mah()); 
    break;
    case 2:
      temp_vec3 = ins.get_gyro();  
      hal.uartB->printf("gx%f,gy%f,gz%f,\n",temp_vec3.x, temp_vec3.y, temp_vec3.z);
    break;
    case 3:
      temp_vec3 = ins.get_accel();  
      hal.uartB->printf("ax%f,ay%f,az%f,\n",temp_vec3.x, temp_vec3.y, temp_vec3.z);
    break;
    case 4:
      float yaw_target_body = yaw_rate_target_bf;
      float yaw_target_earth = yaw_rate_target_ef;
      hal.uartB->printf("yb%f,ye%f\n", yaw_target_body, yaw_target_earth);
    break;
  }
}
// Used for testing
void printStatustoUart(void){   
  //hal.uartB->printf("Status: Armed %d, Pitch %d, Yaw %d, Roll %d, Throttle %d, PowerOff %d\n", receivedCommands.armMotors, receivedCommands.pitch, receivedCommands.yaw, receivedCommands.roll,   receivedCommands.throttle, receivedCommands.powerOff);
  hal.uartA->printf("Status: Armed %d, Pitch %d, Yaw %d, Roll %d, Throttle %d, PowerOff %d\n", receivedCommands.armMotors, receivedCommands.pitch, receivedCommands.yaw, receivedCommands.roll,   receivedCommands.throttle, receivedCommands.powerOff);
}


int flush_count = 0;
void sync_uart(void){
  int num = hal.uartB->available();
  if(num > 0) {
    if(hal.uartB->read() == 's') {
      //hal.uartA->printf("Got something!");
      hal.gpio->write(AN7,HIGH); 
      receiveMessage();
      sendMessageReply();
      //printStatustoUart();
    } 
  } else {
    //if(flush_count % 30) { hal.uartB->flush(); flush_count++; }
    hal.uartB->flush();
    hal.gpio->write(AN7,LOW);
  }
}

//////////
// MISC //
//////////
void arm_LED(){
  if(motors.armed()) hal.gpio->write(AN6, HIGH);  // Change to motors.armed()
  else hal.gpio->write(AN6, LOW);
}

#endif  // USERHOOK_VARIABLES


