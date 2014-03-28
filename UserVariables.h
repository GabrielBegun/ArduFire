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

// LED PINS
#define AN5 59 // Fly mode. On means user
#define AN6 60 // armed
#define AN7 61 // BBB communication
#define AN8 62
#define OUTPUT GPIO_OUTPUT
#define INPUT GPIO_INPUT
#define HIGH 1
#define LOW 0

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
  if(flymode == user_mode){
    hal.gpio->write(AN5,HIGH);
  } else {
    hal.gpio->write(AN5,LOW);
  }
  if(g.rc_5.radio_in > 1400) flymode = user_mode;
  else flymode = auto_mode;
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
  int powerOff;
} receivedCommands;

void prepareUartB(){
  hal.uartB->begin(9600);
  receivedCommands.armMotors = 0;
  receivedCommands.pitch = 0;
  receivedCommands.yaw = 0;
  receivedCommands.throttle = 0;
  receivedCommands.powerOff = 0;
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
      if(value != 0 && flymode == auto_mode) { init_arm_motors(); } 
      else { init_disarm_motors(); }
      return 1;
      break;
    case 'p':
    case 'P':
      //hal.uartB->printf("Changing Pitch to %d\n", value);
      receivedCommands.pitch = value;
      return 1;
      break;
    case 'r':
    case 'R':
      //hal.uartB->printf("Changing Roll to %d\n", value);
      receivedCommands.roll = value;
      return 1;
      break;
    case 'y':
    case 'Y':
      //hal.uartB->printf("Changing Yaw to %d\n", value);
      receivedCommands.yaw = value;
      return 1;
      break;
    case 't':
    case 'T': 
      //hal.uartB->printf("Changing Throttle to %d\n", value);
      receivedCommands.throttle = value;
      return 1;
      break;
    case 'z':
    case 'Z': 
      //hal.uartB->printf("Changing PowerOff to %d\n", value);
      // TODO
      receivedCommands.powerOff = value;
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
void receiveMessage(void){
  char start[3];
  char command[5];
  uint8_t data;
  int num_commands = 0;
  //hal.uartB->flush();
  for(int ii = 0; ii < 5; ii++){
    command[ii] = 'a';
  }
  //hal.uartB->printf("Message Received \n");
  hal.gpio->write(AN7,HIGH);
  for(int ii = 0; ii < 3; ii++){
    data = hal.uartB->read();
    while(!isValid( (char) data)){ data = hal.uartB->read(); } 
    start[ii] = (char) data;
  }
  if(start[0] == 's'){
    if(isNumber(start[1])) num_commands = (start[1] - '0') * 10;
    else hal.uartB->printf("Error in first command [1] \n");
    if(isNumber(start[2])) num_commands += (start[2] - '0');
    else hal.uartB->printf("Error in first command [2] \n");
    hal.uartB->printf("Ready to receive %d commands. \n", num_commands);
  } else {
    hal.uartB->printf("Error in first command [0]. Got %s\n", start);
    return;
  }
  
  // Receve the rest of the commands
  while(num_commands--){
    while(hal.uartB->available() == 0) {} // wait for new data
    for(int ii = 0; ii < 5; ii++){
      data = hal.uartB->read();
      while(!isValid( (char) data)){ data = hal.uartB->read(); }
        command[ii] = (char) data;
    } 
    hal.uartB->printf("Command %d: %s \n", num_commands, command);
    if(processCommand(command) == 0) hal.uartB->printf("Error parsing command!\n");
  }
}

// TODO
void sendMessage(void){
  Vector3f gyro, accel; 
  //Vector3f &compass_field, compass_offset; // .x .y .z
  float b_voltage = battery.voltage();
  float b_current = battery.current_amps();
  float b_current_mah = battery.current_total_mah();
  gyro = ins.get_gyro();
  accel = ins.get_accel();
  //compass.read();
  const Vector3f &compass_field = compass.get_field();
  //compass_offset = compass.get_offset();
  hal.uartB->printf("sbv%fbc%fbm%f\ngx%fgy%fgz%f\nax%fay%faz%f\ncx%fxy%fcz%f\nmo%d\n", b_voltage, b_current, b_current_mah, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, compass_field.x, compass_field.y, compass_field.z,flymode);
}
  
void printStatustoUartB(void){
  hal.uartB->printf("Status: Armed %d, Pitch %d, Yaw %d, Roll %d, Throttle %d, PowerOff %d\n", receivedCommands.armMotors, receivedCommands.pitch, receivedCommands.yaw, receivedCommands.roll,   receivedCommands.throttle, receivedCommands.powerOff);
}

void sync_uart(void){
  int num = hal.uartB->available();
  if(num > 0){
    hal.gpio->write(AN7,HIGH); 
    receiveMessage();
    sendMessage();
    //printStatustoUartB(); 
  } else {
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


