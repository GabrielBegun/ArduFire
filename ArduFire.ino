/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduCopter V3.1.2-rc1"
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *  ArduCopter Version 3.0
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini 
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen, 
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to:	Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera		:Auto Compass Declination
 *  Amilcar Lucas		:Camera mount library
 *  Andrew Tridgell		:General development, Mavlink Support
 *  Angel Fernandez		:Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid		:Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel			:DCM, Libraries, Control law advice
 *  Gregory Fletcher	:Camera mount orientation math
 *  Guntars				:Arming safety suggestion
 *  HappyKillmore		:Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle			:Alpha testing
 *  James Goppert		:Mavlink Support
 *  Jani Hiriven		:Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland	:PPM Encoder
 *  Jose Julio			:Stabilization Control laws, MPU6k driver
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine			:Tri Support, Graphics
 *  Leonard Hall 		:Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini		:Lead tester
 *  Michael Oborne		:Mission Planner GCS
 *  Mike Smith			:Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraaction Layer (HAL)
 *  Robert Lefebvre		:Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/diydrones/ardupilot/graphs/contributors
 *  Wiki: http://copter.ardupilot.com/
 *  Requires modified version of Arduino, which can be found here: http://ardupilot.com/downloads/?category=6
 *
 */

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

// Common dependencies
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Menu.h>
#include <AP_Param.h>
// AP_HAL
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>

// Application dependencies
#include <AP_GPS.h>             // ArduPilot GPS library
#include <AP_GPS_Glitch.h>      // GPS glitch protection library
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>
#include <APM_PI.h>             // PI library
#include <AC_PID.h>             // PID library
#include <RC_Channel.h>         // RC Channel Library
#include <AP_Motors.h>          // AP Motors library
#include <AP_RangeFinder.h>     // Range finder library
#include <AP_OpticalFlow.h>     // Optical Flow library
#include <Filter.h>             // Filter library
#include <AP_Buffer.h>          // APM FIFO Buffer
#include <AP_Relay.h>           // APM relay
#include <AP_ServoRelayEvents.h>
#include <AP_Airspeed.h>        // needed for AHRS build
#include <AP_Vehicle.h>         // needed for AHRS build
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <AC_WPNav.h>     		// ArduCopter waypoint navigation library
#include <AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <SITL.h>               // software in the loop support
#include <AP_Scheduler.h>       // main loop scheduler
#include <AP_RCMapper.h>        // RC input mapping library
#include <AP_Notify.h>          // Notify library
#include <AP_BattMonitor.h>     // Battery monitor library
#include <AP_BoardConfig.h>     // board configuration library
#if SPRAYER == ENABLED
#include <AC_Sprayer.h>         // crop sprayer library
#endif
#if EPM_ENABLED == ENABLED
#include <AP_EPM.h>				// EPM cargo gripper stuff
#endif

// AP_HAL to Arduino compatibility layer
#include "compat.h"
// Configuration
#include "defines.h"
#include "config.h"
#include "config_channels.h"

// Local modules
#include "Parameters.h"
#include "GCS.h"

////////////////////////////////////////////////////////////////////////////////
// cliSerial
////////////////////////////////////////////////////////////////////////////////
// cliSerial isn't strictly necessary - it is an alias for hal.console. It may
// be deprecated in favor of hal.console in later releases.
static AP_HAL::BetterStream* cliSerial;

// N.B. we need to keep a static declaration which isn't guarded by macros
// at the top to cooperate with the prototype mangler.

////////////////////////////////////////////////////////////////////////////////
// AP_HAL instance
////////////////////////////////////////////////////////////////////////////////

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

////////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////////
//
// Global parameters are all contained within the 'g' class.
//
static Parameters g;

// main loop scheduler
static AP_Scheduler scheduler;

// AP_Notify instance
static AP_Notify notify;



////////////////////////////////////////////////////////////////////////////////
// prototypes
////////////////////////////////////////////////////////////////////////////////
static void update_events(void);
static void print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode);

////////////////////////////////////////////////////////////////////////////////
// Dataflash
////////////////////////////////////////////////////////////////////////////////
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
static DataFlash_APM2 DataFlash;
#endif

////////////////////////////////////////////////////////////////////////////////
// the rate we run the main loop at
////////////////////////////////////////////////////////////////////////////////
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_100HZ;

////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
//
// There are three basic options related to flight sensor selection.
//
// - Normal flight mode. Real sensors are used.
// - HIL Attitude mode. Most sensors are disabled, as the HIL
//   protocol supplies attitude information directly.
// - HIL Sensors mode. Synthetic sensors are configured that
//   supply data from the simulation.
//

// All GPS access should be through this pointer.
static GPS         *g_gps;
static GPS_Glitch   gps_glitch(g_gps);

// flight modes convenience array
static AP_Int8 *flight_modes = &g.flight_mode1;

#if HIL_MODE == HIL_MODE_DISABLED

 #if CONFIG_ADC == ENABLED
static AP_ADC_ADS7844 adc;
 #endif

 #if CONFIG_IMU_TYPE == CONFIG_IMU_MPU6000
static AP_InertialSensor_MPU6000 ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_OILPAN
static AP_InertialSensor_Oilpan ins(&adc);
#elif CONFIG_IMU_TYPE == CONFIG_IMU_SITL
static AP_InertialSensor_HIL ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_PX4
static AP_InertialSensor_PX4 ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_FLYMAPLE
AP_InertialSensor_Flymaple ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_L3G4200D
AP_InertialSensor_L3G4200D ins;
#endif

 #if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 // When building for SITL we use the HIL barometer and compass drivers
static AP_Baro_HIL barometer;
static AP_Compass_HIL compass;
static SITL sitl;
 #else
// Otherwise, instantiate a real barometer and compass driver
  #if CONFIG_BARO == AP_BARO_BMP085
static AP_Baro_BMP085 barometer;
  #elif CONFIG_BARO == AP_BARO_PX4
static AP_Baro_PX4 barometer;
  #elif CONFIG_BARO == AP_BARO_MS5611
   #if CONFIG_MS5611_SERIAL == AP_BARO_MS5611_SPI
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
   #elif CONFIG_MS5611_SERIAL == AP_BARO_MS5611_I2C
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
   #else
    #error Unrecognized CONFIG_MS5611_SERIAL setting.
   #endif
  #endif

static AP_Compass_HMC5843 compass;
 #endif

// real GPS selection
 #if GPS_PROTOCOL == GPS_PROTOCOL_NONE
AP_GPS_None     g_gps_driver;
 #else
  #error Unrecognised GPS_PROTOCOL setting.
 #endif // GPS PROTOCOL

static AP_AHRS_DCM ahrs(ins, g_gps);

#elif HIL_MODE == HIL_MODE_SENSORS
// sensor emulators
static AP_ADC_HIL              adc;
static AP_Baro_HIL      barometer;
static AP_Compass_HIL          compass;
static AP_GPS_HIL              g_gps_driver;
static AP_InertialSensor_HIL   ins;
static AP_AHRS_DCM             ahrs(ins, g_gps);

#elif HIL_MODE == HIL_MODE_ATTITUDE
static AP_ADC_HIL              adc;
static AP_InertialSensor_HIL   ins;
static AP_AHRS_HIL             ahrs(ins, g_gps);
static AP_GPS_HIL              g_gps_driver;
static AP_Compass_HIL          compass;                  // never used
static AP_Baro_HIL      barometer;

#else
 #error Unrecognised HIL_MODE setting.
#endif // HIL MODE

////////////////////////////////////////////////////////////////////////////////
// SONAR selection
////////////////////////////////////////////////////////////////////////////////
//
ModeFilterInt16_Size3 sonar_mode_filter(1);
#if CONFIG_SONAR == ENABLED
static AP_HAL::AnalogSource *sonar_analog_source;
static AP_RangeFinder_MaxsonarXL *sonar;
#endif

////////////////////////////////////////////////////////////////////////////////
// User variables
////////////////////////////////////////////////////////////////////////////////
#ifdef USERHOOK_VARIABLES
 #include USERHOOK_VARIABLES
#endif

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

/* Radio values
 *               Channel assignments
 *                       1	Ailerons (rudder if no ailerons)
 *                       2	Elevator
 *                       3	Throttle
 *                       4	Rudder (if we have ailerons)
 *                       5	Mode - 3 position switch
 *                       6  User assignable
 *                       7	trainer switch - sets throttle nominal (toggle switch), sets accels to Level (hold > 1 second)
 *                       8	TBD
 *               Each Aux channel can be configured to have any of the available auxiliary functions assigned to it.
 *               See libraries/RC_Channel/RC_Channel_aux.h for more information
 */

//Documentation of GLobals:
static union {
    struct {
        uint8_t home_is_set         : 1; // 0
        uint8_t simple_mode         : 2; // 1,2 // This is the state of simple mode : 0 = disabled ; 1 = SIMPLE ; 2 = SUPERSIMPLE

        uint8_t pre_arm_rc_check    : 1; // 3   // true if rc input pre-arm checks have been completed successfully
        uint8_t pre_arm_check       : 1; // 4   // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
        uint8_t auto_armed          : 1; // 5   // stops auto missions from beginning until throttle is raised
        uint8_t logging_started     : 1; // 6   // true if dataflash logging has started

        uint8_t do_flip             : 1; // 7   // Used to enable flip code
        uint8_t takeoff_complete    : 1; // 8
        uint8_t land_complete       : 1; // 9   // true if we have detected a landing

        uint8_t new_radio_frame     : 1; // 10      // Set true if we have new PWM data to act on from the Radio
        uint8_t CH7_flag            : 2; // 11,12   // ch7 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH8_flag            : 2; // 13,14   // ch8 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t usb_connected       : 1; // 15      // true if APM is powered from USB connection
        uint8_t yaw_stopped         : 1; // 16      // Used to manage the Yaw hold capabilities

        uint8_t disable_stab_rate_limit : 1; // 17  // disables limits rate request from the stability controller

        uint8_t rc_receiver_present : 1; // 18  // true if we have an rc receiver present (i.e. if we've ever received an update
    };
    uint32_t value;
} ap;

////////////////////////////////////////////////////////////////////////////////
// Radio
////////////////////////////////////////////////////////////////////////////////
// This is the state of the flight control system
// There are multiple states defined such as STABILIZE, ACRO,
static int8_t control_mode = STABILIZE;
// Used to maintain the state of the previous control switch position
// This is set to -1 when we need to re-read the switch
static uint8_t oldSwitchPosition;
static RCMapper rcmap;

// board specific config
static AP_BoardConfig BoardConfig;

// receiver RSSI
static uint8_t receiver_rssi;

////////////////////////////////////////////////////////////////////////////////
// Failsafe
////////////////////////////////////////////////////////////////////////////////
static struct {
    uint8_t rc_override_active  : 1; // 0   // true if rc control are overwritten by ground station
    uint8_t radio               : 1; // 1   // A status flag for the radio failsafe
    uint8_t battery             : 1; // 2   // A status flag for the battery failsafe

    int8_t radio_counter;                  // number of iterations with throttle below throttle_fs_value

    uint32_t last_heartbeat_ms;             // the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
} failsafe;

////////////////////////////////////////////////////////////////////////////////
// Motor Output
////////////////////////////////////////////////////////////////////////////////
// Moved this to userVariables because of scope

// #define MOTOR_CLASS AP_MotorsQuad

//static MOTOR_CLASS motors(&g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4);

////////////////////////////////////////////////////////////////////////////////
// PIDs
////////////////////////////////////////////////////////////////////////////////
// This is a convienience accessor for the IMU roll rates. It's currently the raw IMU rates
// and not the adjusted omega rates, but the name is stuck
static Vector3f omega;
// This is used to hold radio tuning values for in-flight CH6 tuning
float tuning_value;



// A LOT OF STUFF THAT WAS HERE WAS MOVED TO USER SPACE

////////////////////////////////////////////////////////////////////////////////
// Performance monitoring
////////////////////////////////////////////////////////////////////////////////
static int16_t pmTest1;

// System Timers
// --------------
// Time in microseconds of main control loop
static uint32_t fast_loopTimer;
// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t mainLoop_count;
// Loiter timer - Records how long we have been in loiter
static uint32_t rtl_loiter_start_time;

// Used to exit the roll and pitch auto trim function
static uint8_t auto_trim_counter;

// Reference to the relay object (APM1 -> PORTL 2) (APM2 -> PORTB 7)
static AP_Relay relay;

// handle repeated servo and relay events
static AP_ServoRelayEvents ServoRelayEvents(relay);

// a pin for reading the receiver RSSI voltage.
static AP_HAL::AnalogSource* rssi_analog_source;


// Input sources for battery voltage, battery current, board vcc
static AP_HAL::AnalogSource* board_vcc_analog_source;


#if CLI_ENABLED == ENABLED
    static int8_t   setup_show (uint8_t argc, const Menu::arg *argv);
#endif

// Camera/Antenna mount tracking and stabilisation stuff
// --------------------------------------


////////////////////////////////////////////////////////////////////////////////
// Crop Sprayer
////////////////////////////////////////////////////////////////////////////////
#if SPRAYER == ENABLED
static AC_Sprayer sprayer(&inertial_nav);
#endif

////////////////////////////////////////////////////////////////////////////////
// EPM Cargo Griper
////////////////////////////////////////////////////////////////////////////////
#if EPM_ENABLED == ENABLED
static AP_EPM epm;
#endif

////////////////////////////////////////////////////////////////////////////////
// function definitions to keep compiler from complaining about undeclared functions
////////////////////////////////////////////////////////////////////////////////
void get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate);
static void pre_arm_checks(bool display_failure);

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

// setup the var_info table
AP_Param param_loader(var_info, WP_START_BYTE);

/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take (in
  microseconds)
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { throttle_loop,         2,     450 },
//    { update_nav_mode,       1,     400 },
    { update_batt_compass,  10,     720 },
//    { read_aux_switches,    10,      50 },
    { arm_motors_check,     10,      10 },
    { auto_trim,            10,     140 }, // CHECK
    { update_altitude,      10,    1000 },
//    { run_nav_updates,      10,     800 },
    { three_hz_loop,        33,      90 },
    { compass_accumulate,    2,     420 },
    { barometer_accumulate,  2,     250 },
    { update_notify,         2,     100 },
    { one_hz_loop,         100,     420 },
    { crash_check,          10,      20 },
//    { ten_hz_logging_loop,  10,     300 },
//    { fifty_hz_logging_loop, 2,     220 },
    { perf_update,        1000,     200 },
//    { read_receiver_rssi,   10,      50 },
#ifdef USERHOOK_FASTLOOP
    { userhook_FastLoop,     1,    100  },
#endif
#ifdef USERHOOK_50HZLOOP
    { userhook_50Hz,         2,    100  },
#endif
#ifdef USERHOOK_MEDIUMLOOP
    { userhook_MediumLoop,   10,    1000 },
#endif
#ifdef USERHOOK_SLOWLOOP
    { userhook_SlowLoop,     30,    100 },
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    { userhook_SuperSlowLoop,101,   1000 },
#endif
};




/// COPY HERERERERE




void setup() 
{
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
    
}

/*
  if the compass is enabled then try to accumulate a reading
 */
static void compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }
}

/*
  try to accumulate a baro reading
 */
static void barometer_accumulate(void)
{
    barometer.accumulate();
}

static void perf_update(void)
{
    if (g.log_bitmask & MASK_LOG_PM)
        Log_Write_Performance();
    if (scheduler.debug()) {
        cliSerial->printf_P(PSTR("PERF: %u/%u %lu\n"),
                            (unsigned)perf_info_get_num_long_running(),
                            (unsigned)perf_info_get_num_loops(),
                            (unsigned long)perf_info_get_max_time());
    }
    perf_info_reset();
    pmTest1 = 0;
}

void loop()
{
    // wait for an INS sample
    if (!ins.wait_for_sample(1000)) {
        Log_Write_Error(ERROR_SUBSYSTEM_MAIN, ERROR_CODE_MAIN_INS_DELAY);
        return;
    }
    uint32_t timer = micros();

    // check loop time
    perf_info_check_loop_time(timer - fast_loopTimer);

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // Execute the fast loop
    // ---------------------
    fast_loop();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + 10000) - micros();
    scheduler.run(time_available - 300);
}


// Main loop - 100hz
static void fast_loop()
{

    // IMU DCM Algorithm
    // Reads internal sensors (?)
    // --------------------
    read_AHRS();

    // reads all of the necessary trig functions for cameras, throttle, etc.
    // --------------------------------------------------------------------
    update_trig();

    // Acrobatic control
    if (ap.do_flip) {
        if(abs(g.rc_1.control_in) < 4000) {
            // calling roll_flip will override the desired roll rate and throttle output
            roll_flip();
        }else{
            // force an exit from the loop if we are not hands off sticks.
            ap.do_flip = false;
            Log_Write_Event(DATA_EXIT_FLIP);
        }
    }

    // run low level rate controllers that only require IMU data
    run_rate_controllers();
   

    // write out the servo PWM values
    // ------------------------------
    set_servos_4();

    // Inertial Nav
    // --------------------
    read_inertia();

    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    //read_control_switch();
    
    // update mode according to RC controller // custom
    update_mode();
    //readUartB();
    
    
    // custom code/exceptions for flight modes
    // ---------------------------------------
    update_yaw_mode();
    update_roll_pitch_mode();

    // update targets to rate controllers
    update_rate_contoller_targets();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
static void throttle_loop()
{
    // get altitude and climb rate from inertial lib
    read_inertial_altitude();

    // Update the throttle ouput
    // -------------------------
    update_throttle_mode();

    // check if we've landed
    update_land_detector();

    // check auto_armed status
    update_auto_armed();
}

// update_mount - update camera mount position
// should be run at 50hz
static void update_mount()
{

}

// update_batt_compass - read battery and compass
// should be called at 10hz
static void update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    read_battery();

#if HIL_MODE != HIL_MODE_ATTITUDE  // don't execute in HIL mode
    if(g.compass_enabled) {
        if (compass.read()) {
            compass.null_offsets();
        }
        // log compass information
        if (g.log_bitmask & MASK_LOG_COMPASS) {
            Log_Write_Compass();
        }
    }
#endif

    // record throttle output
    throttle_integrator += g.rc_3.servo_out;
}


// three_hz_loop - 3.3hz loop
static void three_hz_loop()
{

#if SPRAYER == ENABLED
    sprayer.update();
#endif

    update_events();

    if(g.radio_tuning > 0)
        tuning();
}

// one_hz_loop - runs at 1Hz
static void one_hz_loop()
{  
    if (g.log_bitmask != 0) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    // pass latest alt hold kP value to navigation controller
    wp_nav.set_althold_kP(g.pi_alt_hold.kP());

    // update latest lean angle to navigation controller
    wp_nav.set_lean_angle_max(g.angle_max);

    // log battery info to the dataflash
    if (g.log_bitmask & MASK_LOG_CURRENT) {
        Log_Write_Current();
    }

    // perform pre-arm checks & display failures every 30 seconds
    static uint8_t pre_arm_display_counter = 15;
    pre_arm_display_counter++;
    if (pre_arm_display_counter >= 30) {
        pre_arm_checks(true);
        pre_arm_display_counter = 0;
    }else{
        pre_arm_checks(false);
    }

    // auto disarm checks
//    auto_disarm_check();

    // CHECK
    if (!motors.armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.set_orientation();

        // check the user hasn't updated the frame orientation
        motors.set_frame_orientation(g.frame_orientation);
    }

    // update assigned functions and enable auxiliar servos
    aux_servos_update_fn();
    enable_aux_servos();

    check_usb_mux();
}

// called at 100hz but data from sensor only arrives at 20 Hz
static void update_optical_flow(void)
{
}

// called at 50hz
static void update_GPS(void)
{
    return; 
}

// set_yaw_mode - update yaw mode and initialise any variables required
bool set_yaw_mode(uint8_t new_yaw_mode)
{
    // boolean to ensure proper initialisation of throttle modes
    bool yaw_initialised = false;

    // return immediately if no change
    if( new_yaw_mode == yaw_mode ) {
        return true;
    }

    switch( new_yaw_mode ) {
        case YAW_HOLD:
            yaw_initialised = true;
            break;
        case YAW_ACRO:
            yaw_initialised = true;
            acro_yaw_rate = 0;
            break;
        case YAW_LOOK_AT_NEXT_WP:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AT_LOCATION:
            if( ap.home_is_set ) {
                // update bearing - assumes yaw_look_at_WP has been intialised before set_yaw_mode was called
                yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), yaw_look_at_WP);
                yaw_initialised = true;
            }
            break;
        case YAW_CIRCLE:
            if( ap.home_is_set ) {
                // set yaw to point to center of circle
                yaw_look_at_WP = circle_center;
                // initialise bearing to current heading
                yaw_look_at_WP_bearing = ahrs.yaw_sensor;
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AT_HEADING:
            yaw_initialised = true;
            break;
        case YAW_LOOK_AT_HOME:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AHEAD:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_DRIFT:
            yaw_initialised = true;
            break;
        case YAW_RESETTOARMEDYAW:
            control_yaw = ahrs.yaw_sensor; // store current yaw so we can start rotating back to correct one
            yaw_initialised = true;
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( yaw_initialised ) {
        yaw_mode = new_yaw_mode;
    }

    // return success or failure
    return yaw_initialised;
}

// update_yaw_mode - run high level yaw controllers
// 100hz update rate
void update_yaw_mode(void)
{
    int16_t pilot_yaw;
    if(flymode == auto_mode){
      pilot_yaw = receivedCommands.yaw;
      //yaw_mode = YAW_HOLD;
      //receivedCommands.yaw = 0;
    } else {
      pilot_yaw = g.rc_4.control_in;
    }
  
    // do not process pilot's yaw input during radio failsafe
    if (failsafe.radio) {
        pilot_yaw = 0;
    }

    switch(yaw_mode) {

    case YAW_HOLD:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }
        // heading hold at heading held in control_yaw but allow input from pilot
        get_yaw_rate_stabilized_ef(pilot_yaw);
        break;

    case YAW_ACRO:
        // pilot controlled yaw using rate controller
        get_yaw_rate_stabilized_bf(pilot_yaw);
        break;

    case YAW_LOOK_AT_NEXT_WP:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }else{
            // point towards next waypoint (no pilot input accepted)
            // we don't use wp_bearing because we don't want the copter to turn too much during flight
            control_yaw = get_yaw_slew(control_yaw, original_wp_bearing, AUTO_YAW_SLEW_RATE);
        }
        get_stabilize_yaw(control_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_LOCATION:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }
        // point towards a location held in yaw_look_at_WP
        get_look_at_yaw();

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_CIRCLE:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }
        // points toward the center of the circle or does a panorama
        get_circle_yaw();

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_HOME:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }else{
            // keep heading always pointing at home with no pilot input allowed
            control_yaw = get_yaw_slew(control_yaw, home_bearing, AUTO_YAW_SLEW_RATE);
        }
        get_stabilize_yaw(control_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_HEADING:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }else{
            // keep heading pointing in the direction held in yaw_look_at_heading with no pilot input allowed
            control_yaw = get_yaw_slew(control_yaw, yaw_look_at_heading, yaw_look_at_heading_slew);
        }
        get_stabilize_yaw(control_yaw);
        break;

	case YAW_LOOK_AHEAD:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }
		// Commanded Yaw to automatically look ahead.
        get_look_ahead_yaw(pilot_yaw);
        break;

    case YAW_DRIFT:
        // if we have landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }
        get_yaw_drift();
        break;

    case YAW_RESETTOARMEDYAW:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }else{
            // changes yaw to be same as when quad was armed
            control_yaw = get_yaw_slew(control_yaw, initial_armed_bearing, AUTO_YAW_SLEW_RATE);
        }
        get_stabilize_yaw(control_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }

        break;
    }
}

// get yaw mode based on WP_YAW_BEHAVIOR parameter
// set rtl parameter to true if this is during an RTL
uint8_t get_wp_yaw_mode(bool rtl)
{
    switch (g.wp_yaw_behavior) {
        case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP:
            return YAW_LOOK_AT_NEXT_WP;
            break;

        case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL:
            if( rtl ) {
                return YAW_HOLD;
            }else{
                return YAW_LOOK_AT_NEXT_WP;
            }
            break;

        case WP_YAW_BEHAVIOR_LOOK_AHEAD:
            return YAW_LOOK_AHEAD;
            break;

        default:
            return YAW_HOLD;
            break;
    }
}

// set_roll_pitch_mode - update roll/pitch mode and initialise any variables as required
bool set_roll_pitch_mode(uint8_t new_roll_pitch_mode)
{
    // boolean to ensure proper initialisation of throttle modes
    bool roll_pitch_initialised = false;

    // return immediately if no change
    if( new_roll_pitch_mode == roll_pitch_mode ) {
        return true;
    }

    switch( new_roll_pitch_mode ) {
        case ROLL_PITCH_STABLE:
            roll_pitch_initialised = true;
            break;
        case ROLL_PITCH_ACRO:
            // reset acro level rates
            acro_roll_rate = 0;
            acro_pitch_rate = 0;
            roll_pitch_initialised = true;
            break;
        case ROLL_PITCH_AUTO:
        case ROLL_PITCH_STABLE_OF:
        case ROLL_PITCH_DRIFT:
        case ROLL_PITCH_LOITER:
        case ROLL_PITCH_SPORT:
            roll_pitch_initialised = true;
            break;

#if AUTOTUNE == ENABLED
        case ROLL_PITCH_AUTOTUNE:
            // only enter autotune mode from stabilized roll-pitch mode when armed and flying
            if (roll_pitch_mode == ROLL_PITCH_STABLE && motors.armed() && !ap.land_complete) {
                // auto_tune_start returns true if it wants the roll-pitch mode changed to autotune
                roll_pitch_initialised = auto_tune_start();
            }
            break;
#endif
    }

    // if initialisation has been successful update the yaw mode
    if( roll_pitch_initialised ) {
        exit_roll_pitch_mode(roll_pitch_mode);
        roll_pitch_mode = new_roll_pitch_mode;
    }

    // return success or failure
    return roll_pitch_initialised;
}

// exit_roll_pitch_mode - peforms any code required when exiting the current roll-pitch mode
void exit_roll_pitch_mode(uint8_t old_roll_pitch_mode)
{
#if AUTOTUNE == ENABLED
    if (old_roll_pitch_mode == ROLL_PITCH_AUTOTUNE) {
        auto_tune_stop();
    }
#endif
}


// update_roll_pitch_mode - run high level roll and pitch controllers
// 100hz update rate
void update_roll_pitch_mode(void)
{
    
    switch(roll_pitch_mode) {
    case ROLL_PITCH_ACRO:
        // copy user input for reporting purposes
        /*if(flymode == auto_mode){
            control_roll = receivedCommands.roll;
            control_pitch = receivedCommands.pitch;
            receivedCommands.roll = 0;
            receivedCommands.pitch = 0;
         } else {*/
            control_roll = g.rc_1.control_in;
            control_pitch = g.rc_1.control_in;
         //}

        acro_level_mix = constrain_float(1-max(max(abs(g.rc_1.control_in), abs(g.rc_2.control_in)), abs(g.rc_4.control_in))/4500.0, 0, 1)*cos_pitch_x;
        get_roll_rate_stabilized_bf(g.rc_1.control_in);
        get_pitch_rate_stabilized_bf(g.rc_2.control_in);
        get_acro_level_rates();
        break;

    case ROLL_PITCH_STABLE:
        // apply SIMPLE mode transform
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, control_roll, control_pitch);

        // pass desired roll, pitch to stabilize attitude controllers
        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);

        break;

    case ROLL_PITCH_AUTO:
        // copy latest output from nav controller to stabilize controller
        control_roll = wp_nav.get_desired_roll();
        control_pitch = wp_nav.get_desired_pitch();
        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);
        break;

    case ROLL_PITCH_STABLE_OF:
        // apply SIMPLE mode transform
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, control_roll, control_pitch);

        // mix in user control with optical flow
        control_roll = get_of_roll(control_roll);
        control_pitch = get_of_pitch(control_pitch);

        // call stabilize controller
        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);
        break;

    case ROLL_PITCH_DRIFT:
        get_roll_pitch_drift();
        break;

    case ROLL_PITCH_LOITER:
        // apply SIMPLE mode transform
        update_simple_mode();

        // update loiter target from user controls
        wp_nav.move_loiter_target(g.rc_1.control_in, g.rc_2.control_in, 0.01f);

        // copy latest output from nav controller to stabilize controller
        control_roll = wp_nav.get_desired_roll();
        control_pitch = wp_nav.get_desired_pitch();

        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);
        break;

    case ROLL_PITCH_SPORT:
        // apply SIMPLE mode transform
        update_simple_mode();

        // copy user input for reporting purposes
        control_roll = g.rc_1.control_in;
        control_pitch = g.rc_2.control_in;
        get_roll_rate_stabilized_ef(g.rc_1.control_in);
        get_pitch_rate_stabilized_ef(g.rc_2.control_in);
        break;

#if AUTOTUNE == ENABLED
    case ROLL_PITCH_AUTOTUNE:
        // apply SIMPLE mode transform
        if(ap.simple_mode && ap.new_radio_frame) {
            update_simple_mode();
        }

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, control_roll, control_pitch);

        // pass desired roll, pitch to stabilize attitude controllers
        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);

        // copy user input for reporting purposes
        get_autotune_roll_pitch_controller(g.rc_1.control_in, g.rc_2.control_in, g.rc_4.control_in);
        break;
#endif
    }

	#if FRAME_CONFIG != HELI_FRAME
    if(g.rc_3.control_in == 0 && control_mode <= ACRO) {
        reset_rate_I();
    }
	#endif //HELI_FRAME

    if(ap.new_radio_frame) {
        // clear new radio frame info
        ap.new_radio_frame = false;
    }
}

static void
init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = cos_yaw;
    simple_sin_yaw = sin_yaw;

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

    // log the simple bearing to dataflash
    if (g.log_bitmask != 0) {
        Log_Write_Data(DATA_INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
}

// update_simple_mode - rotates pilot input if we are in simple mode
void update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (ap.simple_mode == 0 || !ap.new_radio_frame) {
        return;
    }

    if (ap.simple_mode == 1) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = g.rc_1.control_in*simple_cos_yaw - g.rc_2.control_in*simple_sin_yaw;
        pitchx = g.rc_1.control_in*simple_sin_yaw + g.rc_2.control_in*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = g.rc_1.control_in*super_simple_cos_yaw - g.rc_2.control_in*super_simple_sin_yaw;
        pitchx = g.rc_1.control_in*super_simple_sin_yaw + g.rc_2.control_in*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    g.rc_1.control_in = rollx*cos_yaw + pitchx*sin_yaw;
    g.rc_2.control_in = -rollx*sin_yaw + pitchx*cos_yaw;
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void update_super_simple_bearing(bool force_update)
{
    // check if we are in super simple mode and at least 10m from home
    if(force_update || (ap.simple_mode == 2 && home_distance > SUPER_SIMPLE_RADIUS)) {
        // check the bearing to home has changed by at least 5 degrees
        if (labs(super_simple_last_bearing - home_bearing) > 500) {
            super_simple_last_bearing = home_bearing;
            float angle_rad = radians((super_simple_last_bearing+18000)/100);
            super_simple_cos_yaw = cosf(angle_rad);
            super_simple_sin_yaw = sinf(angle_rad);
        }
    }
}

// throttle_mode_manual - returns true if the throttle is directly controlled by the pilot
bool throttle_mode_manual(uint8_t thr_mode)
{
    return (thr_mode == THROTTLE_MANUAL || thr_mode == THROTTLE_MANUAL_TILT_COMPENSATED || thr_mode == THROTTLE_MANUAL_HELI);
}

// set_throttle_mode - sets the throttle mode and initialises any variables as required
bool set_throttle_mode( uint8_t new_throttle_mode )
{
    // boolean to ensure proper initialisation of throttle modes
    bool throttle_initialised = false;

    // return immediately if no change
    if( new_throttle_mode == throttle_mode ) {
        return true;
    }

    // initialise any variables required for the new throttle mode
    switch(new_throttle_mode) {
        case THROTTLE_MANUAL:
        case THROTTLE_MANUAL_TILT_COMPENSATED:
            throttle_accel_deactivate();                // this controller does not use accel based throttle controller
            altitude_error = 0;                         // clear altitude error reported to GCS
            throttle_initialised = true;
            break;

        case THROTTLE_HOLD:
        case THROTTLE_AUTO:
            if(flymode == auto_mode)
              controller_desired_alt = receivedCommands.targetHeight;
            else
              controller_desired_alt = get_initial_alt_hold(current_loc.alt, climb_rate);     // reset controller desired altitude to current altitude
            wp_nav.set_desired_alt(controller_desired_alt);                                 // same as above but for loiter controller
            if (throttle_mode_manual(throttle_mode)) {  // reset the alt hold I terms if previous throttle mode was manual
                reset_throttle_I();
                set_accel_throttle_I_from_pilot_throttle(get_pilot_desired_throttle(g.rc_3.control_in));
            }
            throttle_initialised = true;
            break;

        case THROTTLE_LAND:
            reset_land_detector();  // initialise land detector
            controller_desired_alt = get_initial_alt_hold(current_loc.alt, climb_rate);   // reset controller desired altitude to current altitude
            throttle_initialised = true;
            break;
    }

    // update the throttle mode
    if( throttle_initialised ) {
        throttle_mode = new_throttle_mode;

        // reset some variables used for logging
        desired_climb_rate = 0;
        nav_throttle = 0;
    }

    // return success or failure
    return throttle_initialised;
}

// update_throttle_mode - run high level throttle controllers
// 50 hz update rate
void update_throttle_mode(void)
{
    int16_t pilot_climb_rate;
    int16_t pilot_throttle_scaled;

    if(ap.do_flip)     // this is pretty bad but needed to flip in AP modes.
        return;

#if FRAME_CONFIG != HELI_FRAME
    // do not run throttle controllers if motors disarmed
    if( !motors.armed() ) {
        set_throttle_out(0, false);
        throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
        set_target_alt_for_reporting(0);
        return;
    }
#endif // FRAME_CONFIG != HELI_FRAME

/*    if(flymode == auto_mode){ 
    // custom
    // Same idea as Throttle Manual
      //hal.uartB->printf("Auto Throttle %d\n", receivedCommands.throttle);
      //set_throttle_out(receivedCommands.throttle, false);
      //update_throttle_cruise(receivedCommands.throttle);
     //send pilot's output directly to motors


      pilot_throttle_scaled = get_pilot_desired_throttle(receivedCommands.throttle);
      set_throttle_out(pilot_throttle_scaled, false);
      // check if we've taken off yet
      if (!ap.takeoff_complete && motors.armed()) {
          if (pilot_throttle_scaled > g.throttle_cruise) {
              // we must be in the air by now
              set_takeoff_complete(true); // this function logs takeoff and landing
          }
      }
      set_target_alt_for_reporting(0);
    } else{
      //hal.uartB->printf("Manual Throttle %d\n", get_pilot_desired_throttle(g.rc_3.control_in));
  */    
    switch(throttle_mode) {

    case THROTTLE_MANUAL:
          // completely manual throttle
          if(g.rc_3.control_in <= 0){
              set_throttle_out(0, false);
          }else{
              // send pilot's output directly to motors
              pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);
              set_throttle_out(pilot_throttle_scaled, false);

              // update estimate of throttle cruise

  	      update_throttle_cruise(pilot_throttle_scaled);

              // check if we've taken off yet
              if (!ap.takeoff_complete && motors.armed()) {
                  if (pilot_throttle_scaled > g.throttle_cruise) {
                      // we must be in the air by now
                      set_takeoff_complete(true); // this function logs takeoff and landing
                  }
              }
          }
        set_target_alt_for_reporting(0);
        break;

    case THROTTLE_MANUAL_TILT_COMPENSATED:
        // manual throttle but with angle boost
        if (g.rc_3.control_in <= 0) {
            set_throttle_out(0, false); // no need for angle boost with zero throttle
        }else{
            pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);
            set_throttle_out(pilot_throttle_scaled, true);

            // update estimate of throttle cruise
      			update_throttle_cruise(pilot_throttle_scaled);

            if (!ap.takeoff_complete && motors.armed()) {
                if (pilot_throttle_scaled > g.throttle_cruise) {
                    // we must be in the air by now
                    set_takeoff_complete(true);
                }
            }
        }
        set_target_alt_for_reporting(0);
        break;

    case THROTTLE_HOLD:
        if(ap.auto_armed) {
            // alt hold plus pilot input of climb rate
            pilot_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

            // special handling if we have landed
            if (ap.land_complete) {
                if (pilot_climb_rate > 0) {
                    // indicate we are taking off
                    set_land_complete(false);
                    // clear i term when we're taking off
                    set_throttle_takeoff();
                }else{
                    // move throttle to minimum to keep us on the ground
                    set_throttle_out(0, false);
                    // deactivate accel based throttle controller (it will be automatically re-enabled when alt-hold controller next runs)
                    throttle_accel_deactivate();
                }
            }
            // check land_complete flag again in case it was changed above
            if (!ap.land_complete) {
                if( sonar_alt_health >= SONAR_ALT_HEALTH_MAX ) {
                    // if sonar is ok, use surface tracking
                    get_throttle_surface_tracking(pilot_climb_rate);    // this function calls set_target_alt_for_reporting for us
                }else{
                    // if no sonar fall back stabilize rate controller
                    get_throttle_rate_stabilized(pilot_climb_rate);     // this function calls set_target_alt_for_reporting for us
                }
            }
        }else{
            // pilot's throttle must be at zero so keep motors off
            set_throttle_out(0, false);
            // deactivate accel based throttle controller
            throttle_accel_deactivate();
            set_target_alt_for_reporting(0);
        }
        break;

    case THROTTLE_AUTO:
        // auto pilot altitude controller with target altitude held in wp_nav.get_desired_alt()
        if(ap.auto_armed) {
            // special handling if we are just taking off
            if (ap.land_complete) {
                // tell motors to do a slow start.
                motors.slow_start(true);
            }
            get_throttle_althold_with_slew(wp_nav.get_desired_alt(), -wp_nav.get_descent_velocity(), wp_nav.get_climb_velocity());
            set_target_alt_for_reporting(wp_nav.get_desired_alt()); // To-Do: return get_destination_alt if we are flying to a waypoint
        }else{
            // pilot's throttle must be at zero so keep motors off
            set_throttle_out(0, false);
            // deactivate accel based throttle controller
            throttle_accel_deactivate();
            set_target_alt_for_reporting(0);
        }
        break;

    case THROTTLE_LAND:
        // landing throttle controller
        get_throttle_land();
        set_target_alt_for_reporting(0);
        break;

    }
   // } // TEMP for if statement on flymode
}

// set_target_alt_for_reporting - set target altitude in cm for reporting purposes (logs and gcs)
static void set_target_alt_for_reporting(float alt_cm)
{
    target_alt_for_reporting = alt_cm;
}

// get_target_alt_for_reporting - returns target altitude in cm for reporting purposes (logs and gcs)
static float get_target_alt_for_reporting()
{
    return target_alt_for_reporting;
}

static void read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before ahrs update
    gcs_check_input();
#endif

    ahrs.update();
    omega = ins.get_gyro();
}

static void update_trig(void){
    Vector2f yawvector;
    const Matrix3f &temp   = ahrs.get_dcm_matrix();

    yawvector.x     = temp.a.x;     // sin
    yawvector.y     = temp.b.x;         // cos
    yawvector.normalize();

    cos_pitch_x     = safe_sqrt(1 - (temp.c.x * temp.c.x));     // level = 1
    cos_roll_x      = temp.c.z / cos_pitch_x;                       // level = 1

    cos_pitch_x     = constrain_float(cos_pitch_x, 0, 1.0);
    // this relies on constrain_float() of infinity doing the right thing,
    // which it does do in avr-libc
    cos_roll_x      = constrain_float(cos_roll_x, -1.0, 1.0);

    sin_yaw         = constrain_float(yawvector.y, -1.0, 1.0);
    cos_yaw         = constrain_float(yawvector.x, -1.0, 1.0);

    // added to convert earth frame to body frame for rate controllers
    sin_pitch       = -temp.c.x;
    sin_roll        = temp.c.y / cos_pitch_x;

    // update wp_nav controller with trig values
    wp_nav.set_cos_sin_yaw(cos_yaw, sin_yaw, cos_pitch_x);

    //flat:
    // 0 ° = cos_yaw:  1.00, sin_yaw:  0.00,
    // 90° = cos_yaw:  0.00, sin_yaw:  1.00,
    // 180 = cos_yaw: -1.00, sin_yaw:  0.00,
    // 270 = cos_yaw:  0.00, sin_yaw: -1.00,
}

// read baro and sonar altitude at 20hz
static void update_altitude()
{
#if HIL_MODE == HIL_MODE_ATTITUDE
    // we are in the SIM, fake out the baro and Sonar
    baro_alt                = g_gps->altitude_cm;

    if(g.sonar_enabled) {
        sonar_alt           = baro_alt;
    }
#else
    // read in baro altitude
    baro_alt            = read_barometer();

    // read in sonar altitude
    sonar_alt           = read_sonar();
#endif  // HIL_MODE == HIL_MODE_ATTITUDE

    // write altitude info to dataflash logs
    if (g.log_bitmask & MASK_LOG_CTUN) {
        Log_Write_Control_Tuning();
    }
}

static void tuning(){

    // exit immediately when radio failsafe is invoked so tuning values are not set to zero
    if (failsafe.radio || failsafe.radio_counter != 0) {
        return;
    }

    tuning_value = (float)g.rc_6.control_in / 1000.0f;
    g.rc_6.set_range(g.radio_tuning_low,g.radio_tuning_high);                   // 0 to 1

    switch(g.radio_tuning) {

    // Roll, Pitch tuning
    case CH6_STABILIZE_ROLL_PITCH_KP:
        g.pi_stabilize_roll.kP(tuning_value);
        g.pi_stabilize_pitch.kP(tuning_value);
        break;

    case CH6_RATE_ROLL_PITCH_KP:
        g.pid_rate_roll.kP(tuning_value);
        g.pid_rate_pitch.kP(tuning_value);
        break;

    case CH6_RATE_ROLL_PITCH_KI:
        g.pid_rate_roll.kI(tuning_value);
        g.pid_rate_pitch.kI(tuning_value);
        break;

    case CH6_RATE_ROLL_PITCH_KD:
        g.pid_rate_roll.kD(tuning_value);
        g.pid_rate_pitch.kD(tuning_value);
        break;

    // Yaw tuning
    case CH6_STABILIZE_YAW_KP:
        g.pi_stabilize_yaw.kP(tuning_value);
        break;

    case CH6_YAW_RATE_KP:
        g.pid_rate_yaw.kP(tuning_value);
        break;

    case CH6_YAW_RATE_KD:
        g.pid_rate_yaw.kD(tuning_value);
        break;

    // Altitude and throttle tuning
    case CH6_ALTITUDE_HOLD_KP:
        g.pi_alt_hold.kP(tuning_value);
        break;

    case CH6_THROTTLE_RATE_KP:
        g.pid_throttle_rate.kP(tuning_value);
        break;

    case CH6_THROTTLE_RATE_KD:
        g.pid_throttle_rate.kD(tuning_value);
        break;

    case CH6_THROTTLE_ACCEL_KP:
        g.pid_throttle_accel.kP(tuning_value);
        break;

    case CH6_THROTTLE_ACCEL_KI:
        g.pid_throttle_accel.kI(tuning_value);
        break;

    case CH6_THROTTLE_ACCEL_KD:
        g.pid_throttle_accel.kD(tuning_value);
        break;

    // Loiter and navigation tuning
    case CH6_LOITER_POSITION_KP:
        g.pi_loiter_lat.kP(tuning_value);
        g.pi_loiter_lon.kP(tuning_value);
        break;

    case CH6_LOITER_RATE_KP:
        g.pid_loiter_rate_lon.kP(tuning_value);
        g.pid_loiter_rate_lat.kP(tuning_value);
        break;

    case CH6_LOITER_RATE_KI:
        g.pid_loiter_rate_lon.kI(tuning_value);
        g.pid_loiter_rate_lat.kI(tuning_value);
        break;

    case CH6_LOITER_RATE_KD:
        g.pid_loiter_rate_lon.kD(tuning_value);
        g.pid_loiter_rate_lat.kD(tuning_value);
        break;

    case CH6_WP_SPEED:
        // set waypoint navigation horizontal speed to 0 ~ 1000 cm/s
        wp_nav.set_horizontal_velocity(g.rc_6.control_in);
        break;

    // Acro roll pitch gain
    case CH6_ACRO_RP_KP:
        g.acro_rp_p = tuning_value;
        break;

    // Acro yaw gain
    case CH6_ACRO_YAW_KP:
        g.acro_yaw_p = tuning_value;
        break;

    case CH6_RELAY:
        if (g.rc_6.control_in > 525) relay.on(0);
        if (g.rc_6.control_in < 475) relay.off(0);
        break;

    case CH6_OPTFLOW_KP:
        g.pid_optflow_roll.kP(tuning_value);
        g.pid_optflow_pitch.kP(tuning_value);
        break;

    case CH6_OPTFLOW_KI:
        g.pid_optflow_roll.kI(tuning_value);
        g.pid_optflow_pitch.kI(tuning_value);
        break;

    case CH6_OPTFLOW_KD:
        g.pid_optflow_roll.kD(tuning_value);
        g.pid_optflow_pitch.kD(tuning_value);
        break;

#if HIL_MODE != HIL_MODE_ATTITUDE                                       // do not allow modifying _kp or _kp_yaw gains in HIL mode
    case CH6_AHRS_YAW_KP:
        ahrs._kp_yaw.set(tuning_value);
        break;

    case CH6_AHRS_KP:
        ahrs._kp.set(tuning_value);
        break;
#endif

    case CH6_INAV_TC:
        // To-Do: allowing tuning TC for xy and z separately
        inertial_nav.set_time_constant_xy(tuning_value);
        inertial_nav.set_time_constant_z(tuning_value);
        break;

    case CH6_DECLINATION:
        // set declination to +-20degrees
        compass.set_declination(ToRad((2.0f * g.rc_6.control_in - g.radio_tuning_high)/100.0f), false);     // 2nd parameter is false because we do not want to save to eeprom because this would have a performance impact
        break;

    case CH6_CIRCLE_RATE:
        // set circle rate
        g.circle_rate.set(g.rc_6.control_in/25-20);     // allow approximately 45 degree turn rate in either direction
        break;

    case CH6_SONAR_GAIN:
        // set sonar gain
        g.sonar_gain.set(tuning_value);
        break;

    case CH6_LOIT_SPEED:
        // set max loiter speed to 0 ~ 1000 cm/s
        wp_nav.set_loiter_velocity(g.rc_6.control_in);
        break;
    }
}

AP_HAL_MAIN();

