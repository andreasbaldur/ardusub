/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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
 *  Adam M Rivera       :Auto Compass Declination
 *  Amilcar Lucas       :Camera mount library
 *  Andrew Tridgell     :General development, Mavlink Support
 *  Angel Fernandez     :Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid     :Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel	        :DCM, Libraries, Control law advice
 *  Emile Castelnuovo   :VRBrain port, bug fixes
 *  Gregory Fletcher    :Camera mount orientation math
 *  Guntars             :Arming safety suggestion
 *  HappyKillmore       :Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle         :Alpha testing
 *  James Goppert       :Mavlink Support
 *  Jani Hiriven        :Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland	:PPM Encoder
 *  Jose Julio          :Stabilization Control laws, MPU6k driver
 *  Julien Dubois       :PosHold flight mode
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine          :Tri Support, Graphics
 *  Leonard Hall        :Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini     :Lead tester
 *  Michael Oborne      :Mission Planner GCS
 *  Mike Smith          :Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraction Layer (HAL)
 *  Robert Lefebvre     :Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  Sandro Tognana      :PosHold flight mode
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/diydrones/ardupilot/graphs/contributors
 *  Wiki: http://copter.ardupilot.org/
 *  Requires modified version of Arduino, which can be found here: http://ardupilot.com/downloads/?category=6
 *
 */

#include "Sub.h"

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Sub, &sub, func, rate_hz, max_time_micros)

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Sub::scheduler_tasks[] = {
    SCHED_TASK(rc_loop,              100,    130),
    SCHED_TASK(throttle_loop,         50,     75),
    SCHED_TASK(update_GPS,            50,    200),
#if OPTFLOW == ENABLED
    SCHED_TASK(update_optical_flow,  200,    160),
#endif
    SCHED_TASK(update_batt_compass,   10,    120),
    SCHED_TASK(read_aux_switches,     10,     50),
    SCHED_TASK(arm_motors_check,      10,     50),
    SCHED_TASK(auto_disarm_check,     10,     50),
    SCHED_TASK(auto_trim,             10,     75),
	SCHED_TASK(read_rangefinder,      20,    100),
	SCHED_TASK(update_altitude,       10,    100),
    SCHED_TASK(run_nav_updates,       50,    100),
    SCHED_TASK(update_thr_average,   100,     90),  // this updates the throttle hover level
    SCHED_TASK(three_hz_loop,          3,     75),
	SCHED_TASK(update_turn_counter,   10,     50),
    SCHED_TASK(compass_accumulate,   100,    100),
    SCHED_TASK(barometer_accumulate,  50,     90),
#if PRECISION_LANDING == ENABLED
    SCHED_TASK(update_precland,       50,     50),
#endif
    SCHED_TASK(update_notify,         50,     90),
    SCHED_TASK(one_hz_loop,            1,    100),
    SCHED_TASK(ekf_check,             10,     75),
    SCHED_TASK(landinggear_update,    10,     75),
    SCHED_TASK(lost_vehicle_check,    10,     50),
    SCHED_TASK(gcs_check_input,      400,    180),
    SCHED_TASK(gcs_send_heartbeat,     1,    110),
    SCHED_TASK(gcs_send_deferred,     50,    550),
    SCHED_TASK(gcs_data_stream_send,  50,    550),
    SCHED_TASK(update_mount,          50,     75),
	SCHED_TASK(update_trigger,        50,     75),
    SCHED_TASK(ten_hz_logging_loop,   10,    350),
    SCHED_TASK(twentyfive_hz_logging, 25,    110),
    SCHED_TASK(dataflash_periodic,    400,    300),
    SCHED_TASK(perf_update,           0.1,    75),
    SCHED_TASK(read_receiver_rssi,    10,     75),
    SCHED_TASK(rpm_update,            10,    200),
    SCHED_TASK(compass_cal_update,   100,    100),
    SCHED_TASK(accel_cal_update,      10,    100),
#if FRSKY_TELEM_ENABLED == ENABLED
    SCHED_TASK(frsky_telemetry_send,   5,     75),
#endif
	SCHED_TASK(terrain_update,        10,    100),
#if EPM_ENABLED == ENABLED
    SCHED_TASK(epm_update,            10,     75),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,     3.3,    75),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,   75),
#endif
    SCHED_TASK(baldurLoop, 0.5,   200)
};

/*
  Created by: Andreas
  This loop is devoted for test purposes.
  ---
  Frequency: 1 Hz
 */


int timeSec = 0;
float unit_step_input = 0.0;
void Sub::baldurLoop()
{
/*    gcs_send_text_fmt(MAV_SEVERITY_INFO, "t = %d sec.",timeSec);

    // Apply step input
    if ( timeSec++ >= 5 )
    {
        unit_step_input = 1.0;
    }
    gcs_send_text_fmt(MAV_SEVERITY_INFO, "u = %0.2f",unit_step_input);

    _baldurs_pid.set_input_filter_all(unit_step_input);
    float proportional_output = _baldurs_pid.get_p();
    float integrator_output = _baldurs_pid.get_i();*/
/*    gcs_send_text_fmt(MAV_SEVERITY_INFO, "p: %0.5f",proportional_output);
    gcs_send_text_fmt(MAV_SEVERITY_INFO, "i: %0.5f",integrator_output);
    gcs_send_text_fmt(MAV_SEVERITY_INFO, "acro_yaw_p: %f",g.acro_yaw_p);*/
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "yaw_max_radss: %f",AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_CDSS);

    // ===========================
    // Depth Controller Parameters

    // ===========================

    // ===========================
    // Depth Controller Parameters

    // Position Kp gain
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "g.p_alt_hold.kP() = %f",g.p_alt_hold.kP());

    // Velocity Kp gain
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "_p_vel_z.kP() = %f",pos_control._p_vel_z.kP());

    // Acceleration PID gains
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "_pid_accel_z.kP() = %f",pos_control._pid_accel_z.kP());
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "_pid_accel_z.kI() = %f",pos_control._pid_accel_z.kI());
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "_pid_accel_z.kD() = %f",pos_control._pid_accel_z.kD());
    
    // ---
    
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "ALT_HOLD_P = %f",(float)ALT_HOLD_P); // = 1.0
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "VEL_Z_P = %f",(float)VEL_Z_P);       // = 5.0
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "ACCEL_Z_P = %f",(float)ACCEL_Z_P);   // = 0.5
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "ACCEL_Z_I = %f",(float)ACCEL_Z_I);   // = 1.0
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "ACCEL_Z_D = %f",(float)ACCEL_Z_D);   // = 0.0
    
    // ---
    
    #define PRINT_SENSOR_STATUS 0
    if (PRINT_SENSOR_STATUS)
    {
        // Attitude PID parameters
        gcs_send_text(MAV_SEVERITY_INFO,"---");
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "rangefinder_alt_ok() = %d",rangefinder_alt_ok());  // 0
    }

    // ===========================

    #define PRINT_ALTITUDE_PARAMS 0
    if (PRINT_ALTITUDE_PARAMS)
    {
        gcs_send_text(MAV_SEVERITY_INFO,"---");
        gcs_send_text(MAV_SEVERITY_INFO,"Altitude (depth):");
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Accel. kP = %f",(float)(g.pid_accel_z.kP())); // 1.0
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Accel. kI = %f",(float)(g.pid_accel_z.kI())); // 3.0
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Accel. kD = %f",(float)(g.pid_accel_z.kD())); // 0.0
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Accel. Imax = %f",(float)(g.pid_accel_z.imax()));   
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Pos. kP = %f",(float)(g.p_alt_hold.kP()));    // 3.0
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Vel. kP = %f",(float)(g.p_vel_z.kP()));       // 8.0
        gcs_send_text(MAV_SEVERITY_INFO,"---");
        gcs_send_text(MAV_SEVERITY_INFO,"Max/min limits:");
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_throttle_hover = %f",(float)(pos_control._throttle_hover));      //    0.5: estimated throttle required to maintain a level hover
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_speed_down_cms = %f",(float)(pos_control._speed_down_cms));      // -150.0: max descent rate in cm/s
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_speed_up_cms = %f",(float)(pos_control._speed_up_cms));          //  250.0: max climb rate in cm/s 
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_speed_cms = %f",(float)(pos_control._speed_cms));                //  500.0: max horizontal speed in cm/s
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_accel_z_cms = %f",(float)(pos_control._accel_z_cms));            //  250.0: max vertical acceleration in cm/s/s
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_accel_last_z_cms = %f",(float)(pos_control._accel_last_z_cms));  //    0.0: max vertical acceleration in cm/s/s
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_accel_cms = %f",(float)(pos_control._accel_cms));                //  100.0: max horizontal acceleration in cm/s/s
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_jerk_cmsss = %f",(float)(pos_control._jerk_cmsss));              // 1700.0: max horizontal jerk in cm/s/s/s         
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_alt_max = %f",(float)(pos_control._alt_max));              // 1700.0: max horizontal jerk in cm/s/s/s         
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_alt_min = %f",(float)(pos_control._alt_min));              // 1700.0: max horizontal jerk in cm/s/s/s         
        gcs_send_text(MAV_SEVERITY_INFO,"---");
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_vel_error_filter = %f",(float)(pos_control._vel_error_filter.get_cutoff_freq())); // 4.0
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "POSCONTROL_THROTTLE_CUTOFF_FREQ = %f",(float)(POSCONTROL_THROTTLE_CUTOFF_FREQ)); // 2.0
    }

    #define PRINT_POSZ_VEL_CONTROL 0
    if (PRINT_POSZ_VEL_CONTROL)
    {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_flags.freeze_ff_z = %d",pos_control._flags.freeze_ff_z); // 1 før controlleren startes, men 0 efter.
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_accel_feedforward.z = %f",(float)(pos_control._accel_feedforward.z)); // acceleration feedforward nonzero under kørsel
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_flags.use_desvel_ff_z = %d",pos_control._flags.use_desvel_ff_z);
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_p_pos_z.kP() = %f",(float)(pos_control._p_pos_z.kP()));
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_p_vel_z.kP() = %f",(float)(pos_control._p_vel_z.kP()));
        
    }

    
    #define PRINT_THROTTLE 0
    if (PRINT_THROTTLE)
    {
        attitude_control.set_throttle_out(1, true, 4);
        gcs_send_text(MAV_SEVERITY_INFO,"---");
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "get_throttle_bidirectional = %f",(float)(motors.get_throttle_bidirectional())); 
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_throttle_in = %f",(float)(motors._throttle_in)); 

    }

    #define PRINT_ALTITUDE_SIGNALS 0
    if (PRINT_ALTITUDE_SIGNALS)
    {
        gcs_send_text(MAV_SEVERITY_INFO,"---");
        gcs_send_text(MAV_SEVERITY_INFO,"Altitude signals");
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "_vel_desired = %f",(float)(pos_control._vel_desired.z)); 
    }

    // ===========================
    
    #define PRINT_ATTITUDE_PARAMS 0
    if (PRINT_ATTITUDE_PARAMS)
    {
        // Attitude PID parameters
        gcs_send_text(MAV_SEVERITY_INFO,"---");
        gcs_send_text(MAV_SEVERITY_INFO,"Yaw Attitude");
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Kp = %f",(float)(attitude_control._p_angle_yaw.kP()));  // 0.0 (feedforward disabled)
    }
    
    // ===========================

    #define PRINT_YAW_RATE_PARAMS 0
    if (PRINT_YAW_RATE_PARAMS)
    {
        // Yaw rate controllers
        gcs_send_text(MAV_SEVERITY_INFO,"---");
        gcs_send_text(MAV_SEVERITY_INFO,"Yaw Rate:");
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "kP = %f",(float)(attitude_control.get_rate_yaw_pid().kP()));
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "kI = %f",(float)(attitude_control.get_rate_yaw_pid().kI()));
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "kD = %f",(float)(attitude_control.get_rate_yaw_pid().kD()));
    }

    // ===========================

    #define PRINT_AHRS_PARAMS 0
    if (PRINT_AHRS_PARAMS)
    {
        // Yaw rate controllers
        gcs_send_text(MAV_SEVERITY_INFO,"---");
        gcs_send_text(MAV_SEVERITY_INFO,"AHRS:");
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "ahrs._kp_yaw = %f",(float)(ahrs._kp_yaw));
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "ahrs.get_gyro_drift() = %f",(float)(ahrs.get_gyro_drift().z));
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "AP_AHRS_WITH_EKF1 = %d",AP_AHRS_WITH_EKF1);
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "active_EKF_type() = %d",(int)(ahrs.active_EKF_type()));   // = 2, enum EKF_TYPE {..., EKF_TYPE2=2, ...}
    }

    // ===========================
    // This is used by the depth controller
    #define PRINT_INERTIAL_NAVIGATION 0
    #if PRINT_INERTIAL_NAVIGATION
    gcs_send_text_fmt(MAV_SEVERITY_INFO, "   w = %f",inertial_nav.get_velocity().z);
    gcs_send_text_fmt(MAV_SEVERITY_INFO, "wdot = %f",-(ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f);
    #endif
    
}

void Sub::setup()
{
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    // setup storage layout for copter
    StorageManager::set_layout_copter();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));

    // setup initial performance counters
    perf_info_reset();
    fast_loopTimer = AP_HAL::micros();
}

/*
  if the compass is enabled then try to accumulate a reading
 */
void Sub::compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }
}

/*
  try to accumulate a baro reading
 */
void Sub::barometer_accumulate(void)
{
    barometer.accumulate();
}

void Sub::perf_update(void)
{
    if (should_log(MASK_LOG_PM))
        Log_Write_Performance();
    if (scheduler.debug()) {
        gcs_send_text_fmt(MAV_SEVERITY_WARNING, "PERF: %u/%u %lu %lu\n",
                          (unsigned)perf_info_get_num_long_running(),
                          (unsigned)perf_info_get_num_loops(),
                          (unsigned long)perf_info_get_max_time(),
                          (unsigned long)perf_info_get_min_time());
    }
    perf_info_reset();
    pmTest1 = 0;
}

void Sub::loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    uint32_t timer = micros();

    // check loop time
    perf_info_check_loop_time(timer - fast_loopTimer);

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.0f;
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
    uint32_t time_available = (timer + MAIN_LOOP_MICROS) - micros();
    scheduler.run(time_available);
}


// Main loop - 400hz
void Sub::fast_loop()
{

    // IMU DCM Algorithm
    // --------------------
    read_AHRS();

    // run low level rate controllers that only require IMU data
    switch(control_mode)
    {
        case MANUAL:
        case MANUAL_RAW:
            // no roll/pitch/yaw stabilization
            break;
        case STABILIZE_RP:
        case ALT_HOLD_RP:
            // only roll/pitch stabilization
            attitude_control.roll_pitch_rate_controller_run();
            break;
        default:
            // both roll/pitch and yaw stabilization
            attitude_control.rate_controller_run();
            break;
    }

    // send outputs to the motors library
    motors_output();

    // Inertial Nav
    // --------------------
    read_inertia();

    // check if ekf has reset target heading
    check_ekf_yaw_reset();

    // run the attitude controllers
    update_flight_mode();

    // update home from EKF if necessary
    update_home_from_EKF();

    // check if we've landed or crashed
    update_land_and_crash_detectors();

    // check if we've reached the surface or bottom
    update_surface_and_bottom_detector();

    // camera mount's fast update
    camera_mount.update_fast();

    // log sensor health
    if (should_log(MASK_LOG_ANY)) {
        Log_Sensor_Health();
    }
}

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
void Sub::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
void Sub::throttle_loop()
{
    // update throttle_low_comp value (controls priority of throttle vs attitude control)
    update_throttle_thr_mix();

    // check auto_armed status
    update_auto_armed();

#if GNDEFFECT_COMPENSATION == ENABLED
    update_ground_effect_detector();
#endif // GNDEFFECT_COMPENSATION == ENABLED
}

// update_mount - update camera mount position
// should be run at 50hz
void Sub::update_mount()
{
#if MOUNT == ENABLED
    // update camera mount's position
    camera_mount.update();
#endif
}


// update camera trigger
void Sub::update_trigger(void)
{
 #if CAMERA == ENABLED
	camera.trigger_pic_cleanup();
	if (camera.check_trigger_pin()) {
		gcs_send_message(MSG_CAMERA_FEEDBACK);
		if (should_log(MASK_LOG_CAMERA)) {
			DataFlash.Log_Write_Camera(ahrs, gps, current_loc);
		}
   }
#endif
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Sub::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    read_battery();

    if(g.compass_enabled) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors.get_throttle()/1000.0f);
        compass.read();
        // log compass information
        if (should_log(MASK_LOG_COMPASS) && !ahrs.have_ekf_logging()) {
            DataFlash.Log_Write_Compass(compass);
        }
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Sub::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        DataFlash.Log_Write_Rate(ahrs, motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            DataFlash.Log_Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info() );
            DataFlash.Log_Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info() );
            DataFlash.Log_Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info() );
            DataFlash.Log_Write_PID(LOG_PIDA_MSG, g.pid_accel_z.get_pid_info() );
        }
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        Log_Write_MotBatt();
    }
    if (should_log(MASK_LOG_RCIN)) {
        DataFlash.Log_Write_RCIN();
        if (rssi.enabled()) {
            DataFlash.Log_Write_RSSI(rssi);
        }
    }
    if (should_log(MASK_LOG_RCOUT)) {
        DataFlash.Log_Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (mode_requires_GPS(control_mode) || landing_with_GPS())) {
        Log_Write_Nav_Tuning();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_Vibration(ins);
    }
}

// twentyfive_hz_logging_loop
// should be run at 25hz
void Sub::twentyfive_hz_logging()
{
#if HIL_MODE != HIL_MODE_DISABLED
    // HIL for a copter needs very fast update of the servo values
    gcs_send_message(MSG_RADIO_OUT);
#endif

#if HIL_MODE == HIL_MODE_DISABLED
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        DataFlash.Log_Write_Rate(ahrs, motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            DataFlash.Log_Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info() );
            DataFlash.Log_Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info() );
            DataFlash.Log_Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info() );
            DataFlash.Log_Write_PID(LOG_PIDA_MSG, g.pid_accel_z.get_pid_info() );
        }
    }

    // log IMU data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_IMU) && should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_IMU(ins);
    }
#endif
}

void Sub::dataflash_periodic(void)
{
    DataFlash.periodic_tasks();
}

// three_hz_loop - 3.3hz loop
void Sub::three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

#if AC_FENCE == ENABLED
    // check if we have breached a fence
    fence_check();
#endif // AC_FENCE_ENABLED

#if SPRAYER == ENABLED
    sprayer.update();
#endif

    update_events();

    // update ch6 in flight tuning
    tuning();
}

// one_hz_loop - runs at 1Hz
void Sub::one_hz_loop()
{
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    update_arming_checks();

    if (!motors.armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.set_orientation();

        update_using_interlock();

        // check the user hasn't updated the frame orientation
        motors.set_frame_orientation(g.frame_orientation);

        // set all throttle channel settings
        motors.set_throttle_range(g.throttle_min, channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
        // set hover throttle
        motors.set_hover_throttle(g.throttle_mid);
    }

    // update assigned functions and enable auxiliary servos
    RC_Channel_aux::enable_aux_servos();

    check_usb_mux();

    // update position controller alt limits
    update_poscon_alt_max();

    // enable/disable raw gyro/accel logging
    ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));

    // log terrain data
    terrain_logging();
}

// called at 50hz
void Sub::update_GPS(void)
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];   // time of last gps message
    bool gps_updated = false;

    gps.update();

    // log after every gps message
    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);

            // log GPS message
            if (should_log(MASK_LOG_GPS) && !ahrs.have_ekf_logging()) {
                DataFlash.Log_Write_GPS(gps, i, current_loc.alt);
            }

            gps_updated = true;
        }
    }

    if (gps_updated) {
        // set system time if necessary
        set_system_time_from_GPS();

        // checks to initialise home and take location based pictures
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {

#if CAMERA == ENABLED
            if (camera.update_location(current_loc, sub.ahrs) == true) {
                do_take_picture();
            }
#endif
        }
    }
}

void Sub::init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

    // log the simple bearing to dataflash
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
}

// update_simple_mode - rotates pilot input if we are in simple mode
void Sub::update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (ap.simple_mode == 0 || !ap.new_radio_frame) {
        return;
    }

    // mark radio frame as consumed
    ap.new_radio_frame = false;

    if (ap.simple_mode == 1) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = channel_roll->get_control_in()*simple_cos_yaw - channel_pitch->get_control_in()*simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*simple_sin_yaw + channel_pitch->get_control_in()*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = channel_roll->get_control_in()*super_simple_cos_yaw - channel_pitch->get_control_in()*super_simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*super_simple_sin_yaw + channel_pitch->get_control_in()*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    channel_roll->set_control_in(rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw());
    channel_pitch->set_control_in(-rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw());
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void Sub::update_super_simple_bearing(bool force_update)
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

void Sub::read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before ahrs update
    gcs_check_input();
#endif

    ahrs.update();
}

// read baro and rangefinder altitude at 10hz
void Sub::update_altitude()
{
    // read in baro altitude
    read_barometer();

    // write altitude info to dataflash logs
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
    }
}

AP_HAL_MAIN_CALLBACKS(&sub);
