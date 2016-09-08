/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Sub.h"


/*
 * control_althold.pde - init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Sub::althold_init(bool ignore_checks)
{
    // initialize vertical speeds and leash lengths
    // sets the maximum speed up and down returned by position controller
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    // stop takeoff if running
    takeoff_stop();

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Sub::althold_run()
{
    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    // This changes roll and pitch which is not user-controllable anyways, so forget this call.
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control.get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // === [VERIFIED]Added by Andreas
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "channel_yaw = %u",channel_yaw->get_control_in());
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "target_yaw_rate = %0.2f",target_yaw_rate);
    // remember to declare public
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "yaw_accel_limit = %f",attitude_control.get_accel_yaw_max_radss());
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "_rate_bf_ff_enabled = %d",attitude_control._rate_bf_ff_enabled);    // returns 1
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "_att_ctrl_use_accel_limit = %d",attitude_control._att_ctrl_use_accel_limit);    // returns 1
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "_accel_yaw_max = %f",attitude_control._accel_yaw_max);    // returns 0.0000
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "%f",attitude_control._p_angle_yaw.kP());    // returns 0.0000
    gcs_send_text_fmt(MAV_SEVERITY_INFO, "%f",AC_ATTITUDE_CONTROL_ANGLE_P);    // returns 6.0.
    
    // ====================

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

    //bool takeoff_triggered = (ap.land_complete && (channel_throttle->get_control_in() > get_takeoff_trigger_throttle()) && motors.spool_up_complete());

//    // Alt Hold State Machine Determination
    if(!ap.auto_armed) {
        althold_state = AltHold_NotAutoArmed;
//    if (!motors.armed() || !motors.get_interlock()) {
    //      althold_state = AltHold_MotorStopped;
    // } else if (!ap.auto_armed){
    //     althold_state = AltHold_NotAutoArmed;
//    } else if (takeoff_state.running || takeoff_triggered){
//        althold_state = AltHold_Takeoff;
//    } else if (ap.land_complete){
//        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:
    	// Multicopter do not stabilize roll/pitch/yaw when motor are stopped
        motors.set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->get_control_in())-throttle_average);
        break;

    case AltHold_NotAutoArmed:
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // Multicopters do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->get_control_in())-throttle_average);
        break;

    case AltHold_Takeoff:

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // set motors to full range
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // === Andreas:
        // Attitude controlleren sender ikke endelige kontrolsignaler til thrusterne. Dette sker
        // via "attitude_control.rate_controller_run()", som kaldes i ArduSub.cpp.
        // Bemærk at inputtet til nedenstående kald er input euler vinkler og euler rate.
        // Dette er IKKE nødvendigvis det samme som angular velocity (som er set fra BODY frame).
        // Nedenstående funktion udregner angular velocity target, som så bliver anvendt af rate controlleren.
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control.add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control.update_z_controller();
        break;

    case AltHold_Landed:
    	// Multicopter do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out(get_throttle_pre_takeoff(channel_throttle->get_control_in()),false,g.throttle_filt);
        // if throttle zero reset attitude and exit immediately
        if (ap.throttle_zero) {
            motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->get_control_in())-throttle_average);
        break;

    case AltHold_Flying:
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // call position controller
        if(ap.at_bottom) {
        	pos_control.relax_alt_hold_controllers(0.0); // clear velocity and position targets, and integrator
            pos_control.set_alt_target(inertial_nav.get_altitude() + 10.0f); // set target to 10 cm above bottom
        } else if(ap.at_surface) {
        	if(target_climb_rate < 0.0) { // Dive if the pilot wants to
            	pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        	} else if(pos_control.get_vel_target_z() > 0.0) {
        		pos_control.relax_alt_hold_controllers(0.0); // clear velocity and position targets, and integrator
        		pos_control.set_alt_target(g.surface_depth); // set alt target to the same depth that triggers the surface detector.
        	}
        } else {
        	pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        }

        pos_control.update_z_controller();
        break;
    }

    //control_in is range 0-1000
    //radio_in is raw pwm value
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
