#include "Copter.h"

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool Copter::loiter_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {

		    	// fix loiter glitch
        if (!copter.failsafe.radio) {
            float target_roll, target_pitch;
            // apply SIMPLE mode transform to pilot inputs
            //update_simple_mode();
         // set target to current position	            // convert pilot input to lean angles
            get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());
             // process pilot's roll and pitch input
            loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);
        } else {
            // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
            loiter_nav->clear_pilot_desired_acceleration();
        }
		
        // set target to current position
    	loiter_nav->init_target();

        // initialize vertical speed and acceleration
        //pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        //pos_control->set_accel_z(g.pilot_accel_z);

        // initialise position and desired velocity
        if (!pos_control->is_active_z()) {
            pos_control->set_alt_target_to_current_alt();
            pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
        }

        return true;
    }else{
        return false;
    }
}

#if PRECISION_LANDING == ENABLED
bool Copter::do_precision_loiter()
{
    if (!_precision_loiter_enabled) {
        return false;
    }
    if (ap.land_complete_maybe) {
        return false;        // don't move on the ground
    }
    // if the pilot *really* wants to move the vehicle, let them....
    if (loiter_nav->get_pilot_desired_acceleration().length() > 50.0f) {
        return false;
    }
    if (!precland.target_acquired()) {
        return false; // we don't have a good vector
    }
    return true;
}

void Copter::precision_loiter_xy()
{
	loiter_nav->clear_pilot_desired_acceleration();
    Vector2f target_pos, target_vel_rel;
    if (!precland.get_target_position_cm(target_pos)) {
        target_pos.x = inertial_nav.get_position().x;
        target_pos.y = inertial_nav.get_position().y;
    }
    if (!precland.get_target_velocity_relative_cms(target_vel_rel)) {
        target_vel_rel.x = -inertial_nav.get_velocity().x;
        target_vel_rel.y = -inertial_nav.get_velocity().y;
    }
    pos_control->set_xy_target(target_pos.x, target_pos.y);
    pos_control->override_vehicle_velocity_xy(-target_vel_rel);
}
#endif

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void Copter::loiter_run()
{
    LoiterModeState loiter_state;

    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speed and acceleration
    // 下降速度可以单独设立一个参数，下降速度要比较慢
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);

    // process pilot inputs unless we are in radio failsafe
    if (!failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());
        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        // 下降速度当独设立一个参数
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
    	loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
    	loiter_nav->soften_for_landing();
    }

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors->rotor_runup_complete());
#else
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f) && ap.motor_spin_all;
#endif

    // Loiter State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        loiter_state = Loiter_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        loiter_state = Loiter_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        loiter_state = Loiter_Landed;
    } else {
        loiter_state = Loiter_Flying;
    }

    // Loiter State Machine
    switch (loiter_state) {

    case Loiter_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
#if FRAME_CONFIG == HELI_FRAME
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
#else
        loiter_nav->init_target();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        loiter_nav->update(ekfGndSpdLimit, ekfNavVelGainScaler);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate/*, get_smoothing_gain()*/);
        pos_control->update_z_controller();
        break;

    case Loiter_Takeoff:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }

        // get takeoff adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // run loiter controller
        loiter_nav->update(ekfGndSpdLimit, ekfNavVelGainScaler);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate/*, get_smoothing_gain()*/);

        // update altitude target and call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case Loiter_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }
        loiter_nav->init_target();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0/*, get_smoothing_gain()*/);
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case Loiter_Flying:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if PRECISION_LANDING == ENABLED
        if (do_precision_loiter()) {
            precision_loiter_xy();
        }
#endif

#if 1
        // run loiter controller
        loiter_nav->update(ekfGndSpdLimit, ekfNavVelGainScaler);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate/*, get_smoothing_gain()*/);

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update altitude target and call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
#else 
        // run loiter controller
        loiter_nav->update(ekfGndSpdLimit, ekfNavVelGainScaler);

			        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update altitude target and call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller_new();
		
		pos_control->thrustToAttitude(attitude_control->get_att_target_euler_rad().z);
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate/*, get_smoothing_gain()*/);


#endif
        break;
    }
}
