/*
 * control_position.cpp
 *
 *  Created on: 2018-6-1
 *      Author: liwh1
 *
 * 改自 px4 mc_pos_control.cpp
 *  RC mode where RPT sticks control speed in corresponding directions.
 *  Centered sticks level vehicle and hold it to fixed position and altitude against wind.
 *  Centered RPT sticks hold x, y, z position steady against any wind and levels attitude.
 *	Outside center:
 *		Roll/Pitch sticks control speed over ground in left-right and forward-back directions (respectively) relative to the "front" of the vehicle.
 *		Throttle stick controls speed of ascent-descent.
 *		Yaw stick controls rate of angular rotation above the horizontal plane.
 */
#include "Copter.h"

bool Copter::position_init(bool ignore_checks)
{
    // fail to initialise PosHold mode if no GPS lock
    if (!position_ok() && !ignore_checks) {
        return false;
    }

    // initialize vertical speeds and acceleration
    pos_control->set_speed_xy(wp_nav->get_speed_xy());
    pos_control->set_accel_xy(wp_nav->get_wp_acceleration());

    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    pos_control->init_vel_controller_xyz();

    return true;
}

void Copter::position_run(void)
{
	float vel_fw, vel_right;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;
    //float takeoff_climb_rate = 0.0f;

    //vector2f desired_vel;

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        loiter_nav->init_target();
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control->relax_alt_hold_controllers(0.0f);
        return;
    }


    // process pilot inputs unless we are in radio failsafe
    if (!failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        //get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());
        // process pilot's roll and pitch input
        //loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);
        get_pilot_desired_speed(vel_right, vel_fw, 500.0f);

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

    Vector3f desired_vel;
    // rotate acceleration vectors input to lat/lon frame
    desired_vel.x = (vel_fw * ahrs.cos_yaw() - vel_right * ahrs.sin_yaw());
    desired_vel.y = (vel_fw * ahrs.sin_yaw() + vel_right * ahrs.cos_yaw());
    desired_vel.z = target_climb_rate;


    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // set velocity to zero and stop rotating if no updates received for 3 seconds
    guided_set_desired_velocity_with_accel_and_fence_limits(desired_vel);


    // call velocity controller which includes z axis controller
    pos_control->update_vel_controller_xyz(ekfNavVelGainScaler);

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw_rate/*, get_smoothing_gain()*/);
    } else if (auto_yaw_mode == AUTO_YAW_RATE) {
        // roll & pitch from velocity controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), get_auto_yaw_rate_cds()/*, get_smoothing_gain()*/);
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), get_auto_heading(), true/*, get_smoothing_gain()*/);
    }
}





