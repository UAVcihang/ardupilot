/*
 * control_zigzag.cpp
 *
 *  Created on: 2018-1-15
 *      Author: liwh1
 */
#include "Copter.h"



// zigzag_init - initialise zigzag controller
bool Copter::zigzag_init(bool ignore_checks)
{
	if(!motors->armed()) {
		return false;
	}
	//char buf[128];
    if (position_ok() || ignore_checks) {


        //zigzag_auto_complete_state = false;
        // initialise waypoint state
    	zigzag_change_yaw = false;
        if(zigzag_waypoint_state.b_hasbeen_defined || zigzag_waypoint_state.bp_mode != Zigzag_None) {

        	//get_bearing_cd(_APoint, _BPoint) * 0.01f;
        	zigzag_bearing = get_bearing_cd(zigzag_waypoint_state.a_pos, zigzag_waypoint_state.b_pos);
        	zigzag_mode = Zigzag_Auto;
        	zigzag_auto_complete_state = (zigzag_waypoint_state.bp_mode != Zigzag_None);

        	Vector3f v_A2B  = zigzag_waypoint_state.vB_pos - zigzag_waypoint_state.vA_pos;
        	zigzag_dist = v_A2B.length();
        	//if()
        	// zigzag_waypoint_state.area = (zigzag_waypoint_state.bp_mode == Zigzag_None)?0:zigzag_waypoint_state.area;

        	zigzag_waypoint_state.flag = zigzag_waypoint_state.flag << 1;
        	zigzag_waypoint_state.flag += 1;
			
			//sprintf(buf, "Zigzag auto %x %x", zigzag_waypoint_state.flag & 0x05, zigzag_waypoint_state.flag);
			GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "Zigzag auto");
        	//zigzag_waypoint_state.action &= true;
        	/*if(zigzag_waypoint_state.last_exit_time_ms != 0 && ((millis() - zigzag_waypoint_state.last_exit_time_ms) < 3000))
        	{
        		zigzag_waypoint_state.action = true;
        		zigzag_waypoint_state.index++;
        	}
        	else {
        		zigzag_waypoint_state.action = false;
        	}*/
        	set_auto_yaw_mode(AUTO_YAW_HOLD);
        	wp_nav->wp_and_spline_init();
        	wp_nav->set_wp_destination(current_loc);
        	/*if(zigzag_auto_complete_state) {
                zigzag_set_destination();
                zigzag_auto_complete_state = false;
        	}
        	else{
        	wp_nav->set_wp_destination(current_loc);
        	}*/

        }
        else {
        	GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "Zigzag manual");
        	zigzag_mode = Zigzag_Manual;
        	zigzag_auto_complete_state = false;

        	zigzag_waypoint_state.area = 0.0f;
#if 1
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
            // initialize's loiter position and velocity on xy-axes from current pos and velocity
            loiter_nav->init_target();

            // initialize vertical speed and acceleration's range
            //pos_control->set_speed_z(-g.k_param_pilot_speed_up, g.k_param_pilot_speed_up);
            // the parameters are maximum climb and descent rates
            //pos_control->set_accel_z(g.pilot_accel_z);
            //pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
            //pos_control->set_accel_z(g.pilot_accel_z);

            // Global parameters are all contained within the 'g' class.
            // Parameters g;

            // initialise position_z and desired velocity_z
            if (!pos_control->is_active_z()) {
                // is_active_z - returns true if the z-axis position controller has been run very recently
                pos_control->set_alt_target_to_current_alt();
                pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
            }
#else
            poshold_init(true);
#endif
        }

        //hal.console->printf("zigzag mode init");
        //delay(2000);
		switch(zigzag_waypoint_state.direct) {

			case 1:
				zigzag_rc_state = RC_LEFT;
				break;
			case -1:
				zigzag_rc_state = RC_RIGHT;
				break;
			case 0:
			default:
				zigzag_rc_state = RC_MID;
				break;
		}
        // zigzag_waypoint_state.width = 500;
        //zigzag_waypoint_state.index = 0;
        /*zigzag_waypoint_state.A_hasbeen_defined = false;
        zigzag_waypoint_state.B_hasbeen_defined = false;
        in_zigzag_manual_control = true;
        zigzag_is_between_A_and_B = false;
        zigzag_judge_moving.is_keeping_time = false;*/


        return true;
    }
    else {
        return false;
    }
}

void Copter::zigzag_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0/*, get_smoothing_gain()*/);
        attitude_control->set_throttle_out(0, false, g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        loiter_nav->init_target();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero

        loiter_nav->update(ekfGndSpdLimit, ekfNavVelGainScaler);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), 0/*, get_smoothing_gain()*/);
        pos_control->update_z_controller();
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        // attitude_control->set_throttle_out_unstabilized(0, true, g.throttle_filt);

#endif
        return;
    }

    switch(zigzag_mode) {

    case Zigzag_Manual:
        // receive pilot's inputs, do position and attitude control
    	//GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "Zigzag manual");
#if 1
        zigzag_manual_control();
#else
        poshold_run();
#endif
    	break;

    case Zigzag_Auto:
    	if(zigzag_auto_complete_state && !zigzag_change_yaw/* && zigzag_waypoint_state.action*/){
    		//Vector3f next_dest;
    		//Location_Class next_dest;

    		//zigzag_calculate_next_dest(next_dest);
            // initialise waypoint and spline controller
    		//GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "Zigzag set point");
    		//do{
            wp_nav->wp_and_spline_init();
            zigzag_set_destination();
			//wp_nav->set_fast_waypoint(true);
            zigzag_auto_complete_state = false;
            wp_nav->set_fast_waypoint(true);
    		//}while(zigzag_waypoint_state.action);
    	}
    	else{
    		//GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "Zigzag auto run");
    		zigzag_auto_control();
    	}
    	break;
    }
}

//zigzag_manual_control - process manual control
void Copter::zigzag_manual_control()
{
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;
    float target_roll = 0.0f;
    float target_pitch = 0.0f;

    // initialize vertical speed and acceleration's range
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
        //make sure the climb rate is in the given range, prevent floating point errors
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    }
    else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we
        //do not switch to RTL for some reason
    	loiter_nav->clear_pilot_desired_acceleration();
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    // run loiter controller
    loiter_nav->update(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(),
    		loiter_nav->get_pitch(), target_yaw_rate/*, get_smoothing_gain()*/);

    // adjust climb rate using rangefinder
    if (rangefinder_alt_ok()) {
        // if rangefinder is ok, use surface tracking
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
    }

    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    //adjusts target up or down using a climb rate

    pos_control->update_z_controller();
}


// zigzag_setDirect set ab point run direction; -1:body left, 1: body right
/*void Copter::zigzag_setDirect()
{
    if (!failsafe.radio) {
    	get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);
    }
}*/

/* 1、方向不可控， 油门 俯仰 横滚 均可控
 * 2、控制俯仰 横滚退出作业模式
 * 3、控制油门可控制飞行高度，但不退出作业模式 */
// zigzag_auto auto run
void Copter::zigzag_auto_control()
{
	static uint32_t last_of_update = 0;
    // process pilot's yaw input
	float target_roll = 0, target_pitch = 0;
    float target_yaw_rate = 0;
    float target_climb_rate = 0.0f;

    //pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    //pos_control->set_accel_z(g.pilot_accel_z);
    if (!failsafe.radio) {
        // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    	get_pilot_desired_lean_angles(target_roll, target_pitch, aparm.angle_max, attitude_control->get_althold_lean_angle_max());
        //get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);  //use pilot's yaw input to control attitude
        }

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    }


    // zigzag mode direction
    if(zigzag_waypoint_state.direct == 0) {
    	switch(zigzag_rc_state){
    	case RC_MID:
    		if(target_roll > aparm.angle_max / 2)
    			zigzag_rc_state = RC_RIGHT;
    		else if(target_roll < -aparm.angle_max / 2)
    			zigzag_rc_state = RC_LEFT;
    		break;
    	case RC_RIGHT:
    		// 浮点型 判断0 经常进入不了
    		if(/*is_zero(target_roll)*/ (fabsf(target_roll) < aparm.angle_max / 15.0f)) {
    			//mission.set_ab_direction(AP_Mission::AB_RIGHT);
    			zigzag_waypoint_state.direct = -1;
    			//zigzag_waypoint_state.action = true;
    			//abpoint_state = ABPoint_Auto;
    		}

    		break;
    	case RC_LEFT:
    		if(/*is_zero(target_roll)*/(fabsf(target_roll) < aparm.angle_max / 15.0f)) {
    			//mission.set_ab_direction(AP_Mission::AB_LEFT);
    			zigzag_waypoint_state.direct = 1;
    			//zigzag_waypoint_state.action = true;
    			//abpoint_state = ABPoint_Auto;
    		}
    		break;
    	}
    	if(zigzag_waypoint_state.direct != 0) {
    		zigzag_auto_complete_state = true;
    		zigzag_waypoint_state.flag = 0x05;
    		//zigzag_waypoint_state.action = true;
    		return;
    	}
    }
    else if(fabsf(target_roll) > aparm.angle_max / 15.0f || fabsf(target_pitch) > aparm.angle_max / 15.0f/*!is_zero(target_roll) || !is_zero(target_pitch)*/ /*|| !is_zero(target_yaw_rate)*/){
    	zigzag_mode = Zigzag_Manual;
    	zigzag_auto_complete_state = false;
#if 1
        loiter_nav->init_target();
#else
        poshold_init(true);
#endif
        zigzag_auto_stop();
    	return;
    }
    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // u-turn
    //if(zigzag_waypoint_state.bp_mode == Zigzag_None && zigzag_waypoint_state.index >1 && (zigzag_waypoint_state.index %2 != 0)){
        // run circle controller
    //    circle_nav->update();
    //}else{
    	// fly line
    // run waypoint controller to update xy
    failsafe_terrain_set_status(wp_nav->update_zigzag_wpnav());
    //}

    // adjust climb rate using rangefinder
    if (rangefinder_alt_ok()) {
        // if rangefinder is ok, use surface tracking
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
    }

    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

    //hal.console->printf("taget climb rate:%f\n", target_climb_rate);
    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();


    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), /*target_yaw_rate*/0/*, get_smoothing_gain()*/);
    	attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), wp_nav->get_yaw(), true/*, get_smoothing_gain()*/);
    }
    else if (auto_yaw_mode == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_yaw_rate_cds()/*, get_smoothing_gain()*/);
    }
    else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(), true/*, get_smoothing_gain()*/);
    }

    uint32_t now = AP_HAL::millis();
    if((now - last_of_update) > 1000 && (zigzag_waypoint_state.index % 2) == 0 && zigzag_waypoint_state.index!=0 ){
    	last_of_update = now;
    	zigzag_waypoint_state.area =  (zigzag_dist * zigzag_waypoint_state.index - wp_nav->get_track_covered_zigzag()) * zigzag_waypoint_state.width * 0.0001 * 0.5;
    }

    if(zigzag_waypoint_state.direct != 0 && ((zigzag_waypoint_state.flag & 0x05) == 0x05)) {
        //if(zigzag_waypoint_state.bp_mode == Zigzag_None && zigzag_waypoint_state.index >1 && (zigzag_waypoint_state.index %2 != 0)){
        //	zigzag_auto_complete_state = (circle_nav->get_angle_total() >= M_PI);
       // }else{
        	zigzag_auto_complete_state = wp_nav->reached_wp_destination();
        //}
    if(zigzag_change_yaw && verify_yaw()) {
    	//verify_yaw();
    	zigzag_change_yaw = false;
    }
    if(zigzag_auto_complete_state && zigzag_waypoint_state.bp_mode != Zigzag_None){
    	zigzag_change_yaw = true;
    	set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
    	/*set_auto_yaw_look_at_heading(zigzag_bearing,
    			cmd.content.yaw.turn_rate_dps,
    			cmd.content.yaw.direction,
    			cmd.content.yaw.relative_angle > 0);*/
    	yaw_look_at_heading = zigzag_bearing;
		zigzag_waypoint_state.bp_mode = Zigzag_None;
		zigzag_waypoint_state.index--;
    }

	}
}


void Copter::zigzag_set_destination(/*Location_Class& next_dest*/)
{

	Vector3f next;
	Vector3f center;
	Vector3f cur_pos;
	Vector3f v_BP2Next;
	Vector3f v_BP2Next_uint;
	Vector3f v_BP2Cur;
	Vector3f v_BP2E;
	float track_length;
	float dotproduct;
	//char buf[128];

	switch(zigzag_waypoint_state.bp_mode)
	{
	case Zigzag_None:
		/*Vector3f v_A2B  = zigzag_waypoint_state.vB_pos - zigzag_waypoint_state.vA_pos;
		v_A2B.z = 0;
		// A(x1,y1),B(x2,y2),设下一个点为N,其坐标为(x,y)，AB(x2-x1, y2-y1)垂直于BN(x-x2, y-y2)，则v_A2B * v_B2N = 0, 且|B2N| = width， |v_A2B x v_B2N| = |v_A2B| * |v_B2N|
		// a1 * x + b1 * y = c1   a2 * x + b2 * y = c2
		// 根据高斯消元法 可以求出坐标(x,y)
		float dist_AB = v_A2B.length();
		float a1 = v_A2B.x;
		float b1 = v_A2B.y;
		float c1 = v_A2B.x * zigzag_waypoint_state.vB_pos.x + v_A2B.y * zigzag_waypoint_state.vB_pos.y;
		float a2=0, b2=0, c2 = 0;
		zigzag_waypoint_state.index++;
		switch(zigzag_waypoint_state.index%4)
		{
		case 1:
		case 0:
			c2 = zigzag_waypoint_state.direct * dist_AB * zigzag_waypoint_state.width * ((zigzag_waypoint_state.index+1)>>1);
			c2 = c2 + zigzag_waypoint_state.vB_pos.x * v_A2B.y - zigzag_waypoint_state.vB_pos.y * v_A2B.x;
			a2 = v_A2B.y;
			b2 = -v_A2B.x;
			break;
		case 2:
		case 3:
			c2 = zigzag_waypoint_state.direct * dist_AB * zigzag_waypoint_state.width * ((zigzag_waypoint_state.index+1)>>1);
			c2 = c2 + zigzag_waypoint_state.vA_pos.x * v_A2B.y - zigzag_waypoint_state.vA_pos.y * v_A2B.x;
			a2 = v_A2B.y;
			b2 = -v_A2B.x;
			break;
		}

		Vector3f next;
		float denominator = (a1 * b2 - a2 * b1);
		//float denominatorY = a2 * b1 - a1 * b2;
		// 如果 dist_AB 不等于0， 则 denominator 不可能为0
		if(is_zero(denominator)){
			next.x = (c1 * b2 - c2 * b1) / denominator;
			next.y = -(c1 * a2 - c2 * a1) / denominator;
		}
		next.z = inertial_nav.get_position().z;*/
		zigzag_waypoint_state.index++;
		//Vector3f next;
		zigzag_calculate_next_dest(next, zigzag_waypoint_state.index);
		//sprintf(buf, "zigzag Bx:%.2f By:%.2f next x:%.2f  y:%.2f",  zigzag_waypoint_state.vB_pos.x, zigzag_waypoint_state.vB_pos.y, next.x, next.y);
		//GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, buf);
		// 从第一个点开始走U形,并调整机头，机头朝向与飞行方向一致
		if(/*zigzag_waypoint_state.index>1 && */(zigzag_waypoint_state.fly_direc < Zigzag_A2B )){
			Vector3f ziazag_point_prev = pos_control->get_pos_target();
			center.x  = (ziazag_point_prev.x + next.x) * 0.5f;
			center.y  = (ziazag_point_prev.y + next.y) * 0.5f;
			center.z  = next.z;

			//int8_t sign = (zigzag_bearing<18000)?-1:1;
			bool pi_flag = (zigzag_waypoint_state.direct > 0)?true:false;
			bool cw_flag;
			if(zigzag_waypoint_state.direct > 0){
				cw_flag = (zigzag_waypoint_state.fly_direc == Zigzag_A2A)?true:false;
			}
			else{
				cw_flag = (zigzag_waypoint_state.fly_direc == Zigzag_A2A)?false:true;
			}
			//bool cw_flag = (zigzag_waypoint_state.direct > 0) & (zigzag_waypoint_state.fly_direc == Zigzag_A2A);
			//cw_flag = cw_flag & (zigzag_waypoint_state.fly_direc == Zigzag_A2A);
			wp_nav->set_u_turn(center, radians(zigzag_bearing * 0.01f), zigzag_waypoint_state.width * 0.5f, cw_flag, pi_flag);
			//circle_nav->init(center);
			//sprintf(buf, "zigzag Bx:%.2f By:%.2f next x:%.2f  y:%.2f",  zigzag_waypoint_state.vB_pos.x, zigzag_waypoint_state.vB_pos.y, next.x, next.y);
			//GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, buf);
		}else{

		wp_nav->set_wp_destination(next, false);
		}
		//ziazag_point_prev = next;
		// switch sprayer run on or off
		sprayer.run(zigzag_waypoint_state.index%2 == 0);
		//GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "Set next wp");
		break;

	// no power or no drug will record breakpoint
	case Zigzag_PowerNone:
	case Zigzag_DrugNone:
	case Zigzag_ModeSwitch:
		//next_dest = zigzag_waypoint_state.bp_pos;
		//wp_nav->set_wp_destination(zigzag_waypoint_state.bp_pos);
		wp_nav->set_wp_destination(zigzag_waypoint_state.vBP_pos, false);
		//sprintf(buf, "bp.z %.4f", zigzag_waypoint_state.vBP_pos.z);
		//GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, buf);
		//zigzag_set_destination(next_dest);
		//zigzag_waypoint_state.bp_mode = Zigzag_None;
		//zigzag_waypoint_state.index--;
		//zigzag_waypoint_state.action = false;
		zigzag_waypoint_state.flag = 0x05;
		wp_nav->set_zigzag_mode(false);
		// stop sprayer
		sprayer.run(false);
		break;


	case Zigzag_PilotOverride:
		//Vector3f next;
		zigzag_calculate_next_dest(next, zigzag_waypoint_state.index);
		next.z = zigzag_waypoint_state.vBP_pos.z;

		switch(zigzag_waypoint_state.index%4)
		{
		case 1:
		case 3:
			// 断点位于横移的阶段，不飞向断点，直接飞向横移终点

		break;

		case 2:
		case 0:
			cur_pos = inertial_nav.get_position();
			v_BP2Next = next - zigzag_waypoint_state.vBP_pos;
			v_BP2Next.z = 0;
			track_length = v_BP2Next.length();
			if(is_zero(track_length)) {
				break;
			}
			v_BP2Next_uint = v_BP2Next / track_length;
			//v_BP2Next.z = 0;
			v_BP2Cur = cur_pos - zigzag_waypoint_state.vBP_pos;
			//v_BP2Cur.z = 0;
			dotproduct = v_BP2Next_uint.x * v_BP2Cur.x + v_BP2Next_uint.y * v_BP2Cur.y;
			if(dotproduct > 0){

				if(dotproduct < track_length){
					v_BP2E =  v_BP2Next_uint * dotproduct;
					next = v_BP2E + zigzag_waypoint_state.vBP_pos;
				}

			}
			else
			{
				next = zigzag_waypoint_state.vBP_pos;
			}

			break;
		}
		wp_nav->set_wp_destination(next, false);

		wp_nav->set_zigzag_mode(false);
		// stop sprayer
		sprayer.run(false);
		//zigzag_waypoint_state.bp_mode = Zigzag_None;
		//zigzag_waypoint_state.index--;
		//zigzag_waypoint_state.action = false;
		break;
	}
}

void Copter::zigzag_calculate_next_dest(/*Location_Class& dest*/Vector3f& next, uint16_t index)
{
	Vector3f v_A2B  = zigzag_waypoint_state.vB_pos - zigzag_waypoint_state.vA_pos;
	v_A2B.z = 0;
	// A(x1,y1),B(x2,y2),设下一个点为N,其坐标为(x,y)，AB(x2-x1, y2-y1)垂直于BN(x-x2, y-y2)，则v_A2B * v_B2N = 0, 且|B2N| = width， |v_A2B x v_B2N| = |v_A2B| * |v_B2N|
	// a1 * x + b1 * y = c1   a2 * x + b2 * y = c2
	// 根据高斯消元法 可以求出坐标(x,y)
	float dist_AB = v_A2B.length();
	float a1 = v_A2B.x;
	float b1 = v_A2B.y;
	float c1 = 0.0f;//v_A2B.x * zigzag_waypoint_state.vB_pos.x + v_A2B.y * zigzag_waypoint_state.vB_pos.y;
	float a2=0, b2=0, c2 = 0;
	//char buf[128];

	switch(index%4)
	{
	case 1:
		c1 = v_A2B.x * zigzag_waypoint_state.vB_pos.x + v_A2B.y * zigzag_waypoint_state.vB_pos.y;
		c2 = zigzag_waypoint_state.direct * dist_AB * zigzag_waypoint_state.width * ((index+1)>>1);
		c2 = c2 + zigzag_waypoint_state.vB_pos.x * v_A2B.y - zigzag_waypoint_state.vB_pos.y * v_A2B.x;
		a2 = v_A2B.y;
		b2 = -v_A2B.x;
		zigzag_waypoint_state.fly_direc = Zigzag_B2B;
		break;
	case 0:
		//a1 = v_A2B.x;
		//b1 = v_A2B.y;
		c1 = v_A2B.x * zigzag_waypoint_state.vB_pos.x + v_A2B.y * zigzag_waypoint_state.vB_pos.y;
		c2 = zigzag_waypoint_state.direct * dist_AB * zigzag_waypoint_state.width * ((index+1)>>1);
		c2 = c2 + zigzag_waypoint_state.vB_pos.x * v_A2B.y - zigzag_waypoint_state.vB_pos.y * v_A2B.x;
		a2 = v_A2B.y;
		b2 = -v_A2B.x;
		zigzag_waypoint_state.fly_direc = Zigzag_A2B;
		break;
	case 2:
		c1 = v_A2B.x * zigzag_waypoint_state.vA_pos.x + v_A2B.y * zigzag_waypoint_state.vA_pos.y;
		c2 = zigzag_waypoint_state.direct * dist_AB * zigzag_waypoint_state.width * ((index+1)>>1);
		c2 = c2 + zigzag_waypoint_state.vA_pos.x * v_A2B.y - zigzag_waypoint_state.vA_pos.y * v_A2B.x;
		a2 = v_A2B.y;
		b2 = -v_A2B.x;
		zigzag_waypoint_state.fly_direc = Zigzag_B2A;
		break;
	case 3:
		//a1 = v_A2B.x;
		//b1 = v_A2B.y;
		c1 = v_A2B.x * zigzag_waypoint_state.vA_pos.x + v_A2B.y * zigzag_waypoint_state.vA_pos.y;
		c2 = zigzag_waypoint_state.direct * dist_AB * zigzag_waypoint_state.width * ((index+1)>>1);
		c2 = c2 + zigzag_waypoint_state.vA_pos.x * v_A2B.y - zigzag_waypoint_state.vA_pos.y * v_A2B.x;
		a2 = v_A2B.y;
		b2 = -v_A2B.x;
		zigzag_waypoint_state.fly_direc = Zigzag_A2A;
		break;
	}

	/*if((index%2) == 0){
		zigzag_waypoint_state.area = dist_AB * zigzag_waypoint_state.width * 0.0001 * 0.5 * (index - 2); // 作业面积 单位为m2
	}*/

	//Vector3f next;
	float denominator = (a1 * b2 - a2 * b1);
	//float denominatorY = a2 * b1 - a1 * b2;
	// 如果 dist_AB 不等于0， 则 denominator 不可能为0
	if(!is_zero(denominator)){
		next.x = (c1 * b2 - c2 * b1) / denominator;
		next.y = -(c1 * a2 - c2 * a1) / denominator;
	}
	//sprintf(buf, "zigzag index %d Bnext x:%.2f  y:%.2f", index, next.x, next.y);
	//GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, buf);
	next.z = inertial_nav.get_position().z;
	//wp_nav->set_wp_destination(next, false);
}

// zigzag set breakpoint mode
void Copter::zigzag_set_bp_mode(ZigzagBPMode bp_mode)
{
	zigzag_waypoint_state.bp_mode = bp_mode;
}

// zigzag stop
void Copter::zigzag_stop()
{
	if(zigzag_mode == Zigzag_Auto && zigzag_rc_state != RC_MID){
		zigzag_auto_stop();
		zigzag_waypoint_state.bp_mode = Zigzag_ModeSwitch;
		return;
	}
	else if((zigzag_waypoint_state.flag & 0x05) == 0x05)
		zigzag_waypoint_state.bp_mode = Zigzag_PilotOverride;

	//zigzag_save();
	//zigzag_waypoint_state.bp_mode = Zigzag_ModeSwitch;
}

// zigzag auto stop, stop auto run and record current position as breakpoint position
void Copter::zigzag_auto_stop()
{
	if(zigzag_waypoint_state.bp_mode == Zigzag_None) {
	zigzag_waypoint_state.vBP_pos = inertial_nav.get_position();
	zigzag_waypoint_state.bp_pos = current_loc;
	}

	//zigzag_waypoint_state.bp_mode = Zigzag_ModeSwitch;
}

// zigzag record A point or B point; aPoint==true record A point; aPoint == false record B point
bool Copter::zigzag_record_point(bool aPoint)
{
	bool ret = false;
	const Vector3f& vel = inertial_nav.get_velocity();
	float vel_horizontal = norm(vel.x, vel.y);
	// position not healthy then return, no recording
	// before record point, horizontal velocity must less than 1m/s
	if(!position_ok() || vel_horizontal > 100){
		return false;
	}
	// record A point
	if(aPoint){
		// clear all record
		zigzag_clear_record();

		zigzag_waypoint_state.vA_pos = inertial_nav.get_position();
		zigzag_waypoint_state.a_pos = current_loc;
		// After record A point, clear B point flag
		//zigzag_waypoint_state.b_hasbeen_defined = false;
		zigzag_waypoint_state.a_hasbeen_defined = true;

		zigzag_waypoint_state.area = 0.0f;
		ret = true;
	}
	// before record B point, A point must be recorded
	else if(zigzag_waypoint_state.a_hasbeen_defined){
		zigzag_waypoint_state.vB_pos = inertial_nav.get_position();
		zigzag_waypoint_state.b_pos = current_loc;
		zigzag_waypoint_state.b_hasbeen_defined = true;
		zigzag_waypoint_state.area = 0.0f;
		ret = true;
		//zigzag_waypoint_state.a_hasbeen_defined = true;
	}

	return ret;
}

// clear all record
void Copter::zigzag_clear_record(void)
{
	// set all parameter 0
	g.Zigzag_time.set_and_save(0);

    g2.ab_index.set_and_save(0);
    g2.ab_dirct.set_and_save(0);
    g2.aPos_lat.set_and_save(0);
    g2.aPos_lng.set_and_save(0);
    g2.aPos_alt.set_and_save(0);
    g2.bPos_lat.set_and_save(0);
    g2.bPos_lng.set_and_save(0);
    g2.bPos_alt.set_and_save(0);
    g2.bpPos_lat.set_and_save(0);
    g2.bpPos_lng.set_and_save(0);
    g2.bpPos_alt.set_and_save(0);
    g2.ab_bpMode.set_and_save(0);

	zigzag_waypoint_state.a_hasbeen_defined = false;
	zigzag_waypoint_state.b_hasbeen_defined = false;
	zigzag_waypoint_state.direct = 0;
	zigzag_waypoint_state.index = 0;
	zigzag_waypoint_state.bp_mode = Zigzag_None;
	zigzag_waypoint_state.width = g.Zigzag_width * 100; // convert to cm
}


// save record
void Copter::zigzag_save(void)
{
	// record time
    uint64_t gps_timestamp = gps.time_epoch_usec();
    int32_t cur_timestamp_min = gps_timestamp / 6.0e7f;
    g.Zigzag_time.set_and_save(cur_timestamp_min);

    g2.ab_index.set_and_save(zigzag_waypoint_state.index);
    g2.ab_dirct.set_and_save(zigzag_waypoint_state.direct);
    g2.aPos_lat.set_and_save(zigzag_waypoint_state.a_pos.lat);
    g2.aPos_lng.set_and_save(zigzag_waypoint_state.a_pos.lng);
    g2.aPos_alt.set_and_save(zigzag_waypoint_state.a_pos.alt);
    g2.bPos_lat.set_and_save(zigzag_waypoint_state.b_pos.lat);
    g2.bPos_lng.set_and_save(zigzag_waypoint_state.b_pos.lng);
    g2.bPos_alt.set_and_save(zigzag_waypoint_state.b_pos.alt);
    g2.bpPos_lat.set_and_save(zigzag_waypoint_state.bp_pos.lat);
    g2.bpPos_lng.set_and_save(zigzag_waypoint_state.bp_pos.lng);
    g2.bpPos_alt.set_and_save(zigzag_waypoint_state.bp_pos.alt);

	switch(zigzag_waypoint_state.bp_mode){
	case Zigzag_None:
		g2.ab_bpMode.set_and_save(0);
		break;
	case Zigzag_PowerNone:
		g2.ab_bpMode.set_and_save(1);
		break;
	case Zigzag_DrugNone:
		g2.ab_bpMode.set_and_save(2);
		break;
	case Zigzag_ModeSwitch:
		g2.ab_bpMode.set_and_save(3);
		break;
	case Zigzag_PilotOverride:
		g2.ab_bpMode.set_and_save(4);
		break;
	}
    //g2.ab_bpMode.set_and_save(zigzag_waypoint_state.bp_mode);
}


void Copter::zigzag_load(void)
{
	//char buf[128];
	zigzag_waypoint_state.width = g.Zigzag_width * 100; // convert to cm
	zigzag_waypoint_state.index = g2.ab_index;
	zigzag_waypoint_state.direct = g2.ab_dirct;
	zigzag_waypoint_state.a_pos.lat = g2.aPos_lat;
	zigzag_waypoint_state.a_pos.lng = g2.aPos_lng;
	zigzag_waypoint_state.a_pos.alt = g2.aPos_alt;
	zigzag_waypoint_state.b_pos.lat = g2.bPos_lat;
	zigzag_waypoint_state.b_pos.lng = g2.bPos_lng;
	zigzag_waypoint_state.b_pos.alt = g2.bPos_alt;
	zigzag_waypoint_state.bp_pos.lat = g2.bpPos_lat;
	zigzag_waypoint_state.bp_pos.lng = g2.bpPos_lng;
	zigzag_waypoint_state.bp_pos.alt = g2.bpPos_alt;

	switch(g2.ab_bpMode){
	case 0:
		zigzag_waypoint_state.bp_mode = Zigzag_None;
		break;
	case 1:
		zigzag_waypoint_state.bp_mode = Zigzag_PowerNone;
		break;
	case 2:
		zigzag_waypoint_state.bp_mode = Zigzag_DrugNone;
		break;
	case 3:
		zigzag_waypoint_state.bp_mode = Zigzag_ModeSwitch;
		break;
	case 4:
		zigzag_waypoint_state.bp_mode = Zigzag_ModeSwitch;//Zigzag_PilotOverride;
		break;
	}
	//zigzag_waypoint_state.bp_mode = ZigzagBPMode(g2.ab_bpMode);

	zigzag_waypoint_state.a_pos.get_vector_xy_from_origin_NEU(zigzag_waypoint_state.vA_pos);
	zigzag_waypoint_state.b_pos.get_vector_xy_from_origin_NEU(zigzag_waypoint_state.vB_pos);
	zigzag_waypoint_state.bp_pos.get_vector_xy_from_origin_NEU(zigzag_waypoint_state.vBP_pos);
	zigzag_waypoint_state.vBP_pos.z = zigzag_waypoint_state.bp_pos.alt;

	//加载作业面积

	//sprintf(buf, "zigzag BPx:%.2f BPy:%.2f BPz:%.2f",  zigzag_waypoint_state.vBP_pos.x, zigzag_waypoint_state.vBP_pos.y, zigzag_waypoint_state.vBP_pos.z);
	//GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, buf);
}
