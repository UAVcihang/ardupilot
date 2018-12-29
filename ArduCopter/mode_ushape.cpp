/***********************************************************************************************************************
*文件说明：U型控制实现文件
*文件功能：函数任务
*修改日期：2018-10-17
*修改作者：cihang_uav
*备注信息：Init and run calls for stabilize flight mode
*************************************************************************************************************************/


#include "Copter.h"
#include <AC_Sprayer/AC_Sprayer.h>
#include <AP_Math/AP_Math.h>

/***********************************************************************************************************************
*函数原型：bool Copter::Ushape::init(bool ignore_checks)
*函数功能：U型控制函数初始化
*修改日期：2018-10-18
*修改作者：cihang_uav
*备注信息：Ushape_init - initialise stabilize controller
*************************************************************************************************************************/
bool Copter::ModeUshape::init(bool ignore_checks)
{

    if (!(motors->armed()) ) //没解锁就会立即退出初始化的过程
    {
        return false;
    }

     if (copter.position_ok()  || ignore_checks)  //位置ok，并且ignore_checks=1表示没有解锁，
     {
            //初始化航点状态------initialise waypoint state
    	    ushape_change_yaw = false;
            if(ushape_waypoint_state.b_hasbeen_defined || ushape_waypoint_state.bp_mode != Ushape_None) //b点信息定义，没有断点
            {

            	//获得当前偏航数据----get_bearing_cd(_APoint, _BPoint) * 0.01f;
            	ushape_bearing = get_bearing_cd(ushape_waypoint_state.a_pos, ushape_waypoint_state.b_pos);//获得当前航向

            	ushape_mode = Ushape_Auto;  //1
            	ushape_auto_complete_state = (ushape_waypoint_state.bp_mode != Ushape_None);

            	Vector3f v_A2B  = ushape_waypoint_state.vB_pos - ushape_waypoint_state.vA_pos; //计算B点到A点的矢量
            	ushape_dist=v_A2B.length(); //计算长度

            	ushape_waypoint_state.flag = ushape_waypoint_state.flag << 1;
            	ushape_waypoint_state.flag += 1;

            	AP_Notify::flags.ushape_record_mode=4; //用来记录进入AB点模式的指示灯
    			gcs().send_text(MAV_SEVERITY_WARNING,"Ushape Auto Init"); //发送自动信息

    			auto_yaw.set_mode(AUTO_YAW_HOLD); //设定自动偏航保持
            	wp_nav->wp_and_spline_init();
            	wp_nav->set_wp_destination(copter.current_loc);

            }
            else
            {
            	AP_Notify::flags.ushape_record_mode=0;
            	gcs().send_text(MAV_SEVERITY_WARNING,"Ushape manual"); //发送自动信息
            	ushape_mode = Ushape_Manual;
            	ushape_auto_complete_state = false;
            	ushape_waypoint_state.area = 0.0f; //计算作业面积
#if 1
            	// fix loiter glitch
                if (!copter.failsafe.radio)
                {
                    float target_roll, target_pitch;
                    // apply SIMPLE mode transform to pilot inputs
                    //update_simple_mode();
                 // set target to current position	            // convert pilot input to lean angles
                    get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());
                     // process pilot's roll and pitch input
                    loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);
                } else
                {
                    // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
                    loiter_nav->clear_pilot_desired_acceleration();
                }

            //初始化目标----initialize's loiter position and velocity on xy-axes from current pos and velocity
            loiter_nav->init_target();

            if (!pos_control->is_active_z())
            {

                pos_control->set_alt_target_to_current_alt();
                pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
            }
#else

                copter.mode_poshold.init(true);
#endif
            }

    		switch(ushape_waypoint_state.direct) //copter.zigzag_waypoint_state.direct=0,默认是0
    		{

    			case 1:
    				ushape_rc_state = RC_LEFT;  //在最左边
    				break;
    			case -1:
    				ushape_rc_state = RC_RIGHT; //在最右边
    				break;
    			case 0:
    			default:
    				ushape_rc_state = RC_MID;  //默认在中间
    				break;
    		 }
             return true;
       }
        else
        {
            return false;
        }
}

/***********************************************************************************************************************
*函数原型：void Copter::Ushape::run()
*函数功能：U型控制运行函数
*修改日期：2018-10-18
*修改作者：cihang_uav
*备注信息：Ushape_run - runs the main stabilize controller should be called at 100hz or more
*************************************************************************************************************************/

void Copter::ModeUshape::run()
{
	//如果没有自动解锁，电机没有使能，设置油门值为零，并且立即退出------ if not auto armed or motors not enabled set throttle to zero and exit immediately
	    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete)
	    {
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
	        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), 0);
	        pos_control->update_z_controller();


	#endif
	        return;
	    }

	    switch(ushape_mode) //判断当前模式
	    {

	    case Ushape_Manual:
	#if 1
	    	//gcs().send_text(MAV_SEVERITY_WARNING,"ZIG_Manual RUN1");
	        ushape_manual_control();
	#else
	    	//gcs().send_text(MAV_SEVERITY_WARNING,"ZIG_Manual RUN2");
	        poshold_run();
	#endif
	    	break;

	    case Ushape_Auto:


	    	if(ushape_auto_complete_state && !ushape_change_yaw) //第一次：zigzag_auto_complete_state=0，zigzag_change_yaw=0;不会进入if
	    	{

	           wp_nav->wp_and_spline_init();
	           ushape_set_destination();
	           ushape_auto_complete_state = false;
	           wp_nav->set_fast_waypoint(true);
	          // gcs().send_text(MAV_SEVERITY_WARNING,"ZIG_AUTO RUN1");

	    	}
	    	else
	    	{
	    	//	gcs().send_text(MAV_SEVERITY_WARNING,"ZIG_AUTO RUN2");
	    		ushape_auto_control();
	    	}
	    	break;
	    }
}

/***********************************************************************************************************************
*函数原型：void Copter::ZigZag::zigzag_manual_control()
*函数功能：手动控制模式
*修改日期：2018-10-18
*修改作者：cihang_uav
*备注信息：zigzag_manual_control - process manual control
*************************************************************************************************************************/
void Copter::ModeUshape::ushape_manual_control(void)
{
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;
    float target_roll = 0.0f;
    float target_pitch = 0.0f;

    // initialize vertical speed and acceleration's range
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);
    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio)
    {
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
        target_climb_rate = ushape_constrain(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    }
    else
    {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we
        //do not switch to RTL for some reason
    	loiter_nav->clear_pilot_desired_acceleration();
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    // run loiter controller
    loiter_nav->update(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(),loiter_nav->get_pitch(), target_yaw_rate/*, get_smoothing_gain()*/);

    // adjust climb rate using rangefinder
    if (copter.rangefinder_alt_ok())
    {
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


/***********************************************************************************************************************
*函数原型：void Copter::Ushape::ushape_auto_control(void)
*函数功能：自动控制模式
*修改日期：2018-9-26
*修改作者：cihang_uav
*备注信息： 1、方向不可控， 油门 俯仰 横滚 均可控
 *        2、控制俯仰 横滚退出作业模式
 *        3、控制油门可控制飞行高度，但不退出作业模式
*************************************************************************************************************************/

void Copter::ModeUshape::ushape_auto_control(void)
{
	static uint32_t last_of_update = 0;

    // process pilot's yaw input
	float target_roll = 0, target_pitch = 0;
    float target_yaw_rate = 0;
    float target_climb_rate = 0.0f;


    if (!copter.failsafe.radio)
    {
        // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    	get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());
        //get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate))
        {
        	auto_yaw.set_mode(AUTO_YAW_HOLD); //设定自动偏航保持

        }

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = ushape_constrain(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    }


     // ushape mode direction
     if(ushape_waypoint_state.direct == 0)
    {
    	switch(ushape_rc_state)
    	{
    	case RC_MID:
    		if(target_roll > copter.aparm.angle_max /10)
    		{

    			ushape_rc_state = RC_RIGHT;
    		}

    		else if(target_roll < -copter.aparm.angle_max /10)
    		{

    			ushape_rc_state = RC_LEFT;
    		}

    		break;
    	case RC_RIGHT:
    		if((target_roll<=(copter.aparm.angle_max / 100))&&(target_roll>=(-copter.aparm.angle_max / 100)))
    		{
    			ushape_waypoint_state.direct = -1;
    		}

    		break;
    	case RC_LEFT:
    		if((target_roll<=(copter.aparm.angle_max / 100))&&(target_roll>=(-copter.aparm.angle_max / 100)))
    		{

    			ushape_waypoint_state.direct = 1;

    		}
    		break;
    	}

    	if(ushape_waypoint_state.direct != 0)
    	{
    		ushape_auto_complete_state = true;  //到这里才是真正的执行
    		ushape_waypoint_state.flag = 0x05;

    		return;
    	}
    }

   else if(!is_zero(target_roll) || !is_zero(target_pitch) /*|| !is_zero(target_yaw_rate)*/)
    {
	   ushape_mode = Ushape_Manual;
    ushape_auto_complete_state = false;
#if 1
        loiter_nav->init_target();
#else
        poshold_init(true);
#endif
        ushape_auto_stop();
    	return;
    }
    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller to update xy
    copter.failsafe_terrain_set_status(wp_nav->update_ushape_wpnav()); //运行U型控制

    // adjust climb rate using rangefinder
    if (copter.rangefinder_alt_ok())
    {
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


     //call attitude controller


    if (auto_yaw.mode()  == AUTO_YAW_HOLD)
    {
         //roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), /*target_yaw_rate*/0/*, get_smoothing_gain()*/);
    }
    else if (auto_yaw.mode()  == AUTO_YAW_RATE)
    {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
    	 attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(),auto_yaw.rate_cds());

    }
    else
    {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
       attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true/*, get_smoothing_gain()*/);

    }
    //计算作业面积
    uint32_t now = AP_HAL::millis();
     if((now - last_of_update) > 1000 && (ushape_waypoint_state.index % 2) == 0 && ushape_waypoint_state.index!=0 )
     {
     	last_of_update = now;
     	ushape_waypoint_state.area =  (ushape_dist * ushape_waypoint_state.index - wp_nav->get_track_covered_ushape()) * ushape_waypoint_state.width * 0.0001 * 0.5;
     }


    if(ushape_waypoint_state.direct != 0 && ((ushape_waypoint_state.flag & 0x05) == 0x05))
    {
    	ushape_auto_complete_state = wp_nav->reached_wp_destination();        //没有到就返回0，到了就返回1
		if(ushape_change_yaw && copter.mode_auto.verify_yaw())
		{
			ushape_change_yaw = false;

		}
		if(ushape_auto_complete_state && ushape_waypoint_state.bp_mode != Ushape_None)
		{
			ushape_change_yaw = true;
			auto_yaw.set_mode(AUTO_YAW_FIXED); //设定自动偏航保持

			auto_yaw.set_fixed_yaw(ushape_bearing*0.01f,0,0,0);
			ushape_waypoint_state.bp_mode = Ushape_None;
			ushape_waypoint_state.index--;

			//这里提示断点模式，每次进入会闪烁两下红灯
			if(ushape_change_yaw==1)
			{

				AP_Notify::flags.ushape_record_mode_erro=16;
			}
			else
			{
				AP_Notify::flags.ushape_record_mode_erro=0;

			}
		}

	}
}

/***********************************************************************************************************************
*函数原型：void Copter::Ushape::ushape_set_destination(void)
*函数功能：设置下一目标点
*修改日期：2018-10-18
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/
void Copter::ModeUshape::ushape_set_destination(void)
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


	switch(ushape_waypoint_state.bp_mode)
	{
	case Ushape_None:

		ushape_waypoint_state.index++;

		ushape_calculate_next_dest(next, ushape_waypoint_state.index);

		// 从第一个点开始走U形,并调整机头，机头朝向与飞行方向一致
	       if((ushape_waypoint_state.fly_direc < Ushape_A2B )) //说明是走的U型弧线
			{
	    	    //求出圆心
				Vector3f ziazag_point_prev = pos_control->get_pos_target();  //获取当前位置
				center.x  = (ziazag_point_prev.x + next.x) * 0.5f;
				center.y  = (ziazag_point_prev.y + next.y) * 0.5f;
				center.z  = next.z;

               //ushape_waypoint_state.direct=-1右边飞行，ushape_waypoint_state.direct=1左边飞行
				bool pi_flag = (ushape_waypoint_state.direct > 0)?true:false;  //pi_flag=0右边
				bool cw_flag;
				if(ushape_waypoint_state.direct > 0) //左边飞行
				{
					cw_flag = (ushape_waypoint_state.fly_direc == Ushape_A2A)?true:false;
				}
				else               //右边飞行
				{
					cw_flag = (ushape_waypoint_state.fly_direc == Ushape_A2A)?false:true; //cw_flag=1,顺时针飞行
				}
                //圆心坐标-----航向角度------设定的宽度，也就是半径-------确地哪个U型-----顺时针，还是逆时针运行
				wp_nav->set_u_turn(center, ushape_radians(ushape_bearing * 0.01f), ushape_waypoint_state.width * 0.5f, cw_flag, pi_flag);

				//circle_nav->init(center);
				//sprintf(buf, "zigzag Bx:%.2f By:%.2f next x:%.2f  y:%.2f",  zigzag_waypoint_state.vB_pos.x, zigzag_waypoint_state.vB_pos.y, next.x, next.y);
				//GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, buf);
			}
	        else //走直线
			{
			   wp_nav->set_wp_destination(next, false);
			}

		// switch sprayer run on or off
		copter.sprayer.run(ushape_waypoint_state.index%2 == 0);

		break;

	// no power or no drug will record breakpoint
	case Ushape_PowerNone:
	case Ushape_DrugNone:
	case Ushape_ModeSwitch:

		wp_nav->set_wp_destination(ushape_waypoint_state.vBP_pos, false);

		ushape_waypoint_state.flag = 0x05;
		wp_nav->set_ushape_mode(false);
		// stop sprayer
		copter.sprayer.run(false);
		break;


	case Ushape_PilotOverride:
		//Vector3f next;
		ushape_calculate_next_dest(next, ushape_waypoint_state.index);
		next.z = ushape_waypoint_state.vBP_pos.z;

		switch(ushape_waypoint_state.index%4)
		{
		case 1:
		case 3:

		      break;

		case 2:
		case 0:
			cur_pos = inertial_nav.get_position();
			v_BP2Next = next - ushape_waypoint_state.vBP_pos;
			v_BP2Next.z = 0;
			track_length = v_BP2Next.length();
			if(is_zero(track_length))
			{
				break;
			}

			v_BP2Next_uint = v_BP2Next / track_length;
			//v_BP2Next.z = 0;
			v_BP2Cur = cur_pos - ushape_waypoint_state.vBP_pos;
			//v_BP2Cur.z = 0;
			dotproduct = v_BP2Next_uint.x * v_BP2Cur.x + v_BP2Next_uint.y * v_BP2Cur.y;
			if(dotproduct > 0)
			{

				if(dotproduct < track_length){
					v_BP2E =  v_BP2Next_uint * dotproduct;
					next = v_BP2E + ushape_waypoint_state.vBP_pos;
				}

			}
			else
			{
				next = ushape_waypoint_state.vBP_pos;
			}

			break;
		}
		wp_nav->set_wp_destination(next, false);
		wp_nav->set_ushape_mode(false);
		// stop sprayer
		copter.sprayer.run(false);

		break;
	}
}



/***********************************************************************************************************************
*函数原型：void Copter::Ushape::ushape_calculate_next_dest(Vector3f& next, uint16_t index)
*函数功能：设置下一目标点
*修改日期：2018-10-18
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/
void Copter::ModeUshape::ushape_calculate_next_dest(Vector3f& next, uint16_t index)
{
	Vector3f v_A2B  = ushape_waypoint_state.vB_pos - ushape_waypoint_state.vA_pos;
	v_A2B.z = 0;

	float dist_AB = v_A2B.length();
	float a1 = v_A2B.x;
	float b1 = v_A2B.y;
	float c1 = 0.0f;//v_A2B.x * zigzag_waypoint_state.vB_pos.x + v_A2B.y * zigzag_waypoint_state.vB_pos.y;
	float a2=0, b2=0, c2 = 0;


	switch(index%4)
	{

     //从A往B的方向走
	 case 0:

			c1 = v_A2B.x * ushape_waypoint_state.vB_pos.x + v_A2B.y * ushape_waypoint_state.vB_pos.y;
			c2 = ushape_waypoint_state.direct * dist_AB * ushape_waypoint_state.width * ((index+1)>>1);
			c2 = c2 + ushape_waypoint_state.vB_pos.x * v_A2B.y - ushape_waypoint_state.vB_pos.y * v_A2B.x;
			a2 = v_A2B.y;
			b2 = -v_A2B.x;
			ushape_waypoint_state.fly_direc = Ushape_A2B;    //Ushape_A2B=2
			break;

	//从B往B'的方向走
	 case 1:
			c1 = v_A2B.x * ushape_waypoint_state.vB_pos.x + v_A2B.y * ushape_waypoint_state.vB_pos.y;
			c2 = ushape_waypoint_state.direct * dist_AB * ushape_waypoint_state.width * ((index+1)>>1);
			c2 = c2 + ushape_waypoint_state.vB_pos.x * v_A2B.y - ushape_waypoint_state.vB_pos.y * v_A2B.x;
			a2 = v_A2B.y;
			b2 = -v_A2B.x;
			ushape_waypoint_state.fly_direc = Ushape_B2B;  //Ushape_B2B=1
			break;

	//从B往A的方向走
	 case 2:
		    c1 = v_A2B.x * ushape_waypoint_state.vA_pos.x + v_A2B.y * ushape_waypoint_state.vA_pos.y;
			c2 = ushape_waypoint_state.direct * dist_AB * ushape_waypoint_state.width * ((index+1)>>1);
			c2 = c2 + ushape_waypoint_state.vA_pos.x * v_A2B.y - ushape_waypoint_state.vA_pos.y * v_A2B.x;
			a2 = v_A2B.y;
			b2 = -v_A2B.x;
			ushape_waypoint_state.fly_direc = Ushape_B2A; //Ushape_B2A=3
			break;
	//从A往A'的方向走
	 case 3:

		c1 = v_A2B.x * ushape_waypoint_state.vA_pos.x + v_A2B.y * ushape_waypoint_state.vA_pos.y;
		c2 = ushape_waypoint_state.direct * dist_AB * ushape_waypoint_state.width * ((index+1)>>1);
		c2 = c2 + ushape_waypoint_state.vA_pos.x * v_A2B.y - ushape_waypoint_state.vA_pos.y * v_A2B.x;
		a2 = v_A2B.y;
		b2 = -v_A2B.x;
		ushape_waypoint_state.fly_direc = Ushape_A2A; //Ushape_A2A=0
		break;
	}

	//Vector3f next;
	float denominator = (a1 * b2 - a2 * b1);

	if(!is_zero(denominator))
	{
		next.x = (c1 * b2 - c2 * b1) / denominator;
		next.y = -(c1 * a2 - c2 * a1) / denominator;
	}

	next.z = inertial_nav.get_position().z;

}


/***********************************************************************************************************************
*函数原型：void Copter::Ushape::ushape_set_bp_mode(UshapeBPMode bp_mode)
*函数功能：设置AB点模式
*修改日期：2018-10-18
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/
void Copter::ModeUshape::ushape_set_bp_mode(UshapeBPMode bp_mode)
{
	ushape_waypoint_state.bp_mode = bp_mode;
}

/***********************************************************************************************************************
*函数原型：void Copter::Ushape::ushape_stop()
*函数功能：自动阻止
*修改日期：2018-10-8
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/
void Copter::ModeUshape::ushape_stop()
{
	if(ushape_mode == Ushape_Auto && ushape_rc_state != RC_MID)
	{
		ushape_auto_stop();
		ushape_waypoint_state.bp_mode = Ushape_ModeSwitch;
		return;
	}
	else if((ushape_waypoint_state.flag & 0x05) == 0x05)
	{

		ushape_waypoint_state.bp_mode = Ushape_PilotOverride;
	}

}

/***********************************************************************************************************************
*函数原型：void Copter::Ushape::ushape_auto_stop(void)
*函数功能：自动阻止
*修改日期：2018-9-26
*修改作者：cihang_uav
*备注信息：zigzag auto stop, stop auto run and record current position as breakpoint position
*************************************************************************************************************************/

void Copter::ModeUshape::ushape_auto_stop(void)
{
	if(ushape_waypoint_state.bp_mode == Ushape_None)
	{
		ushape_waypoint_state.vBP_pos = inertial_nav.get_position();
		ushape_waypoint_state.bp_pos = copter.current_loc;
	}

}


/***********************************************************************************************************************
*函数原型：bool Copter::Ushape::ushape_record_point(bool aPoint)
*函数功能：记录AB点信息
*修改日期：2018-10-18
*修改作者：cihang_uav
*备注信息：Ushape record A point or B point; aPoint==true record A point; aPoint == false record B point
*************************************************************************************************************************/

bool Copter::ModeUshape::ushape_record_point(bool aPoint)
{
//	 hal.uartG->printf("AAA\r\n");
	bool ret = false;
	const Vector3f& vel = inertial_nav.get_velocity();
	float vel_horizontal = norm(vel.x, vel.y);
	// position not healthy then return, no recording
	// before record point, horizontal velocity must less than 1m/s
	if(!copter.position_ok() || vel_horizontal > 100) //定位ok,并且运行速度很小，才可以
	{
//		 hal.uartG->printf("AAABBB\r\n");
		return false;
	}
	// record A point
	if(aPoint)
	{
		// clear all record
		ushape_clear_record();

		ushape_waypoint_state.vA_pos = inertial_nav.get_position();
		ushape_waypoint_state.a_pos = copter.current_loc;

		// After record A point, clear B point flag
		ushape_waypoint_state.a_hasbeen_defined = true;

		ret = true;
	}
	// before record B point, A point must be recorded
	else if(ushape_waypoint_state.a_hasbeen_defined)
	{
		ushape_waypoint_state.vB_pos = inertial_nav.get_position();
		ushape_waypoint_state.b_pos = copter.current_loc;
		ushape_waypoint_state.b_hasbeen_defined = true;
		ret = true;
	}
	return ret;
}






/***********************************************************************************************************************
*函数原型：void Copter::Ushape::ushape_clear_record(void)
*函数功能：清除所有的参数
*修改日期：2018-10-18
*修改作者：cihang_uav
*备注信息：clear all record
*************************************************************************************************************************/
void Copter::ModeUshape::ushape_clear_record(void)
{
	// set all parameter 0
	g.Ushape_time.set_and_save(0);

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

    ushape_waypoint_state.a_hasbeen_defined = false;
    ushape_waypoint_state.b_hasbeen_defined = false;
    ushape_waypoint_state.direct = 0;
    ushape_waypoint_state.index = 0;
    ushape_waypoint_state.bp_mode = Ushape_None;
    ushape_waypoint_state.width = g.Ushape_width * 100; // convert to cm
}

/***********************************************************************************************************************
*函数原型：void Copter::Ushape::ushape_save(void)
*函数功能：保存数据
*修改日期：2018-10-18
*修改作者：cihang_uav
*备注信息：save record
*************************************************************************************************************************/
void Copter::ModeUshape::ushape_save(void)
{
	// record time
    uint64_t gps_timestamp = copter.gps.time_epoch_usec();
    int32_t cur_timestamp_min = gps_timestamp / 6.0e7f;
    g.Ushape_time.set_and_save(cur_timestamp_min);

    g2.ab_index.set_and_save(ushape_waypoint_state.index);
    g2.ab_dirct.set_and_save(ushape_waypoint_state.direct);
    g2.aPos_lat.set_and_save(ushape_waypoint_state.a_pos.lat);
    g2.aPos_lng.set_and_save(ushape_waypoint_state.a_pos.lng);
    g2.aPos_alt.set_and_save(ushape_waypoint_state.a_pos.alt);
    g2.bPos_lat.set_and_save(ushape_waypoint_state.b_pos.lat);
    g2.bPos_lng.set_and_save(ushape_waypoint_state.b_pos.lng);
    g2.bPos_alt.set_and_save(ushape_waypoint_state.b_pos.alt);
    g2.bpPos_lat.set_and_save(ushape_waypoint_state.bp_pos.lat);
    g2.bpPos_lng.set_and_save(ushape_waypoint_state.bp_pos.lng);
    g2.bpPos_alt.set_and_save(ushape_waypoint_state.bp_pos.alt);

	switch(ushape_waypoint_state.bp_mode)
	{
	case Ushape_None:
		g2.ab_bpMode.set_and_save(0);
		break;
	case Ushape_PowerNone:
		g2.ab_bpMode.set_and_save(1);
		break;
	case Ushape_DrugNone:
		g2.ab_bpMode.set_and_save(2);
		break;
	case Ushape_ModeSwitch:
		g2.ab_bpMode.set_and_save(3);
		break;
	case Ushape_PilotOverride:
		g2.ab_bpMode.set_and_save(4);
		break;
	}

}

/***********************************************************************************************************************
*函数原型：void Copter::Ushape::ushape_load(void)
*函数功能：加载参数
*修改日期：2018-10-18
*修改作者：cihang_uav
*备注信息：zigzag_auto auto run
*************************************************************************************************************************/
void Copter::ModeUshape::ushape_load(void)
{

	ushape_waypoint_state.width = g.Ushape_width * 100; // convert to cm
	ushape_waypoint_state.index = g2.ab_index;
	ushape_waypoint_state.direct = g2.ab_dirct;
	ushape_waypoint_state.a_pos.lat = g2.aPos_lat;
	ushape_waypoint_state.a_pos.lng = g2.aPos_lng;
	ushape_waypoint_state.a_pos.alt = g2.aPos_alt;
	ushape_waypoint_state.b_pos.lat = g2.bPos_lat;
	ushape_waypoint_state.b_pos.lng = g2.bPos_lng;
	ushape_waypoint_state.b_pos.alt = g2.bPos_alt;
	ushape_waypoint_state.bp_pos.lat = g2.bpPos_lat;
	ushape_waypoint_state.bp_pos.lng = g2.bpPos_lng;
	ushape_waypoint_state.bp_pos.alt = g2.bpPos_alt;

	switch(g2.ab_bpMode)
	{
	case 0:
		ushape_waypoint_state.bp_mode = Ushape_None;
		break;
	case 1:
		ushape_waypoint_state.bp_mode = Ushape_PowerNone;
		break;
	case 2:
		ushape_waypoint_state.bp_mode = Ushape_DrugNone;
		break;
	case 3:
		ushape_waypoint_state.bp_mode = Ushape_ModeSwitch;
		break;
	case 4:
		ushape_waypoint_state.bp_mode = Ushape_PilotOverride;//Zigzag_PilotOverride;
		break;
	}

	ushape_waypoint_state.a_pos.get_vector_from_origin_NEU(ushape_waypoint_state.vA_pos);
	ushape_waypoint_state.b_pos.get_vector_from_origin_NEU(ushape_waypoint_state.vB_pos);
	ushape_waypoint_state.bp_pos.get_vector_from_origin_NEU(ushape_waypoint_state.vBP_pos);
	ushape_waypoint_state.vBP_pos.z = ushape_waypoint_state.bp_pos.alt;

}

/***********************************************************************************************************************
*函数原型：float Copter::ModeUshape::ushape_radians(float deg)
*函数功能：获取半径
*修改日期：2018-10-18
*修改作者：cihang_uav
*备注信息：zigzag_auto auto run
*************************************************************************************************************************/
 float Copter::ModeUshape::ushape_radians(float deg)
{
    return deg * DEG_TO_RAD;
}

 /***********************************************************************************************************************
 *函数原型： float Copter::ModeUshape::ushape_constrain(float val, float min, float max)
 *函数功能：限制范围
 *修改日期：2018-10-18
 *修改作者：cihang_uav
 *备注信息：
 *************************************************************************************************************************/
 float Copter::ModeUshape::ushape_constrain(float val, float min, float max)
 {
 	return (val < min) ? min : ((val > max) ? max : val);
 }



/************************************************************************************************************************************************
*                             File _end
*************************************************************************************************************************************************/

