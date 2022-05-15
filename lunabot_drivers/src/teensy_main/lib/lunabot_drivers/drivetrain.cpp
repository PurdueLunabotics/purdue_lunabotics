#include "drivetrain.h"

namespace drivetrain
{

	Encoder front_left_enc(DT_FT_LFT_ENC_A_PIN, DT_FT_LFT_ENC_B_PIN);
	Encoder front_right_enc(DT_FT_RT_ENC_A_PIN, DT_FT_RT_ENC_B_PIN);
	Encoder back_left_enc(DT_BK_LFT_ENC_A_PIN, DT_BK_LFT_ENC_B_PIN);
	Encoder back_right_enc(DT_BK_RT_ENC_A_PIN, DT_BK_RT_ENC_B_PIN);
	IntervalTimer ctrl_tim;

	void init()
	{
		front_left = {
			.motor = drivetrain_cfg.front_left, .prev_err = 0, .prev_pos = 0, .setp = 0.5, .forward = CW, .backward = CCW };

		front_right = {
			.motor = drivetrain_cfg.front_right, .prev_err = 0, .prev_pos = 0, .setp = 0.5, .forward = CW, .backward = CCW };

		back_left = {
			.motor = drivetrain_cfg.back_left, .prev_err = 0, .prev_pos = 0, .setp = 0.5, .forward = CCW, .backward = CW };

		back_right = {
			.motor = drivetrain_cfg.back_right, .prev_err = 0, .prev_pos = 0, .setp = 0.5, .forward = CW, .backward = CCW };

		// set all pwm and direction pins to output
		init_motor(drivetrain_cfg.front_left);
		init_motor(drivetrain_cfg.front_right);
		init_motor(drivetrain_cfg.back_left);
		init_motor(drivetrain_cfg.back_right);
    	ctrl_tim.begin(ctrl_loop, DT_MILLIS);
	}

	void run_drivetrain(const lunabot_msgs::Drivetrain &drive_msg, ros::NodeHandle &nh)
	{
		/*
			Tank drive steering
		*/
		front_left.setp = drive_msg.left; // wheel velocity [-1,1]	 
		back_left.setp = drive_msg.left;	 
		front_right.setp = drive_msg.right;
		back_right.setp = drive_msg.right;	
	}

	void motor_loop(WheelControl *ctrl, Encoder *enc)
	{
		int pos = enc->read();
		float curr_vel = (pos - ctrl->prev_pos) / DT;
		curr_vel /= F_;
		float goal_vel = calc_pid(ctrl->setp, curr_vel, &(ctrl->prev_err));
		if (goal_vel > 1)
		{
			goal_vel = 1;
		}
		else if (goal_vel < -1)
		{
			goal_vel = -1;
		}

		write_motor(ctrl->motor, abs(goal_vel) * 255, goal_vel > 0 ? ctrl->forward : ctrl->backward);

		ctrl->prev_pos = pos;
	}

	void ctrl_loop(void)
	{
		motor_loop(&front_left,&front_left_enc);
		motor_loop(&front_right,&front_right_enc);
		motor_loop(&back_left,&back_left_enc);
		motor_loop(&back_right,&back_right_enc);
	}

	static float calc_pid(float goal, float curr, volatile float *prev_err)
	{
		float err = goal - curr;
		float vel = goal;
		vel += P_CTRL * err;
		vel += D_CTRL * (err - *prev_err);
		*prev_err = err;
		return vel;
	}
}