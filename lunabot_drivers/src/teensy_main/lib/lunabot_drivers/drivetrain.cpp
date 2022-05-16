#include "drivetrain.h"

namespace drivetrain
{

	WheelControl front_left_ctrl = {
		.motor = drivetrain_cfg.front_left, .prev_err = 0, .prev_pos = 0, .curr_vel = 0, .setp = 0, .forward = CW, .backward = CCW};

	WheelControl front_right_ctrl = {
		.motor = drivetrain_cfg.front_right, .prev_err = 0, .prev_pos = 0, .curr_vel = 0, .setp = 0, .forward = CW, .backward = CCW };

	WheelControl back_left_ctrl = {
		.motor = drivetrain_cfg.back_left, .prev_err = 0, .prev_pos = 0, .curr_vel = 0, .setp = 0, .forward = CCW, .backward = CW};

	WheelControl back_right_ctrl = {
		.motor = drivetrain_cfg.back_right, .prev_err = 0, .prev_pos = 0, .curr_vel = 0, .setp = 0, .forward = CW, .backward = CCW};

	Encoder front_left_enc(DT_FT_LFT_ENC_A_PIN, DT_FT_LFT_ENC_B_PIN);
	Encoder front_right_enc(DT_FT_RT_ENC_A_PIN, DT_FT_RT_ENC_B_PIN);
	Encoder back_left_enc(DT_BK_LFT_ENC_A_PIN, DT_BK_LFT_ENC_B_PIN);
	Encoder back_right_enc(DT_BK_RT_ENC_A_PIN, DT_BK_RT_ENC_B_PIN);
	IntervalTimer ctrl_tim;

	void init()
	{

		// set all pwm and direction pins to output
		init_motor(drivetrain_cfg.front_left);
		init_motor(drivetrain_cfg.front_right);
		init_motor(drivetrain_cfg.back_left);
		init_motor(drivetrain_cfg.back_right);
		ctrl_tim.begin(ctrl_loop, DT_MILLIS);
	}

	void run_drivetrain(const lunabot_msgs::Drivetrain &drive_msg, ros::NodeHandle* nh)
	{
		/*
			Tank drive steering
		*/
		front_left_ctrl.setp = drive_msg.left; // wheel velocity [-1,1]
		back_left_ctrl.setp = drive_msg.left;
		front_right_ctrl.setp = drive_msg.right;
		back_right_ctrl.setp = drive_msg.right;
	}

	float calc_pid(WheelControl *ctrl)
	{
		float err = ctrl->setp - ctrl->curr_vel;
		float vel = ctrl->setp;
		vel += P_CTRL * err;
		vel += D_CTRL * (err - ctrl->prev_err);
		ctrl->prev_err = err;
		return vel;
	}

	void motor_loop(WheelControl *ctrl, Encoder *enc)
	{
		int pos = enc->read();
		ctrl->curr_vel = (pos - ctrl->prev_pos) / DT;
		ctrl->curr_vel /= F_;
		float goal_vel = calc_pid(ctrl);
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
		motor_loop(&front_left_ctrl, &front_left_enc);
		motor_loop(&front_right_ctrl, &front_right_enc);
		motor_loop(&back_left_ctrl, &front_left_enc); // back left encoder doesn't work
		motor_loop(&back_right_ctrl, &back_right_enc);
	}
}