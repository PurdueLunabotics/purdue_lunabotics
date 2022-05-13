#include "drivetrain.h"

namespace drivetrain
{

	void init()
	{
		front_left = {
			.motor = drivetrain_cfg.front_left, .enc = front_left_enc, .prev_err = 0, .prev_pos = 0, .setp = 0.5};

		front_right = {
			.motor = drivetrain_cfg.front_right, .enc = front_right_enc, .prev_err = 0, .prev_pos = 0, .setp = 0.5};

		back_left = {
			.motor = drivetrain_cfg.back_left, .enc = back_left_enc, .prev_err = 0, .prev_pos = 0, .setp = 0.5};

		back_right = {
			.motor = drivetrain_cfg.back_right, .enc = back_right_enc, .prev_err = 0, .prev_pos = 0, .setp = 0.5};

		// set all pwm and direction pins to output
		init_motor(drivetrain_cfg.front_left);
		init_motor(drivetrain_cfg.front_right);
		init_motor(drivetrain_cfg.back_left);
		init_motor(drivetrain_cfg.back_right);
	}

	void run_drivetrain(const lunabot_msgs::Drivetrain &drive_msg, ros::NodeHandle &nh)
	{
		/*
			Tank drive steering
		*/
		float left_wheel = drive_msg.left;	  // left wheel vel
		float right_wheel = drive_msg.right; // right wheel vel

		// calculate left and right chassis velocities
		int vel_l = map(left_wheel, -1, 1, -255, 255);	// Range from [-255,255]
		int vel_r = mapf(right_wheel, -128, 127, -255, 255); // Range from [-255,255]
		vel_l = constrain(vel_l, -255, 255);
		vel_r = constrain(vel_r, -255, 255);
		MotorDir left_front_vel_dir = (vel_l > 0) ? CW : CCW; // wiring issue
		MotorDir left_back_vel_dir = (vel_l > 0) ? CCW : CW;  // wiring issue
		MotorDir right_vel_dir = (vel_r > 0) ? CW : CCW;

		// nh.logerror("left_vel:");
		// nh.logerror(String(vel_l).c_str());
		// nh.logerror("right_vel:");
		// nh.logerror(String(vel_r).c_str());

		write_motor(drivetrain_cfg.front_left, abs(vel_l), left_front_vel_dir);
		write_motor(drivetrain_cfg.back_left, abs(vel_l), left_back_vel_dir);
		write_motor(drivetrain_cfg.front_right, abs(vel_r), right_vel_dir);
		write_motor(drivetrain_cfg.back_right, abs(vel_r), right_vel_dir);
	}



	void ctrl_loop(void)
	{
		int pos = enc.read();
		float curr_vel = (pos - prev_pos) / DT;
		curr_vel /= F;
		float goal_vel = calc_pid(goal, curr_vel);
		if (goal_vel > 1)
		{
			goal_vel = 1;
		}
		else if (goal_vel < -1)
		{
			goal_vel = -1;
		}
		digitalWrite(dir_pin, goal_vel > 0 ? LOW : HIGH);
		analogWrite(drive_pin, abs(goal_vel) * 255);

		prev_pos = pos;
		// delay(dt * 1000);
	}

	static generate_goal_vel()

	static float calc_pid(float goal, float curr, float prev_err)
	{
		float err = goal - curr;
		float vel = goal;
		vel += P_CTRL * err;
		vel += D_CTRL * (err - prev_err);
		prev_err = err;
		return vel;
	}
}