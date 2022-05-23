#include "actuation.h"

namespace actuation
{

	static StepperDir lead_screw_dir;
	static uint8_t lead_screw_en;

	Stepper lead_screw_stepper(actuation_cfg.lead_screw.steps,
							   actuation_cfg.lead_screw.DIR1_pin,
							   actuation_cfg.lead_screw.DIR2_pin);

	State lead_screw_curr = {.state = INT(LeadScrewState::STORED)};
	State lin_act_curr = {.state = INT(LinActState::STORED)};
	State lead_screw_setp = {.state = INT(LeadScrewState::STORED)};
	State lin_act_setp = {.state = INT(LinActState::STORED)};

	void lin_act_hall_cb()
	{
		if (digitalRead(LIN_ACT_HALL_PIN) == LOW)
		{
			lin_act_curr.state = (lin_act_curr.state + 1) % INT(LinActState::CNT);
			stop_motor(actuation_cfg.left_lin_act);
			stop_motor(actuation_cfg.right_lin_act);
		}
	}

	// Lead Screw Actuation

	void lead_screw_hall_cb()
	{
		if (digitalRead(LEAD_SCREW_HALL_PIN) == LOW)
		{
			lead_screw_curr.state = (lead_screw_curr.state + 1) % INT(LeadScrewState::CNT);
			stepper_off(actuation_cfg.lead_screw);
		}
	}

	void init()
	{
		// set all pwm and direction pins to output
		init_motor(actuation_cfg.left_lin_act);
		init_motor(actuation_cfg.right_lin_act);
		init_stepper(actuation_cfg.lead_screw, &lead_screw_stepper);
		stepper_off(actuation_cfg.lead_screw);
		lead_screw_en = 0;
		init_hall(LEAD_SCREW_HALL_PIN, lead_screw_hall_cb, &lead_screw_curr);
		init_hall(LIN_ACT_HALL_PIN, lin_act_hall_cb, &lin_act_curr);
	}

	void stepper_step()
	{
		if (lead_screw_en)
		{
			stepper_step(actuation_cfg.lead_screw, &lead_screw_stepper, lead_screw_dir);
		}
	}

	void move_to_setp(void)
	{
		if (lin_act_setp.state != lin_act_curr.state && lin_act_setp.state != INT(LinActState::STOPPED))
		{
			MotorDir dir = (lin_act_setp.state == INT(LinActState::STORED)) ? CCW : CW; // CCW rotates lin_act up when stored, CW retracts deposition down when extended
			write_motor(actuation_cfg.left_lin_act,
						actuation_cfg.left_lin_act.MAX_PWM, dir);
			write_motor(actuation_cfg.right_lin_act,
						actuation_cfg.right_lin_act.MAX_PWM, dir);
		}
		if (lead_screw_curr.state != lead_screw_setp.state && lead_screw_curr.state != INT(LeadScrewState::STOPPED))
		{
			lead_screw_dir = (lead_screw_curr.state == INT(LeadScrewState::STORED)) ? EXTEND : RETRACT;
			stepper_on(actuation_cfg.lead_screw);
		}
	}

	// Negative value - move DOWN, 0 - STOP, positive value - move UP, range - [-1,1]
	void run_actuation(const lunabot_msgs::Actuation &actuation, ros::NodeHandle *nh)
	{
		lead_screw_dir = (actuation.lead_screw > 0) ? EXTEND : RETRACT;
		lead_screw_en = actuation.lead_screw != 0;

		MotorDir angle_dir = (actuation.angle > 0) ? CW : CCW; // CCW retracts, CW extends

		if (actuation.angle != 0)
		{
			write_motor(actuation_cfg.left_lin_act,
						actuation_cfg.left_lin_act.MAX_PWM, angle_dir);
			write_motor(actuation_cfg.right_lin_act,
						actuation_cfg.right_lin_act.MAX_PWM, angle_dir);
		}
		else
		{
			stop_motor(actuation_cfg.left_lin_act);
			stop_motor(actuation_cfg.right_lin_act);
		}

		// nh.logerror("lead screw:");
		if (lead_screw_en)
		{
			stepper_on(actuation_cfg.lead_screw);
		}
		else
		{
			// nh.logerror("STOP");
			stepper_off(actuation_cfg.lead_screw);
		}
	}

}
