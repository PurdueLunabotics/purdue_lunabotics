#include "actuation.h"

namespace actuation
{

	static StepperDir lead_screw_dir;
	static uint8_t lead_screw_en;

	Stepper lead_screw_stepper(actuation_cfg.lead_screw.steps,
							   actuation_cfg.lead_screw.DIR1_pin,
							   actuation_cfg.lead_screw.DIR2_pin);

	HallSensor lead_screw_hall = {.state = INT(LeadScrewState::STORED), .init_state = INT(LeadScrewState::STORED)};
	HallSensor lin_act_hall = {.state = INT(LinActState::STORED, .init_state = INT(LeadScrewState::STORED))};

	void lin_act_hall_cb()
	{
		if (digitalRead(LIN_ACT_HALL_PIN) == LOW)
		{
			lin_act_hall.state = (lin_act_hall.state + 1) % INT(LinActState::CNT);
			stop_motor(actuation_cfg.left_lin_act);
			stop_motor(actuation_cfg.right_lin_act);
		}
	}

	// Lead Screw Actuation

	void lead_screw_hall_cb()
	{
		if (digitalRead(LEAD_SCREW_HALL_PIN) == LOW)
		{
			lead_screw_hall.state = (lead_screw_hall.state + 1) % INT(LeadScrewState::CNT);
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
		init_hall(LEAD_SCREW_HALL_PIN, lead_screw_hall_cb, &lead_screw_hall);
		init_hall(LIN_ACT_HALL_PIN, lin_act_hall_cb, &lin_act_hall);
	}

	void stepper_step()
	{
		if (lead_screw_en)
		{
			stepper_step(actuation_cfg.lead_screw, &lead_screw_stepper, lead_screw_dir);
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
			if ((lin_act_hall.state == INT(LinActState::STORED) || lin_act_hall.state == INT(LinActState::DRIVING)) && angle_dir == CCW)
			{

				stop_motor(actuation_cfg.left_lin_act);
				stop_motor(actuation_cfg.right_lin_act);
			}
			else if (lin_act_hall.state == INT(LinActState::FULL_EXT) && angle_dir == CW)
			{

				stop_motor(actuation_cfg.left_lin_act);
				stop_motor(actuation_cfg.right_lin_act);
			}
			else {
				write_motor(actuation_cfg.left_lin_act,
							actuation_cfg.left_lin_act.MAX_PWM, angle_dir);
				write_motor(actuation_cfg.right_lin_act,
							actuation_cfg.right_lin_act.MAX_PWM, angle_dir);
			}
		}
		else
		{
			stop_motor(actuation_cfg.left_lin_act);
			stop_motor(actuation_cfg.right_lin_act);
		}

		// nh.logerror("lead screw:");
		if (lead_screw_en)
		{

			if (lead_screw_hall.state == INT(LeadScrewState::STORED) && lead_screw_dir == RETRACT)
			{
				stepper_off(actuation_cfg.lead_screw);
			}
			else if (lead_screw_hall.state == INT(LeadScrewState::FULL_EXT) && lead_screw_dir == EXTEND)
			{
				stepper_off(actuation_cfg.lead_screw);
			}
			else {
				// nh.logerror("ON");
				stepper_on(actuation_cfg.lead_screw);
			}
		}
		else
		{
			// nh.logerror("STOP");
			stepper_off(actuation_cfg.lead_screw);
		}
	}

}
