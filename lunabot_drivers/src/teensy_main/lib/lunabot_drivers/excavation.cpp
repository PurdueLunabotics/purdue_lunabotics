#include "excavation.h"

namespace excavation
{
	HX711 scale;
	IntervalTimer exc_timer;


	CurrentSensor exc_current = {.current_value = 0, .max_value = 511};
	ExcFeedback exc_feedback = {.calibration_factor = 10000, .max_wt = 12.0, .current_wt = 0.0, .gain = 128,.bin_state = INT(BinState::EMPTY), .exc_state = INT(ExcState::NOMINAL) };

	void exc_sensors_cb()
	{
		exc_feedback.current_wt = scale.get_value();
		if (exc_feedback.current_wt < 0.01)
		{
			exc_feedback.bin_state = INT(BinState::EMPTY);
		}
		else if (exc_feedback.max_wt - exc_feedback.current_wt > 0)
		{
			exc_feedback.bin_state = INT(BinState::FILLING);
		}
		else
		{
			exc_feedback.bin_state = INT(BinState::FULL);
			stop_motor(excavation_cfg.exc);
		}

		exc_current.current_value = analogRead(EXC_CURRENT_PIN);
		if (exc_current.current_value >= exc_current.max_value)
		{
			exc_feedback.exc_state = INT(ExcState::OVERCURRENT);
			stop_motor(excavation_cfg.exc);
		}
	}

	void init_load_cell()
	{

		scale.begin(EXC_LOAD_DATA_PIN, EXC_LOAD_CLK_PIN, exc_feedback.gain);
		scale.tare();

	}

	void init()
	{
		// Excavation motor
		init_motor(excavation_cfg.exc);
		stop_motor(excavation_cfg.exc);
		init_load_cell();
		exc_timer.begin(exc_sensors_cb, EXC_UPDATE_PERIOD);

		// Current sensor
		pinMode(EXC_CURRENT_PIN, INPUT);
	}

	void run_excavation(const std_msgs::Float32 &speed, ros::NodeHandle *nh)
	{
		unsigned int excavate_speed = abs(map(speed.data, -1, 1, -255, 255)); // Range from [-255,255]
		MotorDir excavate_dir = (speed.data > 0) ? CCW : CW;
		// nh.logerror("Excavation:");
		// nh.logerror(String(excavate_speed != 0).c_str());

		if (exc_feedback.bin_state == INT(BinState::FULL) || exc_feedback.exc_state == INT(ExcState::OVERCURRENT))
		{
			return;
		}

		if (excavate_speed != 0)
		{
			write_motor(excavation_cfg.exc, excavate_speed, excavate_dir);
		}
		else
		{
			stop_motor(excavation_cfg.exc);
		}
	}
}
