#include "excavation.h"

namespace excavation
{
	HX711 scale;
	IntervalTimer scale_timer;

	void exc_load_cell_cb()
	{
		exc_load.current_wt = scale.get_value();
		if (exc_load.current_wt < 0.01)
		{
			exc_load.state = EMPTY;
		}
		else if (exc_load.max_wt - exc_load.current_wt > 0)
		{
			exc_load.state = FILLING;
		}
		else
		{
			exc_load.state = FULL;
			stop_motor(excavation_cfg.exc);
		}
	}

	void init_load_cell() {
		exc_load = { .calibration_factor = 10000,.max_wt=12.0, 
				     .current_wt = 0.0, .gain=128, .state = EMPTY };
		scale.begin(EXC_LOAD_DATA_PIN,EXC_LOAD_CLK_PIN,exc_load.gain);
		scale.tare();
	}

	void init()
	{
		// set all pwm and direction pins to output
		init_motor(excavation_cfg.exc);
		stop_motor(excavation_cfg.exc);
		init_load_cell();

		scale_timer.begin(exc_load_cell_cb,EXC_UPDATE_PERIOD);
	}

	void run_excavation(const std_msgs::Float64 &speed, ros::NodeHandle *nh)
	{
		unsigned int excavate_speed = abs(map(speed.data, -1, 1, -255, 255)); // Range from [-255,255]
		MotorDir excavate_dir = (speed.data > 0) ? CCW : CW;
		// nh.logerror("Excavation:");
		// nh.logerror(String(excavate_speed != 0).c_str());

		if(exc_load.state == FULL) {
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
