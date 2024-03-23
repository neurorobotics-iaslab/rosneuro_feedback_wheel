#ifndef ROSNEURO_FEEDBACK_CONTROLWHEEL_H_
#define ROSNEURO_FEEDBACK_CONTROLWHEEL_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <rosneuro_msgs/NeuroEvent.h>
#include <rosneuro_msgs/NeuroOutput.h>

#include "rosneuro_feedback_wheel/SingleWheel.h"
#include "rosneuro_feedback_wheel/ControlWheelConfig.h"

namespace rosneuro {
	namespace feedback {

struct Events {
	static const int Start 		= 1;
	static const int Fixation 	= 786;
	static const int CFeedback 	= 781;
	static const int Hit 		= 897;
	static const int Miss 		= 898;
	static const int Off 		= 32768;
	static const int Command 	= 6000;
};

struct Duration {
	int begin;
	int start;
	int fixation;
	int cue;
	int feedback_min;
	int feedback_max;
	int boom;
	int timeout;
	int timeout_on_rest;
	int iti;
	int end;
};

using config_control_wheel = rosneuro_feedback_wheel::ControlWheelConfig;
using dyncfg_control_wheel = dynamic_reconfigure::Server<config_control_wheel>;

class ControlWheel : public SingleWheel {
	

	public:
		ControlWheel(void);
		~ControlWheel(void);

		bool configure(void);
		void run(void);

	protected:
		void setevent(int event);
		void sleep(int msecs);
		Direction class2direction(int eventcue);
		Direction is_over_threshold(float input);
		void on_received_data(const rosneuro_msgs::NeuroOutput& msg);
		void on_request_reconfigure(config_control_wheel &config, uint32_t level);

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle p_nh_;
		ros::Subscriber sub_;
		ros::Publisher	pub_;

		rosneuro_msgs::NeuroEvent  event_msg_;
		rosneuro_msgs::NeuroOutput inputmsg_;

		std::vector<int> classes_;
		Duration duration_;
		
		float current_input_;
		bool has_new_input_;
		const float rate_ = 100.0f;
		
		dyncfg_control_wheel recfg_srv_;
  		dyncfg_control_wheel::CallbackType recfg_callback_type_;

};

	}
}

#endif
