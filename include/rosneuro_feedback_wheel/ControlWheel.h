#ifndef ROSNEURO_FEEDBACK_CONTROLWHEEL_H_
#define ROSNEURO_FEEDBACK_CONTROLWHEEL_H_

#include <ros/ros.h>

#include <rosneuro_msgs/NeuroEvent.h>
#include <rosneuro_msgs/NeuroOutput.h>

#include "rosneuro_feedback_wheel/SingleWheel.h"

namespace rosneuro {
	namespace feedback {

struct Events {
	static const int Start 		= 1;
	static const int Fixation 	= 786;
	static const int CFeedback 	= 781;
	static const int Hit 		= 897;
	static const int Miss 		= 898;
	static const int Off 		= 32768;
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

};

	}
}

#endif
