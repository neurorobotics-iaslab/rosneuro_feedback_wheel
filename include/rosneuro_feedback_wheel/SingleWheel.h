#ifndef ROSNEURO_FEEDBACK_SINGLEWHEEL_H_
#define ROSNEURO_FEEDBACK_SINGLEWHEEL_H_

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <rosneuro_msgs/NeuroOutput.h>

#include <neurodraw/Engine.h>
#include <neurodraw/Ring.h>
#include <neurodraw/Arc.h>
#include <neurodraw/Cross.h>
#include <neurodraw/Rectangle.h>
#include <neurodraw/Palette.h>
#include <neurodraw/EventKey.h>

namespace rosneuro {

class SingleWheel {

	public:
		SingleWheel(void);
		~SingleWheel(void);

		bool configure(void);

		void run(void);
		void update(float angle);
		void reset(void);


	protected:
		void on_keyboard_event(const neurodraw::KeyboardEvent& event);
		void on_received_data(const rosneuro_msgs::NeuroOutput& msg);
		bool on_request_reset(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

	protected:
		void setup_scene(void);
		float input2angle(float input);

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle p_nh_;
		ros::Subscriber sanalog_;
		ros::Subscriber sdiscrete_;
		ros::Subscriber	sevent_;
		ros::ServiceServer 	srv_reset_;


		neurodraw::Engine* 		engine_;
		neurodraw::Ring* 		ring_;
		neurodraw::Arc* 		arc_;
		neurodraw::Cross* 		cross_;
		neurodraw::Rectangle* 	mline_;

		float input_min_;
		float input_max_;
		float angle_min_;
		float angle_max_;

		float current_angle_;
		bool has_new_angle_;
		bool user_quit_;

};



}

#endif
