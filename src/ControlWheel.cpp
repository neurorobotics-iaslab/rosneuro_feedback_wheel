#include "rosneuro_feedback_wheel/ControlWheel.h"

namespace rosneuro {
	namespace feedback {

ControlWheel::ControlWheel(void) : SingleWheel("controlwheel"), p_nh_("~") {

	this->pub_ = this->nh_.advertise<rosneuro_msgs::NeuroEvent>("/events/bus", 1);
	this->sub_ = this->nh_.subscribe("/integrator/neuroprediction", 1, &ControlWheel::on_received_data, this);

	// Bind dynamic reconfigure callback
	this->recfg_callback_type_ = boost::bind(&ControlWheel::on_request_reconfigure, this, _1, _2);
	this->recfg_srv_.setCallback(this->recfg_callback_type_);
}

ControlWheel::~ControlWheel(void) {}

bool ControlWheel::configure(void) {

	std::vector<float> thresholds;
	
	// Getting wheel range
	ros::param::param("~angle_range", this->angle_range_, 180.0f);

	// Getting thresholds
	if(this->p_nh_.getParam("thresholds", thresholds) == false) {
		ROS_ERROR("Parameter 'thresholds' is mandatory");
		return false;
	} else if(thresholds.size() != 2) {
		ROS_ERROR("Thresholds must be two");
		return false;
	}	
	
	// Getting classes
	if(this->p_nh_.getParam("classes", this->classes_) == false) {
		ROS_ERROR("Parameter 'classes' is mandatory");
		return false;
	} else if(this->classes_.size() != 2) {
		ROS_ERROR("The provided number of classes must be 2, now it is: %ld", this->classes_.size());
		return false;
	}

	// Getting duration parameters
	ros::param::param("~duration/begin", this->duration_.begin, 2000);
	ros::param::param("~duration/boom",  this->duration_.boom, 	1000);
	ros::param::param("~duration/iti", 	 this->duration_.iti,    100);
	ros::param::param("~duration/end", 	 this->duration_.end,   2000);

	// Setting parameters
	this->set_threshold(thresholds.at(0), Direction::Left);
	this->set_threshold(thresholds.at(1), Direction::Right);
	
	return true;
}

void ControlWheel::run(void) {

	int 	  boomevent;
	Direction targethit;
	ros::Rate r(this->rate_);
	this->user_quit_ 	 = false;
	
	// Begin
	ROS_INFO("Protocol started");
	this->sleep(this->duration_.begin);

	while(true) {

		ros::spinOnce();
		this->setevent(Events::CFeedback);
		this->has_new_input_ = false;
		this->current_input_ = 0.5f;
		targethit = Direction::None;
		
		while(ros::ok() && this->user_quit_ == false && targethit == Direction::None) {


			if(this->has_new_input_ == true) {
				this->move(this->input2angle(this->current_input_));
				this->has_new_input_ = false;
			}

			targethit = this->is_over_threshold(this->current_input_);

			if(targethit != Direction::None)
				break;

			ros::spinOnce();
			r.sleep();
		}
		
		this->setevent(Events::CFeedback + Events::Off);
		if(ros::ok() == false || this->user_quit_ == true) break;

		// boom
		boomevent = targethit == Direction::Left ? this->classes_.at(0) : this->classes_.at(1);
		this->setevent(boomevent + Events::Command);
		this->show_boom(targethit);
		this->sleep(this->duration_.boom);
		this->hide_boom();
		this->setevent(boomevent + Events::Command + Events::Off);

		// Inter trial interval
		this->hide_cue();
		this->reset();
		this->sleep(this->duration_.iti);

		if(ros::ok() == false || this->user_quit_ == true) break;
	}

	// End
	ROS_INFO("Protocol ended");
}

void ControlWheel::on_received_data(const rosneuro_msgs::NeuroOutput& msg) {
	int refclass = this->classes_.at(0);
	int refclassIdx;
	bool class_not_found = false;
	std::vector<int> msgclasses = msg.decoder.classes;

	// First: check that the incoming classes are the ones provided
	for(auto it = msgclasses.begin(); it != msgclasses.end(); ++it) {
		auto it2 = std::find(this->classes_.begin(), this->classes_.end(), *it);
		if(it2 == this->classes_.end()) {
			class_not_found = true;
			break;
		}
	}

	// Second: find the index of the refclass
	if(class_not_found == true) {
		this->has_new_input_ = false;
		ROS_WARN_THROTTLE(5.0f, "The incoming neurooutput message does not have the provided classes");
		return;
	}

	auto it = std::find(msgclasses.begin(), msgclasses.end(), refclass);

	if(it != msgclasses.end()) {
		refclassIdx = it - msgclasses.begin();

		this->current_input_ = msg.softpredict.data.at(refclassIdx);
		this->current_angle_ = this->input2angle(msg.softpredict.data.at(refclassIdx));
		this->has_new_input_ = true;
	} else {
		this->has_new_input_ = true;
		ROS_WARN_THROTTLE(5.0f, "Cannot find class %d in the incoming message", refclass);
	}
}


void ControlWheel::setevent(int event) {
	this->event_msg_.header.stamp = ros::Time::now();
	this->event_msg_.event = event;
	this->pub_.publish(this->event_msg_);
}

void ControlWheel::sleep(int msecs) {
	std::this_thread::sleep_for(std::chrono::milliseconds(msecs));
}

ControlWheel::Direction ControlWheel::is_over_threshold(float input) {

	Direction direction = Direction::None;

	if(input >= this->thresholds_.at(0)) {
		direction = Direction::Left;
	} else if(input <= this->thresholds_.at(1)) {
		direction = Direction::Right;
	}

	return direction;
}

void ControlWheel::on_request_reconfigure(config_control_wheel &config, uint32_t level) {

	if( std::fabs(config.left_threshold - this->thresholds_.at(0)) > 0.00001) {
		this->set_threshold(config.left_threshold, Direction::Left);
	}
	
	if( std::fabs(config.right_threshold - this->thresholds_.at(1)) > 0.00001) {
		this->set_threshold(config.right_threshold, Direction::Right);
	}
}

	}
}
