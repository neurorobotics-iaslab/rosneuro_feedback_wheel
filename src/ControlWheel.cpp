#include "rosneuro_feedback_wheel/SingleWheel.h"

namespace rosneuro {

SingleWheel::SingleWheel(void) : p_nh_("~") {


	this->sanalog_ = this->nh_.subscribe("/integrator/neuroprediction", 1, &SingleWheel::on_received_data, this);

	this->srv_reset_ = this->p_nh_.advertiseService("reset", &SingleWheel::on_request_reset, this);
	
	this->engine_ = new neurodraw::Engine("SingleWheel");
	this->engine_->on_keyboard(&SingleWheel::on_keyboard_event, this);

	this->user_quit_ = false;

}

SingleWheel::~SingleWheel(void) {

	if(this->engine_ != nullptr)
		delete this->engine_;

}

bool SingleWheel::configure(void) {


	// Getting parameters from nameserver
	ros::param::param("~angle_min", this->angle_min_, 0.0f);
	ros::param::param("~angle_max", this->angle_max_, 180.0f);

	std::vector<float> th;
	this->p_nh_.getParam("thresholds", th);

	if(th.size() == 2) {
		this->set_threshold(th.at(0), Direction::Left);
		this->set_threshold(th.at(1), Direction::Right);
		this->is_threshold_enabled_ = true;
	}
	

	this->setup_scene();
	
	this->current_angle_ = 90.0f;
	this->has_new_angle_ = false;

	return true;

}

void SingleWheel::setup_scene(void) {

	this->cross_   = new neurodraw::Cross(0.3f, 0.05f);
	this->ring_    = new neurodraw::Ring(0.8f, 0.15f, neurodraw::Palette::grey);
	this->arc_     = new neurodraw::Arc(0.8f, 0.15f, 2.0 * M_PI / 3.0f, neurodraw::Palette::lightgrey);
	this->mline_   = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::green);
	this->rline_   = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::firebrick);
	this->lline_   = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::royalblue);
	this->minline_ = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::dimgray);
	this->maxline_ = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::dimgray);

	this->arc_->rotate(30.0f);
	this->mline_->move(0.0f, 0.725f);
	this->rline_->move(0.0f, 0.725f);
	this->lline_->move(0.0f, 0.725f);
	this->minline_->move(0.0f, 0.725f);
	this->maxline_->move(0.0f, 0.725f);
	this->rline_->rotate(this->input2angle(this->thresholds_.at(1)), 0.0f, 0.0f);
	this->lline_->rotate(this->input2angle(this->thresholds_.at(0)), 0.0f, 0.0f);
	this->minline_->rotate(this->angle_min_, 0.0f, 0.0f);
	this->maxline_->rotate(this->angle_max_, 0.0f, 0.0f);

	this->engine_->add(this->ring_);
	this->engine_->add(this->cross_);
	this->engine_->add(this->arc_);
	this->engine_->add(this->minline_);
	this->engine_->add(this->maxline_);
	this->engine_->add(this->mline_);
	this->engine_->add(this->lline_);
	this->engine_->add(this->rline_);


	if(this->is_threshold_enabled_ == false) {
		this->rline_->hide();
		this->lline_->hide();
	}

}

void SingleWheel::update(float angle) {
	this->arc_->rotate(angle - 60.0f);
	this->mline_->rotate(angle, 0.0f, 0.0f);
}

void SingleWheel::reset(void) {
	this->update(90.0f);
}

void SingleWheel::run(void) {


	ros::Rate r(100);
	
	while(ros::ok() & this->user_quit_ == false) {


		if(this->has_new_angle_ == true) {
			this->update(this->current_angle_);
			this->has_new_angle_ = false;
		}

		ros::spinOnce();
		r.sleep();
	}

}

void SingleWheel::on_received_data(const rosneuro_msgs::NeuroOutput& msg) {

	float angle, input;
	input = msg.softpredict.data.at(0);
	this->current_angle_ = this->input2angle(input);
	this->has_new_angle_ = true;

}

void SingleWheel::on_keyboard_event(const neurodraw::KeyboardEvent& event) {

	if(event.state == 0)
		return;

	switch(event.sym) {
	    case neurodraw::EventKey::ESCAPE:
	   	 this->engine_->quit();
		 this->user_quit_ = true;
	   	 break;
	    case neurodraw::EventKey::a:
	   	 this->update(this->input2angle(1.0f));
	   	 break;
	    case neurodraw::EventKey::s:
	   	 this->update(this->input2angle(0.0f));
	   	 break;
	    case neurodraw::EventKey::r:
	   	 this->update(this->input2angle(0.5f));
	   	 break;
	}
}


bool SingleWheel::on_request_reset(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
	this->reset();
	return true;
}


bool SingleWheel::set_threshold(float value, Direction dir) {


	if(value > this->input_max_ | value < this->input_min_) {
		ROS_ERROR("The provided threshold %f is not in the given input range [%f %f]",
				   value, this->input_min_, this->input_max_);
		return false;
	}


	switch(dir) {
		case Direction::Left:
			this->thresholds_.at(0) = value;
			ROS_INFO("Threshold for Direction::Left changed to: %f", value);
			break;
		case Direction::Right:
			this->thresholds_.at(1) = 1.0f - value;
			ROS_INFO("Threshold for Direction::Right changed to: %f", value);
			break;
	}

	return true;
}

float SingleWheel::input2angle(float input) {

	float b, a, xmax, xmin, angle;

	b    = this->angle_max_;
	a    = this->angle_min_;
	xmax = this->input_max_;
	xmin = this->input_min_;
	
	angle = (b-a) * ( (input - xmin) / (xmax - xmin) ) + a;

	return angle;

}


}
