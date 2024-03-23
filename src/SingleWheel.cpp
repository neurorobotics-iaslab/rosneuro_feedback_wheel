#include "rosneuro_feedback_wheel/SingleWheel.h"

namespace rosneuro {
	namespace feedback {

SingleWheel::SingleWheel(const std::string& wintitle, int fps) {
	
	this->engine_ = new neurodraw::Engine(wintitle, fps);
	this->engine_->on_keyboard(&SingleWheel::on_keyboard_event, this);

	this->user_quit_ = false;

	this->setup();

}

SingleWheel::~SingleWheel(void) {

	if(this->engine_ != nullptr)
		delete this->engine_;
}


void SingleWheel::setup(void) {

	this->cross_   = new neurodraw::Cross(0.3f, 0.05f);
	this->ring_    = new neurodraw::Ring(0.8f, 0.15f, neurodraw::Palette::grey);
	this->arc_     = new neurodraw::Arc(0.8f, 0.15f, 2.0 * M_PI / 3.0f, neurodraw::Palette::lightgrey);
	this->mline_   = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::darkgreen);
	this->rline_   = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::firebrick);
	this->lline_   = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::royalblue);
	this->minline_ = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::dimgray);
	this->midline_ = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::dimgray);
	this->maxline_ = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::dimgray);
	this->circle_  = new neurodraw::Circle(0.15f, true, neurodraw::Palette::white);

	this->arc_->rotate(30.0f);
	this->mline_->move(0.0f, 0.725f);
	this->rline_->move(0.0f, 0.725f);
	this->lline_->move(0.0f, 0.725f);
	this->minline_->move(0.0f, 0.725f);
	this->midline_->move(0.0f, 0.725f);
	this->maxline_->move(0.0f, 0.725f);
	this->rline_->rotate(this->input2angle(0.0f), 0.0f, 0.0f);
	this->lline_->rotate(this->input2angle(0.0f), 0.0f, 0.0f);
	this->minline_->rotate(this->angle_range_/2.0f - 90.0f, 0.0f, 0.0f);
	this->maxline_->rotate(this->angle_range_/2.0f + 90.0f, 0.0f, 0.0f);

	this->engine_->add(this->ring_);
	this->engine_->add(this->cross_);
	this->engine_->add(this->arc_);
	this->engine_->add(this->minline_);
	this->engine_->add(this->midline_);
	this->engine_->add(this->maxline_);
	this->engine_->add(this->lline_);
	this->engine_->add(this->rline_);
	this->engine_->add(this->mline_);
	this->engine_->add(this->circle_);

	this->rline_->hide();
	this->lline_->hide();
	this->cross_->hide();
	this->circle_->hide();

	this->set_threshold(this->input_max_, Direction::Left);
	this->set_threshold(this->input_min_, Direction::Right);
	this->show_thresholds();
}

void SingleWheel::move(float angle) {
	this->arc_->rotate(angle - 60.0f);
	this->mline_->rotate(angle, 0.0f, 0.0f);
}

void SingleWheel::reset(void) {
	this->move(90.0f);
}

void SingleWheel::show_fixation(void) {
	this->cross_->show();
}

void SingleWheel::show_cue(Direction dir) {

	neurodraw::Color color = neurodraw::Palette::dimgray;

	switch(dir) {
		case Direction::Left:
			color = CuePalette.at(0);
			break;
		case Direction::Right:
			color = CuePalette.at(1);
			break;
		case Direction::Forward:
			color = CuePalette.at(2);
			break;
		case Direction::None:
			color = CuePalette.at(3);
			break;
		default:
			ROS_WARN("Unknown direction required. Cue color is not set");
			break;
	}
	this->circle_->set_color(color);
	this->circle_->show();
}

void SingleWheel::show_boom(Direction dir) {
	
	neurodraw::Color color = neurodraw::Palette::lightgrey;

	switch(dir) {
		case Direction::Left:
			color = CuePalette.at(0);
			break;
		case Direction::Right:
			color = CuePalette.at(1);
			break;
		case Direction::Forward:
			color = CuePalette.at(2);
			break;
		case Direction::Timeout:
			color = CuePalette.at(3);
			break;
		default:
			break;
	}
	this->arc_->set_color(color);
}

void SingleWheel::hide_boom(void) {
	this->arc_->set_color(neurodraw::Palette::lightgrey);
}

void SingleWheel::show_thresholds(void) {
	this->lline_->show();
	this->rline_->show();
}

void SingleWheel::hide_fixation(void) {
	this->cross_->hide();
}

void SingleWheel::hide_cue(void) {
	this->circle_->hide();
}

void SingleWheel::hide_thresholds(void) {
	this->lline_->hide();
	this->rline_->hide();
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
	   	 	this->move(this->input2angle(this->thresholds_.at(0)));
			this->show_boom(Direction::Left);
	   	 	break;
	    case neurodraw::EventKey::d:
	   	 	this->move(this->input2angle(this->thresholds_.at(1)));
			this->show_boom(Direction::Right);
	   	 	break;
	    case neurodraw::EventKey::s:
	   	 	this->move(this->input2angle((this->input_max_ + this->input_min_)/2.0f));
			this->hide_boom();
	   	 	break;
	    case neurodraw::EventKey::q:
			this->show_cue(Direction::Left);
	   	 	break;
	    case neurodraw::EventKey::w:
			this->show_cue(Direction::Forward);
	   	 	break;
	    case neurodraw::EventKey::e:
			this->show_cue(Direction::Right);
	   	 	break;
	    case neurodraw::EventKey::r:
			this->hide_cue();
	   	 	break;
	    case neurodraw::EventKey::f:
			this->show_fixation();
	   	 	break;
	    case neurodraw::EventKey::g:
			this->hide_fixation();
	   	 	break;
	}
}

bool SingleWheel::set_threshold(float input, Direction dir) {

	float input_c;
	float dist_c;
	float threshold;

	if(input > this->input_max_ | input < this->input_min_) {
		ROS_ERROR("[SingleWheel] The provided threshold %f is not in the given input range [%f %f]",
				   input, this->input_min_, this->input_max_);
		return false;
	}

	input_c = (this->input_max_ + this->input_min_)/2.0f;
	dist_c  = std::fabs(input_c-input);

	switch(dir) {
		case Direction::Left:
			threshold = input_c + dist_c;
			this->thresholds_.at(0) = threshold;
			this->lline_->rotate(this->input2angle(threshold), 0.0f, 0.0f);
			ROS_INFO("[SingleWheel] Threshold for Direction::Left changed to: %f", threshold);
			break;
		case Direction::Right:
			threshold = input_c - dist_c;
			this->thresholds_.at(1) = threshold;
			this->rline_->rotate(this->input2angle(threshold), 0.0f, 0.0f);
			ROS_INFO("[SingleWheel] Threshold for Direction::Right changed to: %f", threshold);
			break;
		case Direction::Forward:
			ROS_WARN("[SingleWheel] Threshold for Direction::Forward is not available");
			break;
		default:
			ROS_ERROR("[SingleWheel] The provided Direction is unknown. Threshold is not set");
			break;
	}
	
	return true;
}

bool SingleWheel::set_angle_range(float angle) {

	if(angle > 360.0f | angle < 0.0f) {
		ROS_ERROR("[SingleWheel] The provided angle %f is not in the allowed angle range [0.0 360.0]", angle);
		return false;
	}

	this->angle_range_ = angle;
	this->minline_->rotate(this->angle_range_/2.0f - 90.0f, 0.0f, 0.0f);
	this->maxline_->rotate(this->angle_range_/2.0f + 90.0f, 0.0f, 0.0f);
	ROS_INFO("[SingleWheel] Angle range changed to: %f", angle);

	return true;
}

float SingleWheel::input2angle(float input) {

	float b, a, xmax, xmin, angle;

	b    = this->angle_range_;
	a    = 0.0f;
	xmax = this->input_max_;
	xmin = this->input_min_;
	
	angle = (b-a) * ( (input - xmin) / (xmax - xmin) ) + a;

	return angle;

}



 }


}
