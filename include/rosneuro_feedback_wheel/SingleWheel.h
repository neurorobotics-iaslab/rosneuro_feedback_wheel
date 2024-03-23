#ifndef ROSNEURO_FEEDBACK_SINGLEWHEEL_H_
#define ROSNEURO_FEEDBACK_SINGLEWHEEL_H_

#include <ros/ros.h>

#include <neurodraw/Engine.h>
#include <neurodraw/Ring.h>
#include <neurodraw/Arc.h>
#include <neurodraw/Cross.h>
#include <neurodraw/Rectangle.h>
#include <neurodraw/Triangle.h>
#include <neurodraw/Circle.h>
#include <neurodraw/Palette.h>
#include <neurodraw/EventKey.h>

namespace rosneuro {
	namespace feedback {

const std::array<neurodraw::Color, 4> CuePalette { 
		neurodraw::Palette::royalblue, 
		neurodraw::Palette::firebrick, 
		neurodraw::Palette::orange,
		neurodraw::Palette::darkgray
};

class SingleWheel {
	
	public:
		enum class Direction {Left = 0, Right, Forward, Timeout, None};

	public:
		SingleWheel(const std::string& wintitle = "neurowheel", int fps = 24);
		virtual ~SingleWheel(void);

		void setup(void);
		void move(float angle);
		void reset(void);
		bool set_threshold(float input, Direction dir);
		bool set_angle_range(float angle);

		void show_fixation(void);
		void show_cue(Direction dir);
		void show_boom(Direction dir);
		void show_thresholds(void);
		void hide_fixation(void);
		void hide_cue(void);
		void hide_boom(void);
		void hide_thresholds(void);
		
		virtual bool configure(void) = 0;
		virtual void run(void) = 0;

	protected:
		virtual void on_keyboard_event(const neurodraw::KeyboardEvent& event);
		float input2angle(float input);
		

	protected:
		// Graphic elements
		neurodraw::Engine* 		engine_;
		neurodraw::Ring* 		ring_;
		neurodraw::Arc* 		arc_;
		neurodraw::Cross* 		cross_;
		neurodraw::Circle*		circle_;
		neurodraw::Rectangle* 	mline_;
		neurodraw::Rectangle* 	lline_;
		neurodraw::Rectangle* 	rline_;
		neurodraw::Rectangle* 	minline_;
		neurodraw::Rectangle* 	midline_;
		neurodraw::Rectangle* 	maxline_;

		// Default configuration
		const float input_min_ 			 = 0.0f;
		const float input_max_ 			 = 1.0f;
		float angle_range_ 	   			 = 180.0f;
		std::array<float, 2> thresholds_;

		float current_angle_;
		bool user_quit_;

		

};

 }

}

#endif
