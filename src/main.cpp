#include "main.h"
#include "devices.h"
#include "lemlib/api.hpp"

lemlib::Drivetrain_t drivetrain{
	&left_motors,
	&right_motors,
	13.25, // track width
	3.25,  // wheel diameter
	360	   // wheel rpm
};

lemlib::OdomSensors_t sensors{
	nullptr,
	nullptr,
	nullptr,
	nullptr,
	&imu};

// forward/backward PID
lemlib::ChassisController_t lateralController{
	8,	 // kP
	30,	 // kD
	1,	 // smallErrorRange
	100, // smallErrorTimeout
	3,	 // largeErrorRange
	500, // largeErrorTimeout
	5	 // slew rate
};

// turning PID
lemlib::ChassisController_t angularController{
	1.28, // kP
	6.00, // kD
	0.5,  // smallErrorRange
	150,  // smallErrorTimeout
	0.5,  // largeErrorRange
	500, // largeErrorTimeout
	0	 // slew rate
};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

using std::abs;
using std::max;
using std::min;

template <typename T>
int sgn(T val)
{
	return (T(0) < val) - (val < T(0));
}

enum AutonType
{
	DEFENSIVE,
	OFFENSIVE,
	SKILLS,
	NONE
};

void auton_selector()
{
  // draw field to the screen
  int ox = 64; // offset x
  screen::set_pen(0x000000);
  // gray background
  screen::draw_rect(ox - 1, -1, 242, 242);
  screen::set_pen(0x282828);
  screen::fill_rect(ox, 0, 240, 240);
  // white tape
  screen::set_pen(0xFFFFFF);
  screen::draw_line(ox + 3, 118, ox + 237, 118);
  screen::draw_line(ox + 3, 122, ox + 237, 122);
  screen::draw_line(ox + 3, 140, ox + 120, 140);
  screen::draw_line(ox + 120, 100, ox + 237, 100);
  // blue goal
  screen::set_pen(0x0000FF);
  screen::fill_rect(ox, 80, 40, 80);
  // red goal
  screen::set_pen(0xFF0000);
  screen::fill_rect(ox + 200, 80, 40, 80);
  // barrier
//   screen.setPenColor(color().black);
//   screen.setPenWidth(5);
//   screen.drawLine(ox + 80, 40, ox + 160, 40);
//   screen.drawLine(ox + 120, 40, ox + 120, 200);
//   screen.drawLine(ox + 80, 200, ox + 160, 200);
//   // red elevation and match load bars
//   screen.setPenColor(color().red);
//   screen.drawLine(ox + 5, 35, ox + 35, 5);
//   screen.drawLine(ox + 5, 205, ox + 35, 235);
//   screen.drawLine(ox + 120, 200, ox + 120, 240);
//   // blue elevation and match load bars
//   screen.setPenColor(color().blue);
//   screen.drawLine(ox + 235, 35, ox + 205, 5);
//   screen.drawLine(ox + 235, 205, ox + 205, 235);
//   screen.drawLine(ox + 120, 0, ox + 120, 40);
}

void screen_function()
{
	while (true)
	{
		delay(20);
	}
}

void initialize()
{
	chassis.calibrate();
	master.rumble(".-");
	Task screen_task(screen_function);
}

void disabled()
{
	master.rumble("---");
}

void competition_initialize()
{
}

void autonomous()
{
}

void opcontrol()
{
	chassis.turnTo(0, -100, 5000);
	bool wingState = false;
	bool hangState = false;
	while (true)
	{
		// buttons
		if (master.get_digital_new_press(DIGITAL_R1))
			wingState = !wingState;
		if (master.get_digital_new_press(DIGITAL_B))
			hangState = !hangState;

		wings.set_value(wingState);
		hang.set_value(hangState);

		if (master.get_digital(DIGITAL_L1))
			intake.move_velocity(600 * 0.9);
		else if (master.get_digital(DIGITAL_L2))
			intake.move_velocity(-600 * 0.9);
		else
			intake.brake();

		if (master.get_digital(DIGITAL_UP))
			kicker.move_velocity(100);
		else
			kicker.brake();

		// split arcade drive
		int dr = master.get_analog(ANALOG_LEFT_Y),
			st = master.get_analog(ANALOG_RIGHT_X);
		st = sgn(st) * pow((double)st / 127.0, 2) * 127.0 * 0.6;

		int left = dr + st,
			right = dr - st,
			m = max(127, max(abs(left), abs(right)));
		left /= m / 127.0;
		right /= m / 127.0;

		left_motors.move(left);
		right_motors.move(right);

		delay(20);
	}
}
