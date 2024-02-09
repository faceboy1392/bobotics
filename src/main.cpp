#include "main.h"
#include "devices.h"
#include "lemlib/api.hpp"
#include "pros/apix.h"

lemlib::Drivetrain drivetrain{
	&left_motors,
	&right_motors,
	13.25, // track width
	lemlib::Omniwheel::NEW_325,
	360, // wheel rpm
	8,	 // "chase power"?
};

lemlib::OdomSensors sensors{
	nullptr,
	nullptr,
	nullptr,
	nullptr,
	&imu};

// forward/backward PID
lemlib::ControllerSettings linearController{
	8.00,  // kP
	0.00,  // kI
	30.00, // kD
	3,	   // anti wind up?
	1,	   // smallErrorRange
	100,   // smallErrorTimeout
	3,	   // largeErrorRange
	500,   // largeErrorTimeout
	5	   // slew rate
};

// turning PID
lemlib::ControllerSettings angularController{
	6.85,  // kP
	0.00,  // kI
	65.00, // kD
	3,	   // anti wind up?
	0.5,   // smallErrorRange
	250,   // smallErrorTimeout
	1.5,   // largeErrorRange
	500,   // largeErrorTimeout
	0	   // slew rate
};

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

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

void auton_close()
{
	chassis.setPose({31, 7, 0});
	// score preload
	chassis.moveToPoint(10, 29, 1800); // in front of goal
	chassis.turnTo(10, 60, 500);	   // turn to goal
	chassis.waitUntilDone();
	intake.move(-127); // outtake
	delay(750);
	intake.brake();
	chassis.turnTo(10, 60, 500, false);		 // turn around
	chassis.moveToPoint(10, 44, 500, false); // reverse into goal
	chassis.moveToPoint(12, 38, 500);		 // back away from goal
	chassis.turnTo(30, 38, 500);			 // turn from wall
	chassis.waitUntilDone();
	chassis.moveToPose(48, 64, 0, 5000, {.maxSpeed = 70}); // move to middle triball
	delay(500);
	intake.move(127); // intake
	chassis.waitUntilDone();
	delay(500);
	intake.brake();
	chassis.moveToPoint(48, 64, 500); // reverse a bit
	chassis.turnTo(48, 0, 800);
	chassis.moveToPose(24, 22, 230, 3000); // move to corner
	chassis.waitUntilDone();
	wings.set_value(true);
	chassis.moveToPose(32, 18, 290, 2000, {.maxSpeed = 40}); // get match load
}

void auton_skills()
{
	chassis.setPose({32, 8, 0});
	chassis.moveToPoint(12, 28, 3000);
	chassis.waitUntilDone();
	intake.move_velocity(-600);
	chassis.turnTo(12, 60, 500);
	chassis.moveToPoint(12, 46, 500);
	chassis.moveToPoint(12, 42, 500, false);
	chassis.turnTo(12, 60, 500, false);
	chassis.waitUntilDone();
	intake.brake();
	// get into kicker position
	chassis.moveToPoint(13, 24, 2000);
	chassis.turnTo(110, 68, 500);
	chassis.waitUntilDone();
	left_motors.move_velocity(-400);
	right_motors.move_velocity(-400);
	delay(250);
	left_motors.brake();
	right_motors.brake();
	wings.set_value(true);
	kicker.move_velocity(92);
	double startTime = pros::millis();
	while (pros::millis() - startTime < 32 * 1000 * 1.0)
	{
		chassis.turnTo(110, 66, 500);
	}
	kicker.brake();
	wings.set_value(false);
	// move to the other side
	chassis.moveToPoint(36, 11, 2000);
	chassis.moveToPoint(112, 11, 3500);
	chassis.turnTo(132, 36, 800, false, 50.0);
	// wings.set_value(true);
	chassis.moveToPose(136, 48, 180, 2000,
					   {.forwards = false, .minSpeed = 127});
	chassis.moveToPoint(129, 22, 500);
	chassis.moveToPoint(136, 48, 1000, false);

	// first frontal push
	chassis.moveToPoint(90, 45, 2000);
	chassis.turnTo(120, 72, 1000, false);
	chassis.waitUntilDone();
	wings.set_value(true);
	chassis.moveToPose(117, 60, 270, 4000,
					   {.forwards = false, .minSpeed = 127});
	chassis.moveToPoint(90, 60, 2000);
	chassis.moveToPose(117, 60, 270, 4000,
					   {.forwards = false, .minSpeed = 127});

	// second frontal push
	chassis.waitUntilDone();
	wings.set_value(false);
	chassis.moveToPose(90, 108, 0, 2000);
	chassis.turnTo(96, 88, 800, false);
	wings.set_value(true);
	chassis.moveToPose(117, 84, 270, 2000,
					   {.forwards = false, .minSpeed = 127});
	chassis.moveToPoint(90, 84, 2000);
	chassis.moveToPoint(117, 84, 2000, false);

	// final frontal push
	chassis.moveToPoint(100, 84, 2000, true);
	chassis.moveToPoint(85, 72, 2000, true);
	chassis.moveToPose(118, 72, 270, 2000,
					   {.forwards = false, .minSpeed = 127});
	chassis.moveToPoint(90, 72, 2000, true);

	// side stuff
	chassis.waitUntilDone();
	wings.set_value(false);
	chassis.turnTo(90, 120, 500);
	chassis.moveToPose(120, 108, 90, 2000);
}

void autonomous()
{
	// auton_skills();
	auton_close();
}

void opcontrol()
{
	// chassis.turnTo(0, -100, 5000);
	// chassis.turnTo(-100, 0, 5000);
	// chassis.moveTo(0, 48, 5000);

	bool wingState = false;
	bool hangState = false;
	while (true)
	{
		// buttons
		if (master.get_digital_new_press(DIGITAL_R1))
			wingState = !wingState;
		if (master.get_digital_new_press(DIGITAL_B))
			hangState = !hangState;

		if (master.get_digital_new_press(DIGITAL_LEFT))
			chassis.turnTo(-100, 0, 5000);
		if (master.get_digital_new_press(DIGITAL_RIGHT))
			chassis.turnTo(0, -100, 5000);

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
