#include "main.h"
#include "devices.h"

using std::abs;
using std::max;
using std::min;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
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

		pros::delay(20);
	}
}
