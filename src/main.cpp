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
// lemlib::ControllerSettings angularController{
// 	1.6,  // kP
// 	0.00,  // kI
// 	5.00, // kD
// 	3,	   // anti wind up?
// 	0.5,   // smallErrorRange
// 	250,   // smallErrorTimeout
// 	1.5,   // largeErrorRange
// 	500,   // largeErrorTimeout
// 	0	   // slew rate
// };


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

AutonType selectedAuton = OFFENSIVE;

void auton_selector()
{
	LV_IMG_DECLARE(field);	

	// create an lvgl image at the top left corner of the screen, from a file
	lv_obj_t *img1 = lv_img_create(lv_scr_act(), NULL);
	lv_img_set_src(img1, &field);
	lv_obj_set_pos(img1, 0, 0);

	// text that says "AUTON SKILLS" centered in the top rightmost 240x120 region
	lv_obj_t *label1 = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(label1, "AUTON SKILLS");
	lv_obj_align(label1, NULL, LV_ALIGN_CENTER, 120, -60);

	// similar text in bottom right that says "NONE"
	lv_obj_t *label2 = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(label2, "NONE");
	lv_obj_align(label2, NULL, LV_ALIGN_CENTER, 120, 60);

	// draw a horizontal line to separate them
	lv_obj_t *line1 = lv_line_create(lv_scr_act(), NULL);
	static lv_point_t line1_points[] = {{240, 120}, {480, 120}};
	lv_line_set_points(line1, line1_points, 2);
	lv_line_set_style(line1, &lv_style_plain_color);

	while (true) {
		// screen is 480x240 wxh

		if (screen::touch_status().touch_status == TOUCH_PRESSED) {
			double x = screen::touch_status().x;
			double y = screen::touch_status().y;
			if (x < 120) {
				if (y < 120) {
					selectedAuton = OFFENSIVE;
					break;
				} else {
					selectedAuton = DEFENSIVE;
					break;
				}
			} else if (x < 240) {
				if (y < 120) {
					selectedAuton = DEFENSIVE;
					break;
				} else {
					selectedAuton = OFFENSIVE;
					break;
				}
			} else if (y < 120) {
				selectedAuton = SKILLS;
				break;
			} else {
				selectedAuton = NONE;
				break;
			}
		}

		delay(20);
	}

	// clear screen
	lv_obj_clean(lv_scr_act());
	// text in the middle displaying what auton was selected
	lv_obj_t *label3 = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(label3, "SELECTED AUTON");
	lv_obj_align(label3, NULL, LV_ALIGN_CENTER, 0, -60);
	lv_obj_t *label4 = lv_label_create(lv_scr_act(), NULL);
	switch (selectedAuton) {
		case OFFENSIVE:
			lv_label_set_text(label4, "[ OFFENSIVE ]");
			break;
		case DEFENSIVE:
			lv_label_set_text(label4, "[ DEFENSIVE ]");
			break;
		case SKILLS:
			lv_label_set_text(label4, "[ SKILLS ]");
			break;
		case NONE:
			lv_label_set_text(label4, "[ NONE ]");
			break;
	}
	lv_obj_align(label4, NULL, LV_ALIGN_CENTER, 0, 0);
}

void screen_function()
{
	auton_selector();
	while (true)
	{
		delay(20);
	}
}

void initialize()
{
	Task screen_task(screen_function);
	chassis.calibrate();
	master.rumble(".-");
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
	chassis.moveToPoint(31, 10, 400);
	// chassis.waitUntilDone();
	// wings.set_value(true);
	chassis.moveToPoint(10, 31, 1800); // in front of goal
	// chassis.waitUntilDone();
	// wings.set_value(false);
	chassis.turnTo(10, 60, 500);	   // turn to goal
	chassis.waitUntilDone();
	intake.move(-127); // outtake
	delay(750);
	intake.brake();
	chassis.turnTo(10, 60, 500, false);		 // turn around
	chassis.moveToPoint(10, 44, 500, false); // reverse into goal
	chassis.moveToPoint(12, 36, 500);		 // back away from goal
	chassis.turnTo(30, 38, 500);			 // turn from wall
	chassis.waitUntilDone();
	chassis.moveToPose(48, 64, 0, 5000); // move to middle triball
	delay(500);
	intake.move(127); // intake
	chassis.waitUntilDone();
	delay(500);
	intake.brake();
	chassis.moveToPoint(48, 60, 500); // reverse a bit
	chassis.turnTo(48, 0, 800);
	chassis.moveToPose(22, 22, 230, 3000); // move to corner
	chassis.waitUntilDone();
	wings.set_value(true);
	chassis.moveToPose(32, 16, 110, 2000); // get match load
	chassis.waitUntilDone();
	wings.set_value(false);
	intake.move(-127);
	delay(500);
	intake.brake();
	chassis.turnTo(32, 32, 800);
	chassis.moveToPoint(61, 10, 1500, false);
	delay(1000);
	wings.set_value(true);
}

void auton_far() {
	chassis.setPose({16, 13, 270});
	intake.move(127);
	chassis.moveToPoint(7, 13, 700);
	chassis.moveToPose(67, 49, 180, 4000, {.forwards = false, .minSpeed = 90});
	chassis.waitUntil(10);
	intake.brake();
	chassis.waitUntil(30);
	wings.set_value(true);
	chassis.waitUntil(52);
	wings.set_value(false);
	chassis.waitUntilDone();
	chassis.moveToPose(28, 55, 90, 3000);
	chassis.turnTo(60, 60, 800);
	chassis.waitUntilDone();
	chassis.moveToPoint(52, 65, 1200);
	intake.move(-127);
	chassis.waitUntilDone();
	intake.brake();
	chassis.moveToPoint(30, 55, 1500, false);
	chassis.moveToPoint(9, 51, 2000);
	delay(500);
	intake.move(127);
	chassis.moveToPoint(12, 72, 1500);
	chassis.waitUntilDone();
	chassis.moveToPoint(52, 72, 2000);
	delay(500);
	intake.move(-127);
	chassis.waitUntilDone();
	intake.brake();
	chassis.moveToPoint(24, 65, 1500, false);
	
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
	switch (selectedAuton)
	{
	case OFFENSIVE:
		auton_far();
		break;
	case DEFENSIVE:
		auton_close();
		break;
	case SKILLS:
		auton_skills();
		break;
	case NONE:
		break;
	}
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
