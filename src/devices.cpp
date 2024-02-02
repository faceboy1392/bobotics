#include "main.h"

Controller master (CONTROLLER_MASTER);

Motor motor_l1 (17, MOTOR_GEAR_BLUE, true, MOTOR_ENCODER_DEGREES);
Motor motor_l2 (20, MOTOR_GEAR_BLUE, true, MOTOR_ENCODER_DEGREES);
Motor motor_l3 (18, MOTOR_GEAR_BLUE, false, MOTOR_ENCODER_DEGREES);
Motor motor_r1 (13, MOTOR_GEAR_BLUE, false, MOTOR_ENCODER_DEGREES);
Motor motor_r2 (11, MOTOR_GEAR_BLUE, false, MOTOR_ENCODER_DEGREES);
Motor motor_r3 (12, MOTOR_GEAR_BLUE, true, MOTOR_ENCODER_DEGREES);

Motor_Group left_motors ({motor_l1, motor_l2, motor_l3});
Motor_Group right_motors ({motor_r1, motor_r2, motor_r3});

Motor kicker (14, MOTOR_GEAR_RED, false, MOTOR_ENCODER_DEGREES);
Motor intake (21, MOTOR_GEAR_BLUE, false, MOTOR_ENCODER_DEGREES);

Imu imu (16);

ADIDigitalOut wings ('a');
ADIDigitalOut hang ('b');