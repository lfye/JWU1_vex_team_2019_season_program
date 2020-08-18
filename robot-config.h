#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

vex::brain Brain;

//  Initiialize the Wheel motors
vex::motor motorLU = vex::motor(vex::PORT14);
vex::motor motorLD = vex::motor(vex::PORT13);
vex::motor motorRU = vex::motor(vex::PORT11);
vex::motor motorRD = vex::motor(vex::PORT12);

//  Initiialize the Lift motors
vex::motor motorLiftL = vex::motor(vex::PORT4);
vex::motor motorLiftR = vex::motor(vex::PORT3);

//  Initiialize the Intake/Claw motors
vex::motor motorClawRotate = vex::motor(vex::PORT7);
vex::motor motorIntakeL = vex::motor(vex::PORT8);
vex::motor motorIntakeR = vex::motor(vex::PORT9);

// Initialize the sonar sensors, if we choose to use them
//vex::sonar LeftSonar_IN = vex::sonar(Brain.ThreeWirePort.C);
//vex::sonar LeftSonar_OUT = vex::sonar(Brain.ThreeWirePort.D);
//vex::sonar RightSonar_IN = vex::sonar(Brain.ThreeWirePort.G);
//vex::sonar RightSonar_OUT = vex::sonar(Brain.ThreeWirePort.H);

vex::pot LiftPot = vex::pot(Brain.ThreeWirePort.B);

vex::limit Intake_Limit = vex::limit(Brain.ThreeWirePort.E);

//  Initialize the gyroscope
vex::gyro gyroscope = vex::gyro(Brain.ThreeWirePort.A);

vex::digital_out rotationAid = vex::digital_out(Brain.ThreeWirePort.F);

vex::controller Controller1 = vex::controller();

const int SLEW_HANDLER_DELAY_MS = 15;
const int SLEW_TOP_SPEED_TIME_MS = 75;	// Time (ms) to go from 0 to MAX_SPEED
const int WHEEL_MAX_SPEED = 100;
const int WHEEL_MIN_SPEED = 5;
const int SLEW_RATE = (SLEW_HANDLER_DELAY_MS * WHEEL_MAX_SPEED) / SLEW_TOP_SPEED_TIME_MS;
const int MOTOR_AMOUNT = 4;
const int DEGREE_CIRCUMFERENCE_RATIO = 1.582*round(360 / 32); //360 degrees / 33cm
const float VEL_REDUCE_PERC = 0.97;
const float VEL_REDUCE_PERC_OVERHEAT = 0.6;
const int LINE_FOLLOWER_TRESHOLD = 50;
const float INCREASE_PERC_WHEN_CENTER_IS_WHITE = 1.75;
const float INCREASE_PERC_WHEN_CENTER_IS_BLACK = 2;
const int LOCK_VEL = 20;


const double LIFT_POTEN_MAX_VALUE = 140;
const double LIFT_POTEN_MID_VALUE = 98;
const double LIFT_POTEN_MIN_VALUE = 15;


const double IN_TAKE_ROTATE_MIN_VALUE = 0;
const double IN_TAKE_ROTATE_MAX_VALUE = 950;

//	Array to hold the requested values to the wheel motors
//	The task motorSlewHandler will read the values from here
int motorRequest[MOTOR_AMOUNT];
		// 	The order in which the motors will be process
		//	and the mapping for the array is the following:
		const int LU_INDEX = 0; //	index 0 -> LUWheel
		const int LD_INDEX = 1; //	index 1 -> LDWheel
		const int RU_INDEX = 2;	//	index 2 -> RUWheel
		const int RD_INDEX = 3; //	index 3 -> RDWheel

vex::motor* FIRST_MOTOR = &motorLU;