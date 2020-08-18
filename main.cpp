/*
 *  JWU1 Programming Code
 */

#include "robot-config.h"

//////////////////////////////////////////////////
////            HELPER FUNCTIONS            //////
//////////////////////////////////////////////////

// sets left side chassis wheel motors to requested vel
void leftMotors(int vel)
{
	motorRequest[LU_INDEX] = vel;
	motorRequest[LD_INDEX] = -vel;
}

// sets right chassis wheel motors to requested vel
void rightMotors(int vel)
{
	motorRequest[RU_INDEX] = -vel;
	motorRequest[RD_INDEX] = vel;
}

// gives helper chassis functions vel to move forward at
void moveForward(int vel)
{
	leftMotors(vel);
	rightMotors(-vel);
}

// gives helper chassis functions vel to rotate at
void moveSide(int vel)
{
	leftMotors(vel);
	rightMotors(vel);
}

// locks all of the chassis wheels
void lockWheels()
{
	motorRequest[LD_INDEX] = -LOCK_VEL;
	motorRequest[RD_INDEX] = -LOCK_VEL;
	motorRequest[LU_INDEX] = LOCK_VEL;
	motorRequest[RU_INDEX] = LOCK_VEL;
}

// Takes a velocity and a direction in which to rotate the intake motors 
void startInTake(int vel, bool isReversed){
    //if the velocity is negative, make it positive for processing purposes
    if (vel < 0) vel *= -1;
    
    if (isReversed == true){
        motorIntakeL.spin(vex::directionType::fwd, -vel, vex::velocityUnits::pct);
        motorIntakeR.spin(vex::directionType::fwd, -vel, vex::velocityUnits::pct);
    } else {
        motorIntakeL.spin(vex::directionType::fwd, vel, vex::velocityUnits::pct);
        motorIntakeR.spin(vex::directionType::fwd, vel, vex::velocityUnits::pct);
    }
}

void startInTake(int vel){
    startInTake(vel, false);
}

// This function stops the intake motors and determines which brake mode to use:
// hold: motor actively resists any force applied to shaft in order to keep 
// 	     shaft from rotating from its current angle 
// coast: current being sent to the motor is cut completely, meaning that any
//        force applied to shaft will cause it to "coast" or rotate freely 
void stopInTake(bool holdIt){
    if (holdIt == true){
        motorIntakeL.stop(vex::brakeType::hold);
        motorIntakeR.stop(vex::brakeType::hold);
    } else{
        motorIntakeL.stop(vex::brakeType::coast);
        motorIntakeR.stop(vex::brakeType::coast);
    }
}

// stops intake motors with brake mode set to coast
void stopInTake(){
    stopInTake(false);
}

int minVel(int vel)
//	Make sure that the value of the velocity is not less than the
//	MIN_VEL to avoid setting the motors to a very small value
{
    //	Return the vel only when the it's higher than the minimum
    if(abs(vel) >= WHEEL_MIN_SPEED)
			return vel;
	//	Otherwise, return MIN_SPEED
	if(vel > 0)
		return WHEEL_MIN_SPEED;
	else
		return -WHEEL_MIN_SPEED;
}

void processMoveForward(int vel){
	// Compares the absolute values of how far the wheels
	// on each side of the bot have rotated.
	// If one side is rotating faster than the other,
	// reduce the velocity being sent to the motors on that side.
	// This is done every loop in order to ensure the robot moves straight
    if(abs(motorLD.rotation(vex::rotationUnits::deg)) > (abs(motorRD.rotation(vex::rotationUnits::deg)))){
       leftMotors(VEL_REDUCE_PERC * vel);
       rightMotors(-vel);
    } else if (abs(motorRD.rotation(vex::rotationUnits::deg)) > abs(motorLD.rotation(vex::rotationUnits::deg))){
       rightMotors(-(VEL_REDUCE_PERC * vel));
       leftMotors(vel);
    } else {
        moveForward(vel);
    }
}
               
//  This function is for the slew handler.
//  It will add/substract the slew rate to the current vel
//  depending upon the requested velocity and return it
int processMotorVel(int currentVel, int requestedVel)
{
	//	Check if we need to update the velocity of the current motor
	if (currentVel != requestedVel)

	//	Check if we have to add or subtract to the velocity
	if (currentVel < requestedVel)
	{
		//	If lower, then ADD to the current vel
		currentVel += SLEW_RATE;

		//	Make sure to not pass the requested value
		if (currentVel > requestedVel) currentVel = requestedVel;
	}
	else
	{
		//	If greater, then SUBTRACK to the current vel
		currentVel -= SLEW_RATE;

		//	Make sure to not pass the requested value
		if (currentVel < requestedVel) currentVel = requestedVel;
	}

	//	Return the processed velocity
	return currentVel;
}

//  Function that returns the next motor in the sequence 
//  in order for the slew handler to use it
vex::motor* getNextMotor(int currentMotor)
{
    //  The order goes: LU, LD, RU, and RD
    //  currentMotor can only be 0-3
	switch(currentMotor)
	{
	case LU_INDEX:
		return &motorLD;
	case LD_INDEX:
		return &motorRU;
	case RU_INDEX:
		return &motorRD;
	case RD_INDEX:
		return &motorLU;
	}

	//	If not in the switch statement, then return NULL
	//	Also makes the compiler happy since it's expecting a return value
	return NULL;
}


//////////////////////////////////////////////////
////        END OF HELPER FUNCTIONS         //////

//////////////////////////////////////////////////
////              CUSTOM API                //////
//////////////////////////////////////////////////

void flipTurningPoint(int vel){
    double initValue = motorClawRotate.rotation(vex::rotationUnits::deg);
    double degrees;
    
    //  Make sure that vel is positive
    if (vel < 0) vel *= -1;
    
    if (initValue > IN_TAKE_ROTATE_MAX_VALUE / 2){
        //  go down to the minimun value
        degrees = initValue;
        while(degrees > IN_TAKE_ROTATE_MIN_VALUE ){
            motorClawRotate.spin(vex::directionType::fwd, -vel, vex::velocityUnits::pct);
            
            vex::task::sleep(10);
            
            degrees = motorClawRotate.rotation(vex::rotationUnits::deg);
        }
        
        motorClawRotate.stop(vex::brakeType::brake);
    } else {
        //  go up to the max value
        degrees = initValue;
        while(degrees < IN_TAKE_ROTATE_MAX_VALUE){
            motorClawRotate.spin(vex::directionType::fwd, vel, vex::velocityUnits::pct);
            
            vex::task::sleep(10);
            
            degrees = motorClawRotate.rotation(vex::rotationUnits::deg);
        }
        motorClawRotate.stop(vex::brakeType::brake);
    }
}

// drives a given distance at a given velocity
void driveDistanceCM(int vel, int dis_cm){
    int tickGoal = DEGREE_CIRCUMFERENCE_RATIO * dis_cm;
    int encValue = 0;
    bool reduceVel = false;
    
    motorLD.resetRotation();
    motorRD.resetRotation();
    
    while(encValue < tickGoal){
        processMoveForward(vel);
        vex::task::sleep(15);
        encValue = ( abs(motorLD.rotation(vex::rotationUnits::deg)) + abs(motorRD.rotation(vex::rotationUnits::deg)))  / 2;
        //	If we have traveled 75 % of the goal distance,
		//  then slow down the motors by 50 % (divide by 2).
        if(encValue >= tickGoal * 0.75 && !reduceVel){
            reduceVel = true;
            vel /= 2;
        }
    }
    //stop moving
    lockWheels();
}

// drives a given distance at a given velocity
// if this takes longer than the end time argument, stop this task
void driveDistanceCM(int vel, int dis_cm, int END_TIME_MS){
    double tickGoal = DEGREE_CIRCUMFERENCE_RATIO * dis_cm;
    double encValue = 0;
    bool reduceVel = false;
    
    motorLD.resetRotation();
    motorRD.resetRotation();
    
    // Clear the time
    Brain.resetTimer();
    
    while(encValue < tickGoal && Brain.timer(vex::timeUnits::msec) < END_TIME_MS){
        processMoveForward(vel);
        
        vex::task::sleep(15);
        
        encValue = ( abs(motorLD.rotation(vex::rotationUnits::deg)) + abs(motorRD.rotation(vex::rotationUnits::deg)))  / 2;
        
        //	If we have traveled 75 % of the goal distance,
		//  then slow down the motors by 50 % (divide by 2).
        if(encValue >= tickGoal * 0.75 && !reduceVel){
            reduceVel = true;
            vel /= 2;
        }
    }
    //stop moving
    lockWheels();
    moveForward(0);
    vex::task::sleep(200);
}


void rotateDegrees10(int vel, double degrees, int END_TIME_MS){
    //  Get the initial value of the Gyro at the beginning of the func
    double initGyroValue = gyroscope.value(vex::rotationUnits::deg);
    double degreesTraveled = 0;
    
    if (degrees < 0) degrees *= -1;
        
    // Clear the time
    Brain.resetTimer();
    
    //  While the traveled degrees are less than the end degrees, rotate
    while(degreesTraveled < degrees && Brain.timer(vex::timeUnits::msec) < END_TIME_MS){
        moveSide(vel);
        
        //  Calculate how much we have traveled (b - a)
        degreesTraveled = gyroscope.value(vex::rotationUnits::deg) - initGyroValue;
        
        //  The traveled distance can only be positive
        if (degreesTraveled < 0) degreesTraveled *= -1;
        
        //  Wait for the gyro to update
        vex::task::sleep(10);
    }
    
    //lockWheels();   //stop moving
    moveForward(0);
}   

void rotateDegrees10(int vel, double degrees, float reducePoint, float reducePerc, int END_TIME_MS){
    
    //  Get the initial value of the Gyro at the beginning of the func
    double initGyroValue = gyroscope.value(vex::rotationUnits::deg);
    double degreesTraveled = 0;
    double tempReducePerc = 0;
    bool isVelReduced = false;
    
    //  You can only travel a positive amount of degrees
    if (degrees < 0) degrees *= -1;
    
    // Clear the time
    Brain.resetTimer();
    
    //  While the traveled degrees are less than the end degrees, rotate
    while(degreesTraveled < degrees && Brain.timer(vex::timeUnits::msec) < END_TIME_MS){
        //  Rotate
        moveSide(vel);
                        
        //  Wait for the gyro to update
        vex::task::sleep(15);
        
        //  Calculate how much we have traveled (b - a)
        degreesTraveled = gyroscope.value(vex::rotationUnits::deg) - initGyroValue;
        
        //  The traveled distance can only be positive
        if (degreesTraveled < 0) degreesTraveled *= -1;
        
        /*
        if(degreesTraveled > degrees * reducePoint && !isVelReduced){
            isVelReduced = false;
            vel = minVel(vel * reducePerc);
        }*/
        
        //  Reduce velocity as we approach the end point
        tempReducePerc = 1.35 - (degreesTraveled / degrees);
        if (tempReducePerc > 1) tempReducePerc = 1;
        vel = minVel(vel * tempReducePerc);
    }
    
    moveForward(0);
}

void setGyro (int vel, double degreesEndpoint, int END_TIME_MS){
    //  Function to rotate the robot until the gyroscope reads the degreesEndpoint
    
    // calculate how much degrees we have to rotate in order to reach 
    // the endpoint, then use the rotate degrees fucntion to rotate that amount
    double degreesToRotate = 0;
    
    //  In order to calculate the degrees to rotate, we have to check which value is greater
    //  between the current value of the gyroscope and the endpoint
    if(gyroscope.value(vex::rotationUnits::deg) < degreesEndpoint) {
        
        //  If true, them b = endpoint and a = current gyroscope value
        //  (b - a)
        degreesToRotate = degreesEndpoint - gyroscope.value(vex::rotationUnits::deg);
        
        // We have to rotate to the left in order to reach the endpoint
        // Make sure the velocity is positive so that we can rotate to the left
        if(vel < 0) vel *= -1;
    }else{
        //  Otherwise, b = endpoint and a = current gyro value
        //  (b - a)
        degreesToRotate = gyroscope.value(vex::rotationUnits::deg) - degreesEndpoint;
        
        // We have to rotate to the right in order to reach the endpoint
        // Make sure the velocity is negative to we can rotate to the right
         if(vel > 0) vel *= -1;
    }
    
    //  Rotate and amount of degrees equal to the value o degreesToRotate
    //  at the velocity in the variable "vel"
    //  Reduce velocity once we have passed the 75% of the degrees to travel
    //  Reduce valocity by 40% after that point
    rotateDegrees10(vel, degreesToRotate, 0.8, 0.6, 1500);
} 


void ensureGyro (int vel, double degreespoint, double errorGap, int reduceVelAmount, int endTimeMS) {
    // This function will continue to run the set gyro function while the current 
    // error value is greater than the error gap

    //  Calculate the difference between the current gyro value and the endpoint
    double errorValue = abs(gyroscope.value(vex::rotationUnits::deg)) - abs(degreespoint);
    errorValue = abs(errorValue);
    int count = 1;
    
    rotationAid.set(false);
    vex::task::sleep(50);
    
    while (errorValue > errorGap){
        //  Set the gyro to the value of degreesPoint
        setGyro(vel, degreespoint, endTimeMS);
    
        vex::task::sleep(300);
                    
        errorValue = abs(gyroscope.value(vex::rotationUnits::deg)) - abs(degreespoint);
        errorValue = abs(errorValue);
        
        if (vel - reduceVelAmount > WHEEL_MIN_SPEED){
            vel = minVel(vel - reduceVelAmount);
        } else {
            vel = WHEEL_MIN_SPEED;
        }
        
        Brain.Screen.printAt(2, 120, "ensureGyro Running");
        Brain.Screen.printAt(2, 140, "Vel = %3d Count = %3d Error = %4.1f EGap = %4.1f", vel, count, errorValue, errorGap);
        
        count++;
    }
    
    Brain.Screen.printAt(2, 120, "ensureGyro is DONE!");
    rotationAid.set(true);
    vex::task::sleep(200);
}
/*
void setInTake(int vel, double endPoint){
    const int emergencyExitTimeMS = 3500;
    //double initValue = InTakePot.value(vex::rotationUnits::deg);
    double initValue = abs(motorClawL.rotation(vex::rotationUnits::deg));

    Brain.resetTimer();
    
    if (initValue < endPoint){
        //Go UP
        if (vel < 0) vel *= -1;
        while(abs(motorClawL.rotation(vex::rotationUnits::deg)) < endPoint &&
              Brain.timer(vex::timeUnits::msec) < emergencyExitTimeMS){
            motorClawL.spin(vex::directionType::fwd, vel, vex::velocityUnits::pct);
            motorClawR.spin(vex::directionType::rev, vel, vex::velocityUnits::pct);
            
            vex::task::sleep(15);
        }
        
    } else if (initValue > endPoint){
        //Go DOWN
        if (vel < 0) vel *= -1;
        while(abs(motorClawL.rotation(vex::rotationUnits::deg)) > endPoint &&
              Brain.timer(vex::timeUnits::msec) < emergencyExitTimeMS){
            motorClawL.spin(vex::directionType::fwd, -vel, vex::velocityUnits::pct);
            motorClawR.spin(vex::directionType::rev, -vel, vex::velocityUnits::pct);

            vex::task::sleep(15);
        }
    }
    if (endPoint == IN_TAKE_ENC_MIN_VALUE){
        motorClawL.stop(vex::brakeType::coast);
        motorClawR.stop(vex::brakeType::coast);
    } else {
        motorClawL.stop(vex::brakeType::hold);
        motorClawR.stop(vex::brakeType::hold);
    }

    //Brain.Screen.printAt(2, 140, "setInTake DONE!");
}
*/

// moves the robot's arm to a given position at a given velocity
void setLift(int vel, double endPoint){
    const int emergencyExitTimeMS = 3000;
    double initValue = LiftPot.value(vex::rotationUnits::deg);
    Brain.resetTimer();

    if (initValue < endPoint){
        //  Go UP
        while(LiftPot.value(vex::rotationUnits::deg) < endPoint  &&
              Brain.timer(vex::timeUnits::msec) < emergencyExitTimeMS ){
            
            motorLiftL.spin(vex::directionType::rev, vel, vex::velocityUnits::pct);
            motorLiftR.spin(vex::directionType::fwd, vel, vex::velocityUnits::pct);
            
            vex::task::sleep(15);
        }
    } else if (initValue > endPoint){
        //  Go DOWN
        while(LiftPot.value(vex::rotationUnits::deg) > endPoint  &&
              Brain.timer(vex::timeUnits::msec) < emergencyExitTimeMS ){
            
            motorLiftL.spin(vex::directionType::rev, -vel, vex::velocityUnits::pct);
            motorLiftR.spin(vex::directionType::fwd, -vel, vex::velocityUnits::pct);
            
            vex::task::sleep(15);
        }
    }
    
    if (endPoint == LIFT_POTEN_MIN_VALUE){
        motorLiftL.stop(vex::brakeType::coast);
        motorLiftR.stop(vex::brakeType::coast);
    } else {
        motorLiftL.stop(vex::brakeType::hold);
        motorLiftR.stop(vex::brakeType::hold);
    }
}

    
/*
            DRIVE UNTIL WALL USING SONAR
void driveUntilWall(int vel, double sonarEndValue){
    //  Go back until sonar reads 45 mm
    double sonarValue = Sonar.distance(vex::distanceUnits::mm);
    
    while(sonarValue > sonarEndValue || sonarValue == -1){
        moveForward(vel);
        
        sonarValue = Sonar.distance(vex::distanceUnits::mm);
    }
    
    moveForward(0); // Stop
}

void driveUntilWall(int vel, double sonarEndValue, double reductDistance_CM, double reducePerc){
    
    //  Go back until sonar reads 45 mm
    double sonarValue = Sonar.distance(vex::distanceUnits::mm);
    double reduceTickPoint = DEGREE_CIRCUMFERENCE_RATIO * reductDistance_CM;
    double encValue;
    bool isVelReduced = false;
    
    
    motorLD.resetRotation();
    motorRD.resetRotation();
    
    while(sonarValue > sonarEndValue || sonarValue == -1){
        moveForward(vel);
        
        if(!isVelReduced){
            encValue = ( abs(motorLD.rotation(vex::rotationUnits::deg)) + abs(motorRD.rotation(vex::rotationUnits::deg)))  / 2;
    
            if (encValue > reduceTickPoint){
                vel = vel * reducePerc;
                isVelReduced = true;
            }
        }
        
        sonarValue = Sonar.distance(vex::distanceUnits::mm);
    }
    
    moveForward(0); // Stop

}
*/

//////////////////////////////////////////////////
////            END OF CUSTOM API          //////


///////////////////////////////////////////////////
////                   TASKS                  /////
///////////////////////////////////////////////////

int printSensorValues(){
    // for testing, debugging, and autonomous purposes
	// print certain relevant values to the cortex's screen
    while(1){
        Brain.Screen.printAt(2, 20, "Left Enc = %-4.1f | Right Enc = %-4.1f", 
                             motorLD.rotation(vex::rotationUnits::deg), 
                             motorRD.rotation(vex::rotationUnits::deg));

        Brain.Screen.printAt(2, 40, "Gyro = %6.1f degrees", gyroscope.value(vex::rotationUnits::deg));

        
        //Brain.Screen.printAt(2, 60, "Left Sonar = %6.1f mm Right Sonar = %6.1f", 
        //                     LeftSonar.distance(vex::distanceUnits::mm),
        //                     RightSonar.distance(vex::distanceUnits::mm));
        
        Brain.Screen.printAt(2, 80, "Enc ClawL = %6.1f Enc ClawR = %6.1f", 
                             motorIntakeL.rotation(vex::rotationUnits::deg),
                             motorIntakeR.rotation(vex::rotationUnits::deg));

        Brain.Screen.printAt(2, 100, "LiftPot = %6.1f InTakePot = %6.1f", 
                             LiftPot.value(vex::rotationUnits::deg)
        //                     InTakePot.value(vex::rotationUnits::deg)
                            );
        
        Brain.Screen.printAt(2, 120, "Enc motorClaw = %6.1f", 
                             motorClawRotate.rotation(vex::rotationUnits::deg));
        
        vex::task::sleep(50);
    }
}

int slew_handler(){
    
    vex::motor *currentMotor;
	int currentMotorVel;
	int requestedMotorVel;
    int velocityToSet;
	int loopCount;
    
    //  Turn on all the wheel motors
    currentMotor = FIRST_MOTOR;
    for(loopCount = 0; loopCount < MOTOR_AMOUNT; loopCount++){
        //  Turn on the current motor
        currentMotor->spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
        
        //  Get the next motor to turn on
        currentMotor = getNextMotor(loopCount);
    }
    
    currentMotor = FIRST_MOTOR;

	while(true)
	{
		//	Set the first motor into currentMotor
		currentMotor = FIRST_MOTOR;
        
        //  Loop to go through every wheel motor
		for(loopCount = 0; loopCount < MOTOR_AMOUNT; loopCount++)
		{
			//	Get the velocity of the current motor
			currentMotorVel = currentMotor->velocity(vex::velocityUnits::pct);

			//	Get the requested velocity for the current motor that is in
			// 	the request array, the loop count is the index of the current motor
			requestedMotorVel = motorRequest[loopCount];

			//	After processing the current velocity, update the current motor
            velocityToSet = processMotorVel(currentMotorVel, requestedMotorVel);
			currentMotor->setVelocity( velocityToSet, vex::velocityUnits::pct);

			//	Get the next motor for the loop
			currentMotor = getNextMotor(loopCount);
		}

		vex::task::sleep(20); // Let the CPU take a break
	}
}

///////////////////////////////////////////////////
////                END OF TASKS              /////


///////////////////////////////////////////////////
////            COMPETITION FUNCTIONS         /////
///////////////////////////////////////////////////


//  CODE TO EXECUTE DURING THE AUTONOMOUS PERIOD  //


// initial test autonomous to run to ensure nothing is broken 
// before a competition match
void auto_test(){
    gyroscope.startCalibration();
    while(gyroscope.isCalibrating()){ }
    const int firstStickDegrees = 96;
    const int secondTurningDegrees = 40;
    const int secondStickDegrees = 185;
    const int poten_turning_value = 29; //error by 3
    
    vex::task::sleep(500);
    driveDistanceCM(50, 30, 4000);
    ensureGyro (60, -firstStickDegrees, 1.5, 35, 2000);
    moveForward(0);
    /*
    setLift(20, 40);
    vex::task::sleep(200);
    flipTurningPoint(40);
    ensureGyro (60, 0, 1.5, 35, 2000);
    moveForward(0);
    startInTake(60, 1);
    vex::task::sleep(500);
    stopInTake(true);
    */
}


// Actual autonomous code to be used during competition
void autonomus(){
    gyroscope.startCalibration();
    while(gyroscope.isCalibrating()){ }
    const int firstStickDegrees = 96;
    const int secondTurningDegrees = 40;
    const int secondStickDegrees = 185;
    const int poten_turning_value = 31; //error by 3
    
    vex::task::sleep(200);
   
    ////  Get ready  ////

    //////////////////////Go and grab the turning point
    setLift(15, poten_turning_value);
    startInTake(60);
    driveDistanceCM(90, 90, 4000); 
    //vex::task::sleep(100);
    //startInTake(60);
    //driveDistanceCM(100, 2, 4000);
    vex::task::sleep(200);//500
    stopInTake(true);
    vex::task::sleep(50);//100
    
    //////////////////////////////
    //push the ball
    driveDistanceCM(60, 35, 4000);
    vex::task::sleep(100);
   // driveDistanceCM(-70, 38, 4000); 
    /////////////////////////////
    
    //////////////////Go back to align to the first stick
    driveDistanceCM(-60, 27+30, 4000); //35 extra (testing 30 extra because it was backing up too far WORKED)
    vex::task::sleep(50);//100
    
    //////////////////////////Face the stick
    ensureGyro (60, -firstStickDegrees, 1.5, 35, 2000);
    moveForward(0);
    
    /////////////////////////Get to the Expansion Zone
    driveDistanceCM(50, 13.5, 4000); 
    
    ////////////////////////////////Go to the altitude
    setLift(30, LIFT_POTEN_MID_VALUE);
    
    ///////////////////////////////Get close to the 1st stick
    driveDistanceCM(20, 23, 4000); //(testing dropping from 25 cm to 23 WORKED)
    
    ////////Lower the intake & the lift to set the first turning point in the stick
    setLift(15, LIFT_POTEN_MID_VALUE - 10);
    vex::task::sleep(50);//100
    startInTake(30,1);
    
    //////////////////////////////////Go back a little bit and slowly
    driveDistanceCM(-20, 24, 4000);
    stopInTake(true);
    vex::task::sleep(50);//200
    setLift(50, LIFT_POTEN_MIN_VALUE);
    
    //////////////////////////////////Go and grab the second Turning Point
    ensureGyro (60, -secondTurningDegrees, 1.5, 35, 2000);
    moveForward(0);
    driveDistanceCM(60, 38, 4000);
    startInTake(60);
    driveDistanceCM(60, 5, 4000);
    vex::task::sleep(300);//500
    stopInTake();
    
    /////////////////////////////Flip Turning Point and throw it
    setLift(20, 40);
    vex::task::sleep(50);//200
    ensureGyro (60, 0, 1.5, 35, 2000);
    moveForward(0);
    flipTurningPoint(40);
    //driveDistanceCM(-20, 24, 4000);

    startInTake(60, 1);
    vex::task::sleep(500);
    stopInTake(true);
    
    ////////////////////////////Align to go parking
    flipTurningPoint(40);
    vex::task::sleep(50);//200
    setLift(50, LIFT_POTEN_MIN_VALUE);
    ensureGyro (60, -firstStickDegrees+7, 1.5, 35, 2000); //(testing changing from -1 to +7)
    moveForward(0);
    driveDistanceCM(-95, 130, 4000);//125
  
}


//  CODE TO EXECUTE DURING THE USER CONTROL PERIOD  //

void usercontrol(){
    gyroscope.startCalibration();
    while(gyroscope.isCalibrating()  ) { }
        
    //Create an infinite loop so that the program can pull remote control values every iteration.
    //This loop causes the program to run forever.

    //Variables to hold the value of the controller
    int vel_hor;
    int vel_ver;
    int ch2_value;
    bool liftBreakOn = false;
    const short CLAW_VEL = 50;
    
    
    while(1) {
        
        //Get the values of the axises of the controller
        vel_ver = Controller1.Axis4.value();
        vel_hor = Controller1.Axis3.value();
        
        //Set the left wheel motors to Vertical + Horizontal
        //Set the right wheel motors to Vertical - Horizontal
        leftMotors(vel_ver + vel_hor);
        rightMotors(vel_ver - vel_hor);
        
        ch2_value = Controller1.Axis2.value();
        
        if(abs(ch2_value) > 5){
            motorLiftL.spin(vex::directionType::rev, ch2_value, vex::velocityUnits::pct);
            motorLiftR.spin(vex::directionType::fwd, ch2_value, vex::velocityUnits::pct);
            liftBreakOn = false;
        } else if (!liftBreakOn){
            motorLiftL.stop(vex::brakeType::hold);
            motorLiftR.stop(vex::brakeType::hold);
           liftBreakOn = true;
        }
                
        if (Controller1.ButtonB.pressing()){
            rotationAid.set(false);
        } else if (Controller1.ButtonA.pressing()){
            rotationAid.set(true);
        }
        
       // if (Controller1.ButtonB.pressing()){
       //    setInTake(10, IN_TAKE_POTEN_MIN_VALUE);
       // } else if (Controller1.ButtonA.pressing()){
       //     setInTake(30, IN_TAKE_POTEN_MID_VALUE);
       // } else if (Controller1.ButtonX.pressing()){
       //     setInTake(50, 56);
       // }
        
        if(Controller1.ButtonL1.pressing()){
            motorClawRotate.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
        } else if (Controller1.ButtonL2.pressing()){
            motorClawRotate.spin(vex::directionType::fwd, -50, vex::velocityUnits::pct);
        } else {
            motorClawRotate.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
        }
        
       if(Controller1.ButtonR1.pressing()){
            motorIntakeR.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        } else if (Controller1.ButtonR2.pressing()){
            motorIntakeR.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
        } else {
            motorIntakeR.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
        }
        
        if(Controller1.ButtonR1.pressing()){
            motorIntakeL.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        } else if (Controller1.ButtonR2.pressing()){
            motorIntakeL.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
        } else {
            motorIntakeL.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
        }
        
		vex::task::sleep(15); //Sleep the task for a short amount of time to prevent wasted resources.
    }
}

///////////////////////////////////////////////////
////        END OF COMPETITION FUNCTIONS      /////





int main() {
    //  Create a competition instance
    vex::competition* comp = new vex::competition();
    
    //  Start the task for the slew handler
    vex::task* slew_handler_task = new vex::task(slew_handler, 2); 
    vex::task* printSensorValues_task = new vex::task(printSensorValues, 4);
    
    rotationAid.set(true);
    
    //  Set the callback functions
    comp->autonomous(autonomus);
    comp->drivercontrol(usercontrol);    
    //autonomus();
    //auto_test();
    //printSensorValues();
    
    // Stop the main func from exiting
    while (1) vex::task::sleep(15);
}