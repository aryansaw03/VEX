#ifndef reverblib
#define reverblib
#include "main.h"

static Controller master = (CONTROLLER_MASTER);
static Controller partner = (CONTROLLER_PARTNER);
static Motor chassisRightFront (RIGHT_FRONT_CHASSIS, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
static Motor chassisRightBack (RIGHT_BACK_CHASSIS, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
static Motor chassisLeftFront (LEFT_FRONT_CHASSIS, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
static Motor chassisLeftBack (LEFT_BACK_CHASSIS, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
static Motor lift (LIFT, MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES);
static Motor tray (TRAY, MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES);
static Motor intakeRight (INTAKE_RIGHT, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
static Motor intakeLeft (INTAKE_LEFT, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);

//Autonomi
static int autonomousType = -1;
static const int RED_PROTECTED = 0;
static const int RED_UNPROTECTED = 1;
static const int BLUE_PROTECTED = 2;
static const int BLUE_UNPROTECTED = 3;
static const int ONE_CUBE = 4;
static const int COLLECT_CUBES = 5;
static const int NUM_OF_AUTONOMI = 6;

//Chassis
static const double WHEEL_DIAMETER = 4.0;
static const double CHASSIS_SNAP_THRESHOLD = 25.0;

//Tray
static const double TRAY_FORWARD_POSITION = 1000.0;
static const double TRAY_BACK_POSITION = 0.0;
static const double MINIMUM_TRAY_VELOCITY = 30.0;
static const double TRAY_DOWN_VELOCITY_PERCENT = 0.6;

//Intake
static const double INTAKE_REVERSE_SLOW_VELOCITY_PERCENT = 0.6;
static const double INTAKE_REVERSE_FAST_VELOCITY_PERCENT = 1.0;
static const double SPROCKET_DIAMETER = 2.0;

//Lift
static const double LIFT_UP_POSITION = 770.0;
static const double LIFT_DOWN_POSITION = 0.0;

//General
static const double PI = 3.141593;

//Odometry
static double x = 1.5;
static double y = .5;
static double theta = 90.0;
static const double DISTANCE_BETWEEN_WHEELS = 9.0;

static double feetToDeg(double feet){
	double circ = (1.0/12.0)*WHEEL_DIAMETER*PI;
	return feet/circ*360;
}

static double degToFeet(double deg){
	double circ = (1.0/12.0)*WHEEL_DIAMETER*PI;
	return deg*circ/360;
}

static double rpmToIPM(double rpm, double diameter){
	return rpm*diameter*PI;
}

static double ipmToRPM(double ipm, double diameter){
	return (ipm)/(diameter*PI);
}

static std::int32_t getMaxVelocity(Motor m){
	motor_gearset_e_t gearset = m.get_gearing();
	switch (gearset){
		case MOTOR_GEARSET_36:
			return 100;
			break;
		case MOTOR_GEARSET_18:
			return 200;
			break;
		case MOTOR_GEARSET_06:
			return 600;
			break;
	}
}

static void tareAllMotors(){
	chassisRightFront.tare_position();
	chassisRightBack.tare_position();
	chassisLeftFront.tare_position();
	chassisLeftBack.tare_position();
	lift.tare_position();
	tray.tare_position();
	intakeRight.tare_position();
	intakeLeft.tare_position();
}

static void tareMotor(Motor m){
	m.tare_position();
}

static void brakeChassisRight(motor_brake_mode_e_t brakeType){
	chassisRightFront.set_brake_mode(brakeType);
	chassisRightBack.set_brake_mode(brakeType);
	chassisRightFront.move_velocity(0);
	chassisRightBack.move_velocity(0);
}

static void brakeChassisLeft(motor_brake_mode_e_t brakeType){
	chassisLeftFront.set_brake_mode(brakeType);
	chassisLeftBack.set_brake_mode(brakeType);
	chassisLeftFront.move_velocity(0);
	chassisLeftBack.move_velocity(0);
}

static void brakeChassis(motor_brake_mode_e_t brakeType){
	chassisRightFront.set_brake_mode(brakeType);
	chassisRightBack.set_brake_mode(brakeType);
	chassisLeftFront.set_brake_mode(brakeType);
	chassisLeftBack.set_brake_mode(brakeType);
	chassisRightFront.move_velocity(0);
	chassisRightBack.move_velocity(0);
	chassisLeftFront.move_velocity(0);
	chassisLeftBack.move_velocity(0);
}

static void moveChassisRightVoltage(std::int32_t voltage){
	chassisRightFront.move(voltage);
	chassisRightBack.move(voltage);
}

static void moveChassisLeftVoltage(std::int32_t voltage){
	chassisLeftFront.move(voltage);
	chassisLeftBack.move(voltage);
}

static void moveChassisVoltage(std::int32_t voltage){
	moveChassisRightVoltage(voltage);
	moveChassisLeftVoltage(voltage);
}

static void moveChassisRightVelocity(std::int16_t velocity){
	chassisRightFront.move_velocity(velocity);
	chassisRightBack.move_velocity(velocity);
}

static void moveChassisLeftVelocity(std::int16_t velocity){
	chassisLeftFront.move_velocity(velocity);
	chassisLeftBack.move_velocity(velocity);
}

static void moveChassisVelocity(std::int16_t velocity){
	moveChassisRightVelocity(velocity);
	moveChassisLeftVelocity(velocity);
}

static void turnChassisVelocity(std::int16_t velocity){
	moveChassisRightVelocity(velocity);
	moveChassisLeftVelocity(-velocity);
}

static void moveChassisRightVoltageTime(std::int32_t voltage, uint32_t time){
	chassisRightFront.move(voltage);
	chassisRightBack.move(voltage);
	delay(time);
	brakeChassis(MOTOR_BRAKE_BRAKE);
}

static void moveChassisLeftVoltageTime(std::int32_t voltage, uint32_t time){
	chassisLeftFront.move(voltage);
	chassisLeftBack.move(voltage);
	delay(time);
	brakeChassis(MOTOR_BRAKE_BRAKE);
}

static void moveChassisVoltageTime(std::int32_t voltage, uint32_t time){
	chassisRightFront.move(voltage);
	chassisRightBack.move(voltage);
	chassisLeftFront.move(voltage);
	chassisLeftBack.move(voltage);
	delay(time);
	brakeChassis(MOTOR_BRAKE_BRAKE);
}

static void moveChassisRightVelocityTime(std::int32_t velocity, uint32_t time){
	chassisRightFront.move_velocity(velocity);
	chassisRightBack.move_velocity(velocity);
	delay(time);
}

static void moveChassisLeftVelocityTime(std::int32_t velocity, uint32_t time){
	chassisLeftFront.move_velocity(velocity);
	chassisLeftBack.move_velocity(velocity);
	delay(time);
}

static void moveChassisVelocityTime(std::int32_t velocity, uint32_t time){
	chassisRightFront.move(velocity);
	chassisRightBack.move(velocity);
	chassisLeftFront.move(velocity);
	chassisLeftBack.move(velocity);
	delay(time);
	brakeChassis(MOTOR_BRAKE_BRAKE);
}

static void moveChassisRightRelative(double position, std::int32_t velocity){
	chassisRightFront.move_relative(position, velocity);
	chassisRightBack.move_relative(position, velocity);
}

static void moveChassisLeftRelative(double position, std::int32_t velocity){
	chassisLeftFront.move_relative(position, velocity);
	chassisLeftBack.move_relative(position, velocity);
}

static void moveChassisRelative(double position, std::int32_t velocity){
	moveChassisRightRelative(position, velocity);
	moveChassisLeftRelative(position, velocity);
}

static void moveChassisRightAbsolute(double position, std::int32_t velocity){
	chassisRightFront.move_relative(position, velocity);
	chassisRightBack.move_relative(position, velocity);
}

static void moveChassisLeftAbsolute(double position, std::int32_t velocity){
	chassisLeftFront.move_absolute(position, velocity);
	chassisLeftBack.move_absolute(position, velocity);
}

static void moveChassisAbsolute(double position, std::int32_t velocity){
	moveChassisRightAbsolute(position, velocity);
	moveChassisLeftAbsolute(position, velocity);
}

static void moveChassisForDistancePD(double feet, double pGain, double dGain){
	double deg = feetToDeg(feet);
	double prevRightError = deg-chassisRightFront.get_position();
	double prevLeftError = deg-chassisRightFront.get_position();
	double rightError = 0;
	double leftError = 0;
	while(true){
		rightError = deg-chassisRightFront.get_position();
		leftError = deg-chassisLeftFront.get_position();
		double rightSlope = (rightError-prevRightError)/20;
		double leftSlope = (leftError-prevLeftError)/20;
		double rawRightVel = pGain*rightError+dGain*rightSlope;
		double rawLeftVel = pGain*leftError+dGain*leftSlope;
		double rightVel = (rawRightVel > getMaxVelocity(chassisRightFront)) ? getMaxVelocity(chassisRightFront) : rawRightVel;
		double leftVel = (rawLeftVel > getMaxVelocity(chassisLeftFront)) ? getMaxVelocity(chassisLeftFront) : rawLeftVel;
		if(std::abs(rightError) < 0.2 || std::abs(leftError) < 0.2){
			brakeChassis(MOTOR_BRAKE_BRAKE);
			return;
		}
		prevRightError = rightError;
		prevLeftError = leftError;
		moveChassisRightVelocity(rightVel);
		moveChassisLeftVelocity(leftVel);
		delay(20);
	}
}

static void runIntake(){
	intakeLeft.move_velocity(getMaxVelocity(intakeLeft));
	intakeRight.move_velocity(getMaxVelocity(intakeRight));
}

static void runIntakeVelocity(std::int32_t velocity){
	intakeLeft.move_velocity(velocity);
	intakeRight.move_velocity(velocity);
}

static void intakeBrake(motor_brake_mode_e_t brakeType){
	intakeLeft.set_brake_mode(brakeType);
	intakeRight.set_brake_mode(brakeType);
	intakeLeft.move_velocity(0);
	intakeRight.move_velocity(0);
}

static void moveTrayVoltage(std::int32_t voltage){
	tray.move(voltage);
}

static void moveTrayVelocity(std::int16_t velocity){
	tray.move_velocity(velocity);
}

static void moveTrayAbsolute(double targetPosition, std::int32_t velocity){
	tray.move_absolute(targetPosition, velocity);
}

static void trayBrake(motor_brake_mode_e_t brakeType){
	tray.set_brake_mode(brakeType);
	tray.move_velocity(0);
}

static void moveTrayForDegreesPD(double targetDegrees, double maxVelocity, double pGain, double dGain){
	double prevTrayError = targetDegrees-tray.get_position();
	double prevPos = tray.get_position();
	double trayError = 0;
	while(true){
		trayError = targetDegrees-tray.get_position(); //Calculate Proportional
		double traySlope = (trayError-prevTrayError)/20; //Calculate Derivative
		double rawTrayVel = pGain*trayError+dGain*traySlope; //Calculate raw velocity
		double trayVel = (rawTrayVel > maxVelocity) ? maxVelocity : rawTrayVel; //Cap velocity
		if(std::abs(trayError) < 0.2){ //if error less than .2 exit
			trayBrake(MOTOR_BRAKE_HOLD);
			return;
		}
		prevTrayError = trayError;
		turnChassisVelocity(trayVel);
		delay(20);
	}
}

static void moveLiftVelocity(std::int16_t velocity){
	lift.move_velocity(velocity);
}

static void moveLiftAbsolute(double targetPosition, std::int32_t velocity){
	lift.move_absolute(targetPosition, velocity);
}

static void liftBrake(motor_brake_mode_e_t brakeType){
	lift.set_brake_mode(brakeType);
	lift.move_velocity(0);
}

static void trayFlipOut(){
	runIntakeVelocity(-getMaxVelocity(intakeRight));
	delay(1000);
	intakeBrake(MOTOR_BRAKE_HOLD);
}

//Odometry
static double calcHeading(double targetX, double targetY){
	double dx = targetX-x;
	double dy = targetY-y;
	return std::atan(dy/dx);
}

static void updateCoords(double prevRight, double prevLeft){
	double dRight = 69 - prevRight; //TODO: change to passive wheel input
	double dLeft = 69 - prevLeft; //TODO: change to passive wheel input
	double dRightFeet = degToFeet(dRight);
	double dLeftFeet = degToFeet(dLeft);
	double dCenterFeet = (dLeftFeet+dRightFeet)/2;
	x += dCenterFeet*std::cos(theta);
	y += dCenterFeet*std::sin(theta);
	theta += (dRightFeet-dLeftFeet)/DISTANCE_BETWEEN_WHEELS;
}

static void turnToHeadingPD(double heading, double pGain, double dGain, double maxVelocity){
	double prevHeadingError = heading-theta;
	double prevRight = 69; //TODO: change to passive wheel input
	double prevLeft = 69; //TODO: change to passive wheel input
	double headingError = 0;
	while(true){
		updateCoords(prevRight, prevLeft);
		headingError = heading-theta; //Calculate Proportional
		double headingSlope = (headingError-prevHeadingError)/20; //Calculate Derivative
		double rawTurnVel = pGain*headingError+dGain*headingSlope; //Calculate raw velocity
		double turnVel = (rawTurnVel > maxVelocity) ? maxVelocity : rawTurnVel; //Cap velocity
		if(std::abs(headingError) < 0.2){ //if error less than .2 exit
			brakeChassis(MOTOR_BRAKE_BRAKE);
			return;
		}
		prevHeadingError = headingError;
		prevRight = 69; //TODO: change to passive wheel input
		prevLeft = 69; //TODO: change to passive wheel input
		turnChassisVelocity(turnVel);
		delay(20);
	}
}

// static void moveToPositionPD(double targetX, double targetY, double pGain, double dGain){
// 	double prevRightError = deg-chassisRightFront.get_position();
// 	double prevLeftError = deg-chassisRightFront.get_position();
// 	double rightError = 0;
// 	double leftError = 0;
// 	while(true){
// 		double heading = calcHeading(targetX, targetY);
// 		turnToHeadingPD(heading, 1, 0.1, double maxVelocity)
// 		rightError = deg-chassisRightFront.get_position();
// 		leftError = deg-chassisLeftFront.get_position();
// 		double rightSlope = (rightError-prevRightError)/20;
// 		double leftSlope = (leftError-prevLeftError)/20;
// 		double rawRightVel = pGain*rightError+dGain*rightSlope;
// 		double rawLeftVel = pGain*leftError+dGain*leftSlope;
// 		double rightVel = (rawRightVel > getMaxVelocity(chassisRightFront)) ? getMaxVelocity(chassisRightFront) : rawRightVel;
// 		double leftVel = (rawLeftVel > getMaxVelocity(chassisLeftFront)) ? getMaxVelocity(chassisLeftFront) : rawLeftVel;
// 		if(std::abs(rightError) < 0.2 || std::abs(leftError) < 0.2){
// 			brakeChassis(MOTOR_BRAKE_BRAKE);
// 			return;
// 		}
// 		prevRightError = rightError;
// 		prevLeftError = leftError;
// 		moveChassisRightVelocity(rightVel);
// 		moveChassisLeftVelocity(leftVel);
// 		delay(20);
// 	}
// }

#endif
