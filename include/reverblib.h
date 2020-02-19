#ifndef reverblib
#define reverblib
#include "main.h"
#include "vec2.h"

static Controller master = (CONTROLLER_MASTER);
static Controller partner = (CONTROLLER_PARTNER);
static Motor chassisRightFront(RIGHT_FRONT_CHASSIS, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
static Motor chassisRightBack(RIGHT_BACK_CHASSIS, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
static Motor chassisLeftFront(LEFT_FRONT_CHASSIS, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
static Motor chassisLeftBack(LEFT_BACK_CHASSIS, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
static Motor lift(LIFT, MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES);
static Motor tray(TRAY, MOTOR_GEARSET_36, true, MOTOR_ENCODER_DEGREES);
static Motor intakeRight(INTAKE_RIGHT, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
static Motor intakeLeft(INTAKE_LEFT, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);

static ADIEncoder leftEncoder('C', 'D', false);
static ADIEncoder rightEncoder('E', 'F', false);
static ADIEncoder horizontalEncoder('G', 'H', false);

//ALL DIMENTIONS IN INCHES!!!

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
static int DELAY_MS = 10;
static double DELAY_S = DELAY_MS/1000.0;

static vec2d pos(0, 0);
static double theta = PI / 2;
static double prevL = 0;
static double prevR = 0;
static double prevH = 0;
static const double DISTANCE_BETWEEN_TRACKING_WHEELS = 8.125;
static const double DISTANCE_TO_HORIZONTAL_WHEEL = 1.0 / 12.0; //TODO measure actual value
static const double SMALL_WHEEL_DIAMETER = 3.25;

static double degToRad(double deg){
	return deg*(PI/180);
}

static double radToDeg(double rad){
	return rad*(180/PI);
}

static double inchToDeg(double inch) {
    double circ = WHEEL_DIAMETER * PI;
    return inch / circ * 360;
}

static double degToInch(double deg) {
    double circ = WHEEL_DIAMETER * PI;
    return deg * circ / 360;
}

static double inchToDegSmall(double inch) {
    double circ = SMALL_WHEEL_DIAMETER * PI;
    return inch / circ * 360;
}

static double degToInchSmall(double deg) {
    double circ = SMALL_WHEEL_DIAMETER * PI;
    return deg * circ / 360;
}

static double rpmToIPM(double rpm, double diameter) {
    return rpm * diameter * PI;
}

static double ipmToRPM(double ipm, double diameter) {
    return (ipm) / (diameter * PI);
}

static std::int32_t getMaxVelocity(Motor m) {
    motor_gearset_e_t gearset = m.get_gearing();
    switch (gearset) {
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

static void tareAllMotors() {
    chassisRightFront.tare_position();
    chassisRightBack.tare_position();
    chassisLeftFront.tare_position();
    chassisLeftBack.tare_position();
    lift.tare_position();
    tray.tare_position();
    intakeRight.tare_position();
    intakeLeft.tare_position();
}

static void tareMotor(Motor m) {
    m.tare_position();
}

static void brakeChassisRight(motor_brake_mode_e_t brakeType) {
    chassisRightFront.set_brake_mode(brakeType);
    chassisRightBack.set_brake_mode(brakeType);
    chassisRightFront.move_velocity(0);
    chassisRightBack.move_velocity(0);
}

static void brakeChassisLeft(motor_brake_mode_e_t brakeType) {
    chassisLeftFront.set_brake_mode(brakeType);
    chassisLeftBack.set_brake_mode(brakeType);
    chassisLeftFront.move_velocity(0);
    chassisLeftBack.move_velocity(0);
}

static void brakeChassis(motor_brake_mode_e_t brakeType) {
    chassisRightFront.set_brake_mode(brakeType);
    chassisRightBack.set_brake_mode(brakeType);
    chassisLeftFront.set_brake_mode(brakeType);
    chassisLeftBack.set_brake_mode(brakeType);
    chassisRightFront.move_velocity(0);
    chassisRightBack.move_velocity(0);
    chassisLeftFront.move_velocity(0);
    chassisLeftBack.move_velocity(0);
}

static void moveChassisRightVoltage(std::int32_t voltage) {
    chassisRightFront.move(voltage);
    chassisRightBack.move(voltage);
}

static void moveChassisLeftVoltage(std::int32_t voltage) {
    chassisLeftFront.move(voltage);
    chassisLeftBack.move(voltage);
}

static void moveChassisVoltage(std::int32_t voltage) {
    moveChassisRightVoltage(voltage);
    moveChassisLeftVoltage(voltage);
}

static void moveChassisRightVelocity(std::int16_t velocity) {
    chassisRightFront.move_velocity(velocity);
    chassisRightBack.move_velocity(velocity);
}

static void moveChassisLeftVelocity(std::int16_t velocity) {
    chassisLeftFront.move_velocity(velocity);
    chassisLeftBack.move_velocity(velocity);
}

static void moveChassisVelocity(std::int16_t velocity) {
    moveChassisRightVelocity(velocity);
    moveChassisLeftVelocity(velocity);
}

static void turnChassisVelocity(std::int16_t velocity) {
    moveChassisRightVelocity(velocity);
    moveChassisLeftVelocity(-velocity);
}

static void moveChassisRightVoltageTime(std::int32_t voltage, uint32_t time) {
    chassisRightFront.move(voltage);
    chassisRightBack.move(voltage);
    delay(time);
    brakeChassis(MOTOR_BRAKE_BRAKE);
}

static void moveChassisLeftVoltageTime(std::int32_t voltage, uint32_t time) {
    chassisLeftFront.move(voltage);
    chassisLeftBack.move(voltage);
    delay(time);
    brakeChassis(MOTOR_BRAKE_BRAKE);
}

static void moveChassisVoltageTime(std::int32_t voltage, uint32_t time) {
    chassisRightFront.move(voltage);
    chassisRightBack.move(voltage);
    chassisLeftFront.move(voltage);
    chassisLeftBack.move(voltage);
    delay(time);
    brakeChassis(MOTOR_BRAKE_BRAKE);
}

static void moveChassisRightVelocityTime(std::int32_t velocity, uint32_t time) {
    chassisRightFront.move_velocity(velocity);
    chassisRightBack.move_velocity(velocity);
    delay(time);
}

static void moveChassisLeftVelocityTime(std::int32_t velocity, uint32_t time) {
    chassisLeftFront.move_velocity(velocity);
    chassisLeftBack.move_velocity(velocity);
    delay(time);
}

static void moveChassisVelocityTime(std::int32_t velocity, uint32_t time) {
    chassisRightFront.move(velocity);
    chassisRightBack.move(velocity);
    chassisLeftFront.move(velocity);
    chassisLeftBack.move(velocity);
    delay(time);
    brakeChassis(MOTOR_BRAKE_BRAKE);
}

static void moveChassisRightRelative(double position, std::int32_t velocity) {
    chassisRightFront.move_relative(position, velocity);
    chassisRightBack.move_relative(position, velocity);
}

static void moveChassisLeftRelative(double position, std::int32_t velocity) {
    chassisLeftFront.move_relative(position, velocity);
    chassisLeftBack.move_relative(position, velocity);
}

static void moveChassisRelative(double position, std::int32_t velocity) {
    moveChassisRightRelative(position, velocity);
    moveChassisLeftRelative(position, velocity);
}

static void moveChassisRightAbsolute(double position, std::int32_t velocity) {
    chassisRightFront.move_relative(position, velocity);
    chassisRightBack.move_relative(position, velocity);
}

static void moveChassisLeftAbsolute(double position, std::int32_t velocity) {
    chassisLeftFront.move_absolute(position, velocity);
    chassisLeftBack.move_absolute(position, velocity);
}

static void moveChassisAbsolute(double position, std::int32_t velocity) {
    moveChassisRightAbsolute(position, velocity);
    moveChassisLeftAbsolute(position, velocity);
}

static void moveChassisForDistancePD(double inch, double pGain, double dGain) {
    double deg = inchToDeg(inch);
    double prevRightError = deg - chassisRightFront.get_position();
    double prevLeftError = deg - chassisRightFront.get_position();
    double rightError = 0;
    double leftError = 0;
    while (true) {
        rightError = deg - chassisRightFront.get_position();
        leftError = deg - chassisLeftFront.get_position();
        double rightSlope = (rightError - prevRightError) / DELAY_S;
        double leftSlope = (leftError - prevLeftError) / DELAY_S;
        double rawRightVel = pGain * rightError + dGain * rightSlope;
        double rawLeftVel = pGain * leftError + dGain * leftSlope;
        double rightVel = (rawRightVel > getMaxVelocity(chassisRightFront)) ? getMaxVelocity(chassisRightFront) : rawRightVel;
        double leftVel = (rawLeftVel > getMaxVelocity(chassisLeftFront)) ? getMaxVelocity(chassisLeftFront) : rawLeftVel;
        if (std::abs(rightError) < 0.2 || std::abs(leftError) < 0.2) {
            brakeChassis(MOTOR_BRAKE_BRAKE);
            return;
        }
        prevRightError = rightError;
        prevLeftError = leftError;
        moveChassisRightVelocity(rightVel);
        moveChassisLeftVelocity(leftVel);
        delay(DELAY_MS);
    }
}

static void runIntake() {
    intakeLeft.move_velocity(getMaxVelocity(intakeLeft));
    intakeRight.move_velocity(getMaxVelocity(intakeRight));
}

static void runIntakeVelocity(std::int32_t velocity) {
    intakeLeft.move_velocity(velocity);
    intakeRight.move_velocity(velocity);
}

static void intakeBrake(motor_brake_mode_e_t brakeType) {
    intakeLeft.set_brake_mode(brakeType);
    intakeRight.set_brake_mode(brakeType);
    intakeLeft.move_velocity(0);
    intakeRight.move_velocity(0);
}

static void moveTrayVoltage(std::int32_t voltage) {
    tray.move(voltage);
}

static void moveTrayVelocity(std::int16_t velocity) {
    tray.move_velocity(velocity);
}

static void moveTrayAbsolute(double targetPosition, std::int32_t velocity) {
    tray.move_absolute(targetPosition, velocity);
}

static void trayBrake(motor_brake_mode_e_t brakeType) {
    tray.set_brake_mode(brakeType);
    tray.move_velocity(0);
}

static void moveTrayForDegreesPD(double targetDegrees, double maxVelocity, double pGain, double dGain) {
    double prevTrayError = targetDegrees - tray.get_position();
    double prevPos = tray.get_position();
    double trayError = 0;
    while (true) {
        trayError = targetDegrees - tray.get_position(); //Calculate Proportional
        double traySlope = (trayError - prevTrayError) / DELAY_S; //Calculate Derivative
        double rawTrayVel = pGain * trayError + dGain * traySlope; //Calculate raw velocity
        double trayVel = (rawTrayVel > maxVelocity) ? maxVelocity : rawTrayVel; //Cap velocity
        if (std::abs(trayError) < 0.2) { //if error less than .2 exit
            trayBrake(MOTOR_BRAKE_HOLD);
            return;
        }
        prevTrayError = trayError;
        moveTrayVelocity(trayVel);
        delay(DELAY_MS);
    }
}

static void moveTrayForDegreesPD_OC(double targetDegrees, double maxVelocity, double pGain, double dGain) {
    double prevTrayError = targetDegrees - tray.get_position();
    double prevPos = tray.get_position();
    double trayError = 0;
    trayError = targetDegrees - tray.get_position(); //Calculate Proportional
    double traySlope = (trayError - prevTrayError) / DELAY_S; //Calculate Derivative
    double rawTrayVel = pGain * trayError + dGain * traySlope; //Calculate raw velocity
    double trayVel = (rawTrayVel > maxVelocity) ? maxVelocity : rawTrayVel; //Cap velocity
    if (std::abs(trayError) < 0.2) { //if error less than .2 exit
        trayBrake(MOTOR_BRAKE_HOLD);
        return;
    }
    prevTrayError = trayError;
    moveTrayVelocity(trayVel);

}

static void moveLiftVelocity(std::int16_t velocity) {
    lift.move_velocity(velocity);
}

static void moveLiftAbsolute(double targetPosition, std::int32_t velocity) {
    lift.move_absolute(targetPosition, velocity);
}

static void liftBrake(motor_brake_mode_e_t brakeType) {
    lift.set_brake_mode(brakeType);
    lift.move_velocity(0);
}

static void trayFlipOut() {
    runIntakeVelocity(-getMaxVelocity(intakeRight));
    delay(1000);
    intakeBrake(MOTOR_BRAKE_HOLD);
}

//Odometry
static double calcHeading(double targetX, double targetY) {
    double dx = targetX - pos.x;
    double dy = targetY - pos.y;
    return std::atan(dy / dx);
}

static double smallestAngle(double target){
	double tempAngle = target - theta;
	return std::atan2(std::sin(tempAngle), std::cos(tempAngle));
}

static void updatePosition() {
    //find current encoder values
    double currentL = degToInchSmall(leftEncoder.get_value());
    double currentR = degToInchSmall(rightEncoder.get_value());
    double currentH = 0; //degToInchSmall(horizontalEncoder.get_value());
    //find the change in encoder values
    double changeL = prevL - currentL;
    double changeR = prevR - currentR;
    double changeH = prevL - currentH;

    //from pilons
    double changeTheta = (changeL - changeR) / DISTANCE_BETWEEN_TRACKING_WHEELS;

    //to avoid division by 0
    if (changeTheta == 0.0) changeTheta = 0.00000001;

    //from pilons
    double radiusOfTrackingCenterArc = changeR / changeTheta + DISTANCE_BETWEEN_TRACKING_WHEELS / 2;
    //double radiusOfTrackingCenterArc2 = changeL/changeTheta - DISTANCE_BETWEEN_TRACKING_WHEELS/2;
    //printf("%f    %f\n",radiusOfTrackingCenterArc,radiusOfTrackingCenterArc2);

    //from pilons
    double radiusOfHorizontalArc = 0; //changeH/changeTheta + DISTANCE_TO_HORIZONTAL_WHEEL;

    // find the local offset
    // chord forumla: 2sin(changeTheta/2) * radius
    vec2d localTranslation(radiusOfHorizontalArc, radiusOfTrackingCenterArc);
    localTranslation *= 2 * std::sin(changeTheta / 2);

    // convert from local translation to global translation
    double localThetaOffset = theta + changeTheta / 2;
    vec2d globalTranslation = localTranslation.rotateAndReturn(-localThetaOffset);

    // update position and theta
    pos += globalTranslation;
    theta += changeTheta;

    //update previous encoder values
    prevL = currentL;
    prevR = currentR;
    prevH = currentH;
}

static void turnToHeadingPD(double heading, double pGain, double dGain, double maxVelocity) {
    double prevHeadingError = heading - theta;
    double headingError = 0;
    while (true) {
        updatePosition();
        headingError = smallestAngle(heading); //Calculate Proportional
        double headingSlope = (headingError - prevHeadingError) / DELAY_S; //Calculate Derivative
        double rawTurnVel = pGain * headingError + dGain * headingSlope; //Calculate raw velocity
        double turnVel = (rawTurnVel > maxVelocity) ? maxVelocity : rawTurnVel; //Cap velocity
		printf("Heading: %f  Scaled Error: %f  Scaled Slope: %f  Turn Vel: %f\n", radToDeg(theta), pGain * headingError, dGain * headingSlope, turnVel);
        if (std::abs(headingError) < .005 && std::abs(headingSlope) < .0001) { //if error less than certain amount, exit
            brakeChassis(MOTOR_BRAKE_BRAKE);
            return;
        }
        prevHeadingError = headingError;
        turnChassisVelocity(turnVel);
        delay(DELAY_MS);
    }
}

static void moveToPositionPD(double targetX, double targetY, double pGainTurn, double dGainTurn, double turnMaxVelocity, double pGainMove, double dGainMove, double moveMaxVelocity, double pGainCorrection, double dGainCorrection, double maxCorrection){
	vec2d desiredPos(targetX,targetY);
	double heading = calcHeading(targetX, targetY);
	turnToHeadingPD(heading,pGainTurn,dGainTurn, turnMaxVelocity);

	double prevHeadingError = 0;
    double headingError = 0;

	double prevLocationError = 0;
	double locationError = 0;
	while(true){
		updatePosition();

		// correction is a value -1 < x < 1;
		// negative correction should correct by decreasing the speed of the left motors
		// positive correction should correct by decreasing the speed of the right motors
		headingError = smallestAngle(heading); //Calculate Proportional
        double headingSlope = (headingError - prevHeadingError) / DELAY_S; //Calculate Derivative
        double rawCorrection = pGainCorrection * headingError + dGainCorrection * headingSlope; //Calculate correction value
		// maxCorrection should be between 0 and 1
		double correction = (rawCorrection > maxCorrection) ? maxCorrection : rawCorrection; // cap the correction

		// for determining speed to move at
		locationError = desiredPos.dist(pos); // Proportional
		double locationSlope = (locationError - prevLocationError) / DELAY_S; // Derivative
		double rawMoveVelocity = pGainMove * locationError + dGainMove * locationSlope;
		double moveVelocity = (rawMoveVelocity > moveMaxVelocity) ? moveMaxVelocity : rawMoveVelocity; // cap move velocity

		double leftVelocity;
		double rightVelocity;
		if(correction < 0){ //negative correction should correct by decreasing the speed of the left motors
			leftVelocity = moveVelocity * (1.0 + correction); //descrease speed;
			rightVelocity = moveVelocity;
		}
		else{ //positive correction should correct by decreasing the speed of the right motors
			leftVelocity = moveVelocity;
			rightVelocity = moveVelocity * (1.0 - correction);
		}
		if (std::abs(locationError) < .005 && std::abs(locationSlope) < .0001) {
            brakeChassis(MOTOR_BRAKE_BRAKE);
            return;
        }

		// update previous values
		prevHeadingError = headingError;
		prevLocationError = locationError;

		moveChassisLeftVelocity(leftVelocity);
		moveChassisRightVelocity(rightVelocity);

		delay(DELAY_MS);
	}
}

// static void moveToPositionPD(double targetX, double targetY, double pGain, double dGain) {
//     double prevRightError = deg - chassisRightFront.get_position();
//     double prevLeftError = deg - chassisRightFront.get_position();
//     double rightError = 0;
//     double leftError = 0;
//     while (true) {
//         double heading = calcHeading(targetX, targetY);
//         turnToHeadingPD(heading, 1, 0.1, double maxVelocity)
//         rightError = deg - chassisRightFront.get_position();
//         leftError = deg - chassisLeftFront.get_position();
//         double rightSlope = (rightError - prevRightError) / 20;
//         double leftSlope = (leftError - prevLeftError) / 20;
//         double rawRightVel = pGain * rightError + dGain * rightSlope;
//         double rawLeftVel = pGain * leftError + dGain * leftSlope;
//         double rightVel = (rawRightVel > getMaxVelocity(chassisRightFront)) ? getMaxVelocity(chassisRightFront) : rawRightVel;
//         double leftVel = (rawLeftVel > getMaxVelocity(chassisLeftFront)) ? getMaxVelocity(chassisLeftFront) : rawLeftVel;
//         if (std::abs(rightError) < 0.2 || std::abs(leftError) < 0.2) {
//             brakeChassis(MOTOR_BRAKE_BRAKE);
//             return;
//         }
//         prevRightError = rightError;
//         prevLeftError = leftError;
//         moveChassisRightVelocity(rightVel);
//         moveChassisLeftVelocity(leftVel);
//         delay(20);
//     }
// }

# endif
