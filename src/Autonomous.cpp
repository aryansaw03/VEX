#include "main.h"
#include "reverblib.h"

void allSide1(){
	moveChassisVoltageTime(127, 1000);
	moveChassisVoltageTime(-127, 1000);
}

void blueUnprot5(){
	//pos.y = 2.6;
	trayFlipOut();
	delayWithOdom(1000);

	runIntake();
	delay(50);
	int maxIntakeIPM = rpmToIPM(getMaxVelocity(intakeLeft), SPROCKET_DIAMETER);
	swingMoveToPositionPD(0, 36, // targetX, targetY
		20, 0, ipmToRPM(maxIntakeIPM, WHEEL_DIAMETER), // pGainMove, dGainMove, moveMaxVelocity,
		0.2, 0, 1, // pGainCorrection, dGainCorrection, maxCorrection
		false); //exitOnPosSlope

	swingMoveToPositionBackwardsPD(13, 10, // targetX, targetY
		20, 0, 130, // pGainMove, dGainMove, moveMaxVelocity,
		0.3, 0, 1, // pGainCorrection, dGainCorrection, maxCorrection
		false); //exitOnPosSlope

	moveToPositionPD(-1, 4.5, // targetX, targetY
		50, 0, 90, //pGainTurn, dGainTurn, turnMaxVelocity
		20, 0, 60, // pGainMove, dGainMove, moveMaxVelocity,
		0.2, 0, 1, // pGainCorrection, dGainCorrection, maxCorrection
		false); //exitOnPosSlope

	moveForTimeOdom(50, 200);

	delay(800);
	intakeBrake(MOTOR_BRAKE_HOLD);
	delay(300);
	setUpCubeFor5Stack();
	stack();
}

void redUnprot5(){
	pos.y = 2.6;
	trayFlipOut();
	delayWithOdom(1000);

	runIntake();
	delay(50);
	int maxIntakeIPM = rpmToIPM(getMaxVelocity(intakeLeft), SPROCKET_DIAMETER);
	swingMoveToPositionPD(0, 36, // targetX, targetY
		20, 0, ipmToRPM(maxIntakeIPM, WHEEL_DIAMETER), // pGainMove, dGainMove, moveMaxVelocity,
		0.2, 0, 1, // pGainCorrection, dGainCorrection, maxCorrection
		false); //exitOnPosSlope

	swingMoveToPositionBackwardsPD(-21, 8, // targetX, targetY
		20, 0, 130, // pGainMove, dGainMove, moveMaxVelocity,
		0.35, 0, 1, // pGainCorrection, dGainCorrection, maxCorrection
		false); //exitOnPosSlope

	moveToPositionPD(1, 3, // targetX, targetY
		50, 0, 90, //pGainTurn, dGainTurn, turnMaxVelocity
		20, 0, 100, // pGainMove, dGainMove, moveMaxVelocity,
		0.4, 0, 1, // pGainCorrection, dGainCorrection, maxCorrection
		false); //exitOnPosSlope

	turnToHeadingPD(degToRad(-10),50, 0, 90);

	moveForTimeOdom(50, 400);

	delay(500);
	intakeBrake(MOTOR_BRAKE_HOLD);
	delay(300);
	setUpCubeFor5Stack();
	stack();
}

void blueUnprot7(){
	trayFlipOut();
	delayWithOdom(600);

	runIntake();
	delay(50);

	int maxIntakeIPM = rpmToIPM(getMaxVelocity(intakeLeft), SPROCKET_DIAMETER);
	swingMoveToPositionPD(0, 37, // targetX, targetY
		20, 0, ipmToRPM(maxIntakeIPM, WHEEL_DIAMETER), // pGainMove, dGainMove, moveMaxVelocity,
		0.2, 0, 0.1, // pGainCorrection, dGainCorrection, maxCorrection
		false); //exitOnPosSlope
	delayWithOdom(100);
	swingMoveToPositionBackwardsPD(26.3, 9, // targetX, targetY
		20, 0, 130, // pGainMove, dGainMove, moveMaxVelocity,
		0.3, 0, 0.7, // pGainCorrection, dGainCorrection, maxCorrection
		false); //exitOnPosSlope

	turnToHeadingPD(degToRad(95),50, 0, 90);

	delay(100);
	printf("%f\n", pos.x);
	swingMoveToPositionPD(pos.x, 25.5, // targetX, targetY
		20, 0, 80, // pGainMove, dGainMove, moveMaxVelocity,
		0.4, 0, 0.5, // pGainCorrection, dGainCorrection, maxCorrection
		true); //exitOnPosSlope

	double cornerX = 0;
	double cornerY = 17;
	double heading = calcHeading(cornerX, cornerY);
	turnToHeadingPD(heading+degToRad(0),40, 0, 130);

	setUpCubeFor7Stack();
	tray.move_relative((int)(TRAY_FORWARD_POSITION*.7), 100);

	swingMoveToPositionPD(cornerX, cornerY, // targetX, targetY
		10, 0, 150, // pGainMove, dGainMove, moveMaxVelocity,
		0.2, 0, 0.16, // pGainCorrection, dGainCorrection, maxCorrection
		false); //exitOnPosSlope

	turnToHeadingPD(degToRad(235),45, 0, 100);

	moveForTimeOdom(70, 600);

	moveIntakeRelative(-50, 200);

	stack();
}

void redUnprot7(){

}

void blueUnprot6(){
	trayFlipOut();
	delay(700);
	runIntake();
	moveChassisRelative(inchToDeg(35), 80);
	delay(2000);
	pointTurnRelative(inchToDeg(1.5), 30);
	delay(500);
	moveChassisRelative(inchToDeg(10), 50);
	delay(1200);
	moveChassisRelative(-inchToDeg(4), 50);
	delay(500);
	pointTurnRelative(-inchToDeg(1.5), 30);
	delay(700);
	moveChassisRelative(-inchToDeg(15), 80);
	delay(1000);
	pointTurnRelative(-inchToDeg(11.5), 30);
	delay(2000);
	setUpCubeFor5Stack();
	delay(500);
	moveChassisVelocityTime(80, 700);
	delay(500);
	stack();
}

void redUnprot9(){

}

void blueProt5(){

}

void redProt5(){

}

void testAuton(){
	trayFlipOut();
	return;
	trayFlipOutMove();
	delayWithOdom(500);

	runIntake();
	delay(50);
	swingMoveToPositionPD(0, 40, // targetX, targetY
		20, 0, 100, // pGainMove, dGainMove, moveMaxVelocity,
		0.2, 0, 0.2, // pGainCorrection, dGainCorrection, maxCorrection
		false); //exitOnPosSlope
	printf("X: %f  Y: %f  Heading: %f \n", pos.x, pos.y, radToDeg(theta));
}

void exampleMethods(){
	swingMoveToPositionPD(-20, 30, // targetX, targetY
		20, 0, 100, // pGainMove, dGainMove, moveMaxVelocity,
		1, 0, 1, // pGainCorrection, dGainCorrection, maxCorrection
		false); //exitOnPosSlope

	swingMoveToPositionBackwardsPD(0, 0, // targetX, targetY
		20, 0, 100, // pGainMove, dGainMove, moveMaxVelocity,
		1, 0, 1, // pGainCorrection, dGainCorrection, maxCorrection
		false); //exitOnPosSlope

	moveToPositionPD(-10, 6, // targetX, targetY
		50, 0, 90, //pGainTurn, dGainTurn, turnMaxVelocity
		20, 0, 100, // pGainMove, dGainMove, moveMaxVelocity,
		1, 0, 1, // pGainCorrection, dGainCorrection, maxCorrection
		false); //exitOnPosSlope

	turnToHeadingPD(degToRad(90),50, 0, 90);

	setUpCubeFor5Stack();
	stack();

	trayFlipOut();
}

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

void autonomous() {
	// testAuton();
	// return;
	autonomousType = BLUE_UNPROT_7;
	switch (autonomousType) {
		case -1:
			trayFlipOut();
			break;
		case RED_UNPROT_5:
			redUnprot5();
			break;
		case BLUE_UNPROT_5:
			blueUnprot5();
			break;
		case RED_UNPROT_7:
			redUnprot7();
			break;
		case BLUE_UNPROT_7:
			blueUnprot7();
			break;
		case RED_UNPROT_9:
			redUnprot9();
			break;
		case BLUE_UNPROT_6:
			blueUnprot6();
			break;
		case RED_PROT_5:
			redProt5();
			break;
		case BLUE_PROT_5:
			blueProt5();
			break;

	}
}
