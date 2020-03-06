#include "main.h"
#include "reverblib.h"

void allSide1(){
	moveChassisVoltageTime(127, 1000);
	moveChassisVoltageTime(-127, 1000);
}

void blueUnprot5(){
	trayFlipOut();
	delay(1000);
	//moveChassisVelocityTime(-200, 300);
	runIntake();
	moveChassisRelative(inchToDeg(33.5), 70, true);
	//delay(1000);
	moveChassisRelative(-inchToDeg(20.5), 70, true);
	//delay(1000);
	pointTurnRelative(-inchToDeg(11.8), 50, true);
	setUpCubeFor5Stack();
	tray.move_relative((int)(TRAY_FORWARD_POSITION*.5), 100);
	moveChassisVelocityTime(80, 700);
	delay(200);
	stack5();
}

void redUnprot5(){

}

void blueUnprot6(){
	trayFlipOut();
	delay(1000);
	runIntake();
	moveChassisRelative(inchToDeg(33.5), 70, true);
	pointTurnRelative(inchToDeg(1.5), 30, true);
	moveChassisRelative(inchToDeg(10), 50, true);
	moveChassisRelative(-inchToDeg(4), 50, true);
	pointTurnRelative(-inchToDeg(1.5), 50, true);
	moveChassisRelative(-inchToDeg(13.5), 100, true);
	pointTurnRelative(-inchToDeg(11.8), 50, true);
	setUpCubeFor5Stack();
	tray.move_relative((int)(TRAY_FORWARD_POSITION*.5), 100);
	moveChassisVelocityTime(80, 700);
	delay(200);
	stack();
}

void blueUnprot11() {
	trayFlipOut();
	delay(1000);
	//moveChassisVelocityTime(-200, 300);
	runIntake();
	moveChassisRelative(inchToDeg(33.5), 70, true);
	//delay(1000);
	moveChassisRelative(-inchToDeg(20.5), 70, true);
	//delay(1000);
	pointTurnRelative(-inchToDeg(11.8), 50, true);
	setUpCubeFor5Stack();
	tray.move_relative((int)(TRAY_FORWARD_POSITION*.5), 100);
	moveChassisVelocityTime(80, 700);
	delay(200);
	stack5();
}

void redUnprot9(){

}

void blueUnprot7(){
	trayFlipOut();
	delayWithOdom(400);

	runIntake();
	delay(50);

	int maxIntakeIPM = rpmToIPM(getMaxVelocity(intakeLeft), SPROCKET_DIAMETER);
	swingMoveToPositionPD(0, 37, // targetX, targetY
		20, 0, ipmToRPM(maxIntakeIPM, WHEEL_DIAMETER)-10, // pGainMove, dGainMove, moveMaxVelocity,
		0.2, 0, 0.1, // pGainCorrection, dGainCorrection, maxCorrection
		false); //exitOnPosSlope
	delayWithOdom(100);
	swingMoveToPositionBackwardsPD(26.3, 9, // targetX, targetY
		20, 0, 130, // pGainMove, dGainMove, moveMaxVelocity,
		0.3, 0, 0.7, // pGainCorrection, dGainCorrection, maxCorrection
		false); //exitOnPosSlope

	turnToHeadingPD(degToRad(90),50, 0, 90);

	delay(100);
	printf("%f\n", pos.x);
	swingMoveToPositionPD(pos.x, 25.5, // targetX, targetY
		20, 0, 80, // pGainMove, dGainMove, moveMaxVelocity,
		0.4, 0, 0.5, // pGainCorrection, dGainCorrection, maxCorrection
		true); //exitOnPosSlope
	double cornerX = 1;
	double cornerY = 18;
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

void blueProt5(){
	theta = PI/4;
	trayFlipOut();
	delayWithOdom(600);

	runIntakeVelocity(200);
	delay(50);

	swingMoveToPositionPD(17, 17, // targetX, targetY
		20, 0, 130, // pGainMove, dGainMove, moveMaxVelocity,
		0.2, 0, 0.1, // pGainCorrection, dGainCorrection, maxCorrection
		true); //exitOnPosSlope
	moveToPositionPD(-2, 13.2, // targetX, targetY
		60, 0, 150, //pGainTurn, dGainTurn, turnMaxVelocity
		20, 0, 130, // pGainMove, dGainMove, moveMaxVelocity,
		0.2, 0, 0.1, // pGainCorrection, dGainCorrection, maxCorrection
		true); //exitOnPosSlope
	runIntakeVelocity(120);
	moveToPositionPD(-14, 11, // targetX, targetY
		75, 0, 150, //pGainTurn, dGainTurn, turnMaxVelocity
		20, 0, 80, // pGainMove, dGainMove, moveMaxVelocity,
		0.3, 0, 0.2, // pGainCorrection, dGainCorrection, maxCorrection
		true); //exitOnPosSlope

	delayWithOdom(200);

	double cornerX = 16;
	double cornerY = 2;
	double heading = calcHeading(cornerX, cornerY);
	turnToHeadingPD(heading+degToRad(0),55, 0, 110);

	setUpCubeFor4Stack();
	tray.move_relative((int)(TRAY_FORWARD_POSITION*.7), 100);

	swingMoveToPositionPD(cornerX, cornerY, // targetX, targetY
		15, 0, 130, // pGainMove, dGainMove, moveMaxVelocity,
		0.3, 0, 0.2, // pGainCorrection, dGainCorrection, maxCorrection
		true); //exitOnPosSlope

	//delayWithOdom(200);
	turnToHeadingPD(degToRad(0),70, 0, 90);

	moveForTimeOdom(80, 600);

	moveIntakeRelative(-50, 200);

	stack5();
}

void redProt5(){

}

void testAuton(){

	swingMoveToPositionBackwardsPD(20, -20, // targetX, targetY
		20, 0, 130, // pGainMove, dGainMove, moveMaxVelocity,
		0.3, 0, 0.7, // pGainCorrection, dGainCorrection, maxCorrection
		false); //exitOnPosSlope

	turnToHeadingPD(degToRad(90),50, 0, 90);

	//delay(100);
	printf("%f\n", pos.x);
	moveForTimeOdom(20, 100);
	swingMoveToPositionPD(pos.x, 20, // targetX, targetY
		20, 0, 80, // pGainMove, dGainMove, moveMaxVelocity,
		0.4, 0, 0.5, // pGainCorrection, dGainCorrection, maxCorrection
		true); //exitOnPosSlope
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
	trayBrake(MOTOR_BRAKE_COAST);
	// testAuton();
	//return;

	// blueUnprot11();
	// return;
	autonomousType = BLUE_PROT_5;
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
		case RED_UNPROT_9:
			redUnprot9();
			break;
		case BLUE_UNPROT_6:
			blueUnprot6();
			break;
		case RED_UNPROT_7:
			redUnprot7();
			break;
		case BLUE_UNPROT_7:
			blueUnprot7();
			break;
		case RED_PROT_5:
			redProt5();
			break;
		case BLUE_PROT_5:
			blueProt5();
			break;

	}
}
