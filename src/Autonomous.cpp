#include "main.h"
#include "reverblib.h"

void runInAllAuton(){
	trayFlipOut();
}

void runRedProtected(){
	runInAllAuton();
}

void runRedUnprotected(){
	runInAllAuton();
}

void runBlueProtected(){
	runInAllAuton();
}

void runOneCube(){
	moveChassisVoltageTime(127, 1000);
	moveChassisVoltageTime(-127, 1000);
	runInAllAuton();
}

void runCollectCubes(){
	runInAllAuton();
	runIntake();
	moveChassisForDistancePD(2, 1, 0.5);
	brakeChassis(MOTOR_BRAKE_HOLD);
}

void testAuton(){
	//turnToHeadingPD(degToRad(270),60, 5, 60);
	// delay(50);
	// turnToHeadingPD(degToRad(0),60, 5, 60);

	//  moveToPositionPD(24, 24, // targetX, targetY
	// 	190, 35, 100, // pGainTurn, dGainTurn, turnMaxVelocity
	// 	5, 0, 30, // pGainMove, dGainMove, moveMaxVelocity,
	// 	0.1, 0.05, 0.4); // pGainCorrection, dGainCorrection, maxCorrection

	swingMoveToPositionBackwardsPD(-20, -20, // targetX, targetY
		30, 5, 100, // pGainMove, dGainMove, moveMaxVelocity,
		1, 0, 1); // pGainCorrection, dGainCorrection, maxCorrection

	swingMoveToPositionBackwardsPD(-20, 20, // targetX, targetY
		30, 5, 100, // pGainMove, dGainMove, moveMaxVelocity,
		1, 0, 1); // pGainCorrection, dGainCorrection, maxCorrection
	swingMoveToPositionBackwardsPD(20, 20, // targetX, targetY
		30, 5, 100, // pGainMove, dGainMove, moveMaxVelocity,
		1, 0, 1); // pGainCorrection, dGainCorrection, maxCorrection

	swingMoveToPositionBackwardsPD(20, -20, // targetX, targetY
		30, 5, 100, // pGainMove, dGainMove, moveMaxVelocity,
		1, 0, 1); // pGainCorrection, dGainCorrection, maxCorrection
	//  moveToPositionPD(0, 0, // targetX, targetY
	// 	70, 5, 100, // pGainTurn, dGainTurn, turnMaxVelocity
	// 	30, 5, 100, // pGainMove, dGainMove, moveMaxVelocity,
	// 	1, 0, 1); // pGainCorrection, dGainCorrection, maxCorrection

	//turnToHeadingPD(degToRad(270),70, 5, 100);

	// 	130, 0, 30, // pGainMove, dGainMove, moveMaxVelocity,
	// 	2, 0, 1.0); // pGainCorrection, dGainCorrection, maxCorrection
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
	testAuton();
	return;
	//autonomousType = ONE_CUBE;
	switch (autonomousType) {
		case -1:
			runInAllAuton();
			break;
		case RED_PROTECTED:
			runRedProtected();
			break;
		case RED_UNPROTECTED:
			runRedUnprotected();
			break;
		case BLUE_PROTECTED:
			runBlueProtected();
			break;
		case ONE_CUBE:
			runOneCube();
			break;
		case COLLECT_CUBES:
			runCollectCubes();
			break;
	}
}
