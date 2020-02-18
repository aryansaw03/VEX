#include "../include/main.h"
#include "../include/reverblib.h"

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
	turnToHeadingPD(degToRad(180),190, 35, getMaxVelocity(chassisLeftBack));
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
	autonomousType = ONE_CUBE;
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
