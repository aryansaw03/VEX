#include "main.h"
#include "reverblib.h"

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol(){
	master.clear();

	//=========//
	//Variables//
	//=========//
	int count = 0;
	int intakeToggleCycles = 0;
	int intakeReverseVelocityToggleCycles = 0;
	bool fastReverse = false;
	bool intakeForward = false;
	bool intakeBackward = false;
	bool movingTrayBackward = false;
	double prevTrayError = 0;

	while (true){
		//testing
		//printf("%d     %d\n", rightEncoder.get_value(), leftEncoder.get_value());
		updatePosition();
		printf("X: %f  Y: %f  Heading: %f \n", pos.x, pos.y, radToDeg(theta));
		//printf("%f     %f     %f\n", pos.x, pos.y, theta*180/PI);
		//printf("tray:%f\n", tray.get_position());
		//Display image on lcd
		lcd::clear();
		if(count%9 == 0){
			master.print(0, 0, "LF: %d  RF: %d", (int)chassisLeftFront.get_temperature(), (int)chassisRightFront.get_temperature());
		}
		else if(count%9 == 3){
			master.print(1, 0, "LB: %d  RB: %d", (int)chassisLeftFront.get_temperature(), (int)chassisRightFront.get_temperature());
		}
		else if(count%9 == 6){
			master.print(2, 0, "IL: %d  IR: %d", (int)intakeLeft.get_temperature(), (int)intakeRight.get_temperature());
		}
		//=================//
		//Controller Inputs//
		//=================//
		int leftVJoystick = master.get_analog(ANALOG_LEFT_Y);
		int leftHJoystick = master.get_analog(ANALOG_LEFT_X);
		int rightVJoystick = master.get_analog(ANALOG_RIGHT_Y);
		int rightHJoystick = master.get_analog(ANALOG_RIGHT_X);
		bool buttonUp = master.get_digital(DIGITAL_UP);
		bool buttonDown = master.get_digital(DIGITAL_DOWN);
		bool buttonLeft = master.get_digital(DIGITAL_LEFT);
		bool buttonRight = master.get_digital(DIGITAL_RIGHT);
		bool buttonA = master.get_digital(DIGITAL_A);
		bool buttonB = master.get_digital(DIGITAL_B);
		bool buttonY = master.get_digital(DIGITAL_Y);
		bool buttonX = master.get_digital(DIGITAL_X);
		bool buttonR1 = master.get_digital(DIGITAL_R1);
		bool buttonR2 = master.get_digital(DIGITAL_R2);
		bool buttonL1 = master.get_digital(DIGITAL_L1);
		bool buttonL2 = master.get_digital(DIGITAL_L2);
		//====End Controller Inputs====//


		//=======//
		//Chassis//
		//=======//
		bool sameSign = (rightVJoystick<0 && leftVJoystick<0) || (rightVJoystick>0 && leftVJoystick>0);
		int rightVel = rightVJoystick;
		int leftVel = leftVJoystick;
		if(sameSign && std::abs(rightVel - leftVel) < CHASSIS_SNAP_THRESHOLD){ //Snap chassis to same value if within threshold
			int avg = (int)((rightVel + leftVel)/2);
			rightVel=avg;
			leftVel=avg;
		}
		if(std::abs(rightVel)>5){
			moveChassisRightVoltage(rightVel);
		}
		else if(buttonL2 && tray.get_position() < TRAY_FORWARD_POSITION-100){
			int maxIntakeIPM = rpmToIPM(getMaxVelocity(intakeLeft), SPROCKET_DIAMETER);
			moveChassisVelocity(-ipmToRPM(maxIntakeIPM, WHEEL_DIAMETER));
		}
		else{
			brakeChassisRight(MOTOR_BRAKE_COAST);
		}
		if(std::abs(leftVel)>5){
			moveChassisLeftVoltage(leftVel);
		}
		else if(buttonL2 && tray.get_position() < TRAY_FORWARD_POSITION-100){
			int maxIntakeIPM = rpmToIPM(getMaxVelocity(intakeLeft), SPROCKET_DIAMETER);
			moveChassisVelocity(-ipmToRPM(maxIntakeIPM, WHEEL_DIAMETER));
		}
		else{
			brakeChassisLeft(MOTOR_BRAKE_COAST);
		}
		//====End Chassis====//


		//====//
		//Tray//
		//====//
		if(buttonL2 && !movingTrayBackward){
			movingTrayBackward = true;
		}

		if(buttonL1){
			double trayError = TRAY_FORWARD_POSITION - tray.get_position(); //Calculate Proportional
	        double rawTrayVel = 0.13 * trayError; //Calculate raw velocity
	        double trayVel = (rawTrayVel > getMaxVelocity(tray)) ? getMaxVelocity(tray) : rawTrayVel; //Cap velocity
			trayVel = (trayVel < 40) ? 40 : trayVel;
	        if (trayError < 1) { //if error less than .2 exit
	            trayBrake(MOTOR_BRAKE_HOLD);
	        }
			else{
	        	moveTrayVelocity(trayVel);
			}
			movingTrayBackward = false;
		}
		else if(movingTrayBackward){
			moveTrayAbsolute(TRAY_BACK_POSITION, getMaxVelocity(tray));
			if(tray.get_position() < TRAY_BACK_POSITION+2){
				movingTrayBackward = false;
			}
		}
		else{
			trayBrake(MOTOR_BRAKE_HOLD);
		}
		//====End Tray====//


		//======//
		//Intake//
		//======//
		if(buttonA && intakeReverseVelocityToggleCycles>20){ //Toggle roller outtake speeds between fast and slow
			fastReverse = !fastReverse;
			intakeReverseVelocityToggleCycles = 0;
		}

		if(buttonR1 && intakeToggleCycles>20) { //Toggle roller intake
			intakeBackward = false;
			intakeForward = !intakeForward;
			intakeToggleCycles = 0;
		}
		else if(buttonR2) { //Hold to run roller outtake
			intakeForward = false;
			intakeBackward = true;
		}
		else{
			intakeBackward = false;
		}

		if(intakeForward){
			runIntake();
		}
		else if(intakeBackward){
			if(fastReverse){
				runIntakeVelocity(-getMaxVelocity(intakeRight)*INTAKE_REVERSE_FAST_VELOCITY_PERCENT);
			}
			else{
				runIntakeVelocity(-getMaxVelocity(intakeRight)*INTAKE_REVERSE_SLOW_VELOCITY_PERCENT);
			}
		}
		else if(buttonL2  && tray.get_position() < TRAY_FORWARD_POSITION-100){
			runIntakeVelocity(-getMaxVelocity(intakeLeft));
		}
		else{
			intakeBrake(MOTOR_BRAKE_HOLD);
		}
		intakeToggleCycles++;
		intakeReverseVelocityToggleCycles++;
		//====End Intake====//


		//====//
		//Lift//
		//====//
		if(buttonUp){
			moveLiftAbsolute(LIFT_UP_POSITION, getMaxVelocity(lift));
		}
		else if(buttonDown){
			moveLiftAbsolute(LIFT_DOWN_POSITION, getMaxVelocity(lift));
		}
		else{
			liftBrake(MOTOR_BRAKE_HOLD);
		}
		//====End Lift====//

		count++;
		pros::delay(20);
	}
}
