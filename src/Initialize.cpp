#include "main.h"
#include "reverblib.h"

int scrollIndex = 0;
std::string autonomi [NUM_OF_AUTONOMI] = {"Red Protected", "Red Unprotected", "Blue Protected", "Blue Unprotected", "One Cube", "Collect Cubes"};

/**
 * LCD DISPLAY ON BUTTON PRESS METHODS
 */
void onLeftButton() {
	if(autonomousType != -1){
		autonomousType = -1;
	}
	if(scrollIndex == 0){
		scrollIndex = NUM_OF_AUTONOMI-1;
	}
	else {
		scrollIndex--;
	}
}

void onCenterButton() {
	if(autonomousType == -1){
		autonomousType = scrollIndex;
	}
}

void onRightButton() {
	if(autonomousType != -1){
		autonomousType = -1;
	}
	if(scrollIndex == NUM_OF_AUTONOMI-1){
		scrollIndex = 0;
	}
	else {
		scrollIndex++;
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	tareAllMotors();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	lcd::initialize();
	lcd::set_text(1, autonomi[scrollIndex]);
	if(autonomousType != -1){
		lcd::set_text(2, "auton selected");
	}
	lcd::register_btn0_cb(onLeftButton);
	lcd::register_btn1_cb(onCenterButton);
	lcd::register_btn2_cb(onRightButton);
}