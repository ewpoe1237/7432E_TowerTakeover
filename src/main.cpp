#include "main.h"
#define M_PI           3.14159265358979323846  /* pi */

pros::Imu inertial(7);

pros::Controller master(CONTROLLER_MASTER);
pros::Motor leftFront(9);
pros::Motor leftBack(10);
pros::Motor rightFront(2, 1);
pros::Motor rightBack(1, 1);
pros::Motor lift(7);
pros::Motor tilter(4);
pros::Motor intakeLeft(8);
pros::Motor intakeRight(6, 1);
bool liftStopped = true, intakeStopped = true, tilterStopped = true;
bool autonDone = false;
int setVelocity = 0, scaledPosition = 0, maxPosition = -5556;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
static bool pressed = false;
pressed = !pressed;
if (pressed) {
pros::lcd::set_text(2, "I was pressed!");
} else {
pros::lcd::clear_line(2);
}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
pros::lcd::initialize();
pros::lcd::set_text(1, "I.C.U.P");

pros::Imu inertial(5);
  inertial.reset();

tilter.tare_position();

pros::lcd::register_btn1_cb(on_center_button);
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
void competition_initialize() {}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void moveChassis(int x, int y) {
//move all sides of the chassis: left side corresponds to x power, right side corresponds to y power
  leftFront.move(x);
  leftBack.move(x);
  rightFront.move(y);
  rightBack.move(y);
}

void turnRight(int degrees) {
	while(inertial.get_rotation() < (degrees - 2) || inertial.get_rotation() > (degrees + 2)) {
		moveChassis(50, -50);
	}
  moveChassis(0, 0);
}

void turnLeft(int degrees) {
	while(inertial.get_rotation() < (degrees - 2) || inertial.get_rotation() > (degrees + 2)) {
		moveChassis(-50, 50);
	}
  moveChassis(0, 0);
}

void builtInPid(int left, int right, int speed) {
//use the built in pid to move the chassis.
leftBack.tare_position();
leftFront.tare_position();
rightBack.tare_position();
rightFront.tare_position();

leftFront.move_absolute(left, 60);
leftBack.move_absolute(left, 60);
rightFront.move_absolute(right, 60);
rightBack.move_absolute(right, 60);

while(!(leftFront.get_position() < (left + 8) && leftFront.get_position() > (left - 8)) ||
!(rightFront.get_position() < (right + 8) && rightFront.get_position() > (right - 8))) {
pros::delay(2);
}

leftBack.tare_position();
leftFront.tare_position();
rightBack.tare_position();
rightFront.tare_position();
moveChassis(0, 0);
}

void flipOut() {

//move the chassis forwards a little bit

intakeLeft.move(-127);
intakeRight.move(-127);

moveChassis(80, 80);
pros::delay(400);
moveChassis(0,0);

pros::delay(300);

moveChassis(-40, -40);
pros::delay(600);
moveChassis(0,0);

pros::delay(50);
lift.move(0);

pros::delay(250);

lift.move(-100);
pros::delay(200);
lift.move(0);

intakeLeft.move(0);
intakeRight.move(0);
//autonDone = true;
}
 //////////////////////////////WHEEL PID////////////////////////////////////
 float kpL = 0.65, kiL = 0.0000000001, kdL = 0.035;
  //kp will be largest, then kd, then ki
  //kp increases until error is small but robot still moves fast enough
  //ki should be REALLY small... too large = overshoot
  //kd should be increased until there is little to no overshoot

 float errorL = 0, readingL = 0;
 float integralL = 0, derivativeL = 0, previousL = 0;
 float speedL = 1;

 float kpR = 0.65, kiR = 0.000000001, kdR = 0.035;
  //kp will be largest, then kd, then ki
  //kp increases until error is small but robot still moves fast enough
  //ki should be REALLY small... too large = overshoot
  //kd should be increased until there is little to no overshoot

 float errorR = 0, readingR = 0;
 float integralR = 0, derivativeR = 0, previousR = 0;
 float speedR = 1;

 void pidChassis(int needL, int needR) {
     leftFront.tare_position();
    leftBack.tare_position();
    rightFront.tare_position();
    rightBack.tare_position();


     while(fabs(leftBack.get_position()) < fabs(needL) || fabs(rightBack.get_position()) < fabs(needR)) {
       readingL = leftBack.get_position();
       readingR = rightBack.get_position();

       errorL = needL - readingL;
       errorR = needR - readingR;

       //finding the integral or the running sum of all errors
       integralL += errorL;
       integralR += errorR;

       //if there is no error, there is no integral
       if(errorL == 0) {
         integralL = 0;
       }
       if(errorR == 0) {
         integralR = 0;
       }

       //the maximum value of integral will be 40
       if(fabs(errorL) >= 40) {
         integralL = 0;
       }
       if(fabs(errorR) >= 40) {
         integralR = 0;
       }

       //predicting the future value of the error... speeding it up
       //or slowing it down accordingly
       derivativeL = errorL - previousL;
       previousL = errorL;
       derivativeR = errorR - previousR;
       previousR = errorR;


       //setting the speed to
       //the error constant * error
       //plus the int constant * the int
       //plus the der constant * the der
       speedL = (kpL * errorL) + (kiL * integralL) + (kdL * derivativeL);
       speedR = (kpR * errorR) + (kiR * integralR) + (kdR * derivativeR);

       leftBack.move(speedL/4);
       leftFront.move(speedL/4);

       rightBack.move(speedR/4);
       rightFront.move(speedR/4);

       pros::delay(20);

       if(speedL > -30 && speedL < 30) break;
       if(speedR > -30 && speedR < 30) break;
     }

     leftFront.move(0);
     rightFront.move(0);
     leftBack.move(0);
     rightBack.move(0);

  //reset vals
   kpL = 0.65;
   kiL = 0.000000001;
   kdL = 0.025;
  errorL = 0;
  readingL = 0;
  integralL = 0;
  derivativeL = 0;
  previousL = 0;
  speedL = 1;
  kpR = 0.65;
  kiR = 0.000000001;
  kdR = 0.025;
  errorR = 0;
  readingR = 0;
  integralR = 0;
  derivativeR = 0;
  previousR = 0;
  speedR = 1;
 }

 //////////////////////////////SLOW PID////////////////////////////////////
 float skpL = 0.2875, skiL = 0.0000000001, skdL = 0.045;
  //kp will be largest, then kd, then ki
  //kp increases until error is small but robot still moves fast enough
  //ki should be REALLY small... too large = overshoot
  //kd should be increased until there is little to no overshoot

 float serrorL = 0, sreadingL = 0;
 float sintegralL = 0, sderivativeL = 0, spreviousL = 0;
 float sspeedL = 1;

 float skpR = 0.2875, skiR = 0.0000000001, skdR = 0.045;
  //kp will be largest, then kd, then ki
  //kp increases until error is small but robot still moves fast enough
  //ki should be REALLY small... too large = overshoot
  //kd should be increased until there is little to no overshoot

 float serrorR = 0, sreadingR = 0;
 float sintegralR = 0, sderivativeR = 0, spreviousR = 0;
 float sspeedR = 1;

 void slowPidChassis(int needL, int needR) {
     leftFront.tare_position();
    leftBack.tare_position();
    rightFront.tare_position();
    rightBack.tare_position();


     while(fabs(leftBack.get_position()) < fabs(needL) || fabs(rightBack.get_position()) < fabs(needR)) {
       sreadingL = leftBack.get_position();
       sreadingR = rightBack.get_position();

       serrorL = needL - sreadingL;
       serrorR = needR - sreadingR;

       //finding the integral or the running sum of all errors
       sintegralL += serrorL;
       sintegralR += serrorR;

       //if there is no error, there is no integral
       if(serrorL == 0) {
         sintegralL = 0;
       }
       if(serrorR == 0) {
         sintegralR = 0;
       }

       //the maximum value of integral will be 40
       if(fabs(serrorL) >= 40) {
         sintegralL = 0;
       }
       if(fabs(serrorR) >= 40) {
         sintegralR = 0;
       }

       //predicting the future value of the error... speeding it up
       //or slowing it down accordingly
       sderivativeL = serrorL - spreviousL;
       spreviousL = serrorL;
       sderivativeR = serrorR - spreviousR;
       spreviousR = serrorR;


       //setting the speed to
       //the error constant * error
       //plus the int constant * the int
       //plus the der constant * the der
       sspeedL = (skpL * serrorL) + (skiL * sintegralL) + (skdL * sderivativeL);
       sspeedR = (skpR * serrorR) + (skiR * sintegralR) + (skdR * sderivativeR);

       leftBack.move(sspeedL/8);
       leftFront.move(sspeedL/8);

       rightBack.move(sspeedR/8);
       rightFront.move(sspeedR/8);

       pros::delay(20);

       if(sspeedL > -100 && sspeedL < 100) break;
       if(sspeedR > -100 && sspeedR < 100) break;
     }

     leftFront.move(0);
     rightFront.move(0);
     leftBack.move(0);
     rightBack.move(0);

  //reset vals
   kpL = 0.65;
   kiL = 0.000000001;
   kdL = 0.025;
  errorL = 0;
  readingL = 0;
  integralL = 0;
  derivativeL = 0;
  previousL = 0;
  speedL = 1;
  kpR = 0.65;
  kiR = 0.000000001;
  kdR = 0.025;
  errorR = 0;
  readingR = 0;
  integralR = 0;
  derivativeR = 0;
  previousR = 0;
  speedR = 1;
 }

 void largeBlue() {

flipOut();

intakeLeft.move(127);
  intakeRight.move(127);

  lift.move(0);

  pros::delay(300);

  slowPidChassis(1700, 1700); //grab dem first few cubes

 builtInPid(600, -600, 50); //turn towards the next cube

  slowPidChassis(2000, 2000); //grab the third cube

builtInPid(250, -250, 30);


intakeLeft.move(-100);
intakeRight.move(-100);
pros::delay(250);
intakeLeft.move(0);
intakeRight.move(0);

moveChassis(80, 80);
pros::delay(400);
moveChassis(0, 0);

pros::delay(300);

  intakeLeft.move(-127);
  intakeRight.move(-127);
  pros::delay(150);
  intakeLeft.move(0);
  intakeRight.move(0);

  moveChassis(100, 100);
  pros::delay(275);
  moveChassis(0, 0);

  tilter.move(-100);
  pros::delay(2000);
  tilter.move(-40);
  pros::delay(160);
  tilter.move_absolute(tilter.get_position(), 200);

  moveChassis(40, 40);
  pros::delay(400);
  moveChassis(0, 0);

  pros::delay(300);

  intakeLeft.move(-127);
  intakeRight.move(-127);

  moveChassis(-50, -50);
  pros::delay(1000);
  moveChassis(0, 0);
  intakeLeft.move(0);
  intakeRight.move(0);


autonDone = true;
 }


  void largeRed() {
//right wheels start on the zig zag part of the tile
flipOut();

intakeLeft.move(127);
intakeRight.move(127);

pros::delay(300);

lift.move(0);

pros::delay(300);

slowPidChassis(1700, 1700); //grab dem first few cubes

intakeLeft = intakeRight = 0;

builtInPid(-560, 560, 50); //turn towards the next cube

intakeLeft.move(127);
intakeRight.move(127);

slowPidChassis(2000, 2000); //grab the third cube

builtInPid(-210, 210, 30);

intakeLeft.move(-100);
intakeRight.move(-100);
pros::delay(200);
intakeLeft.move(0);
intakeRight.move(0);

moveChassis(80, 80);
pros::delay(400);
moveChassis(0, 0);

pros::delay(300);

  intakeLeft.move(-127);
  intakeRight.move(-127);
  pros::delay(150);
  intakeLeft.move(0);
  intakeRight.move(0);

  moveChassis(100, 100);
  pros::delay(275);
  moveChassis(0, 0);

  tilter.move(-100);
  pros::delay(2000);
  tilter.move(-40);
  pros::delay(130);
  tilter.move_absolute(tilter.get_position(), 200);

  moveChassis(40, 40);
  pros::delay(400);
  moveChassis(0, 0);

  pros::delay(300);

  intakeLeft.move(-127);
  intakeRight.move(-127);

  moveChassis(-50, -50);
  pros::delay(1000);
  moveChassis(0, 0);
  intakeLeft.move(0);
  intakeRight.move(0);

  autonDone = true;
  }

void smallRed() {
std::string text("Red for small goal zoen");

flipOut();

intakeLeft.move(127);
intakeRight.move(127);

lift.move(-30);

pros::delay(300);

slowPidChassis(3000, 3000); //grab dem first few cubes

pros::delay(1000);

lift.move(0);

//pros::delay(400);

intakeLeft.move(0);
intakeRight.move(0);

intakeLeft.move(-127);
intakeRight.move(-127);
pros::delay(60);
intakeLeft.move(0);
intakeRight.move(0);

intakeLeft.move(0);
intakeRight.move(0);

pidChassis(-1900, -1900);

builtInPid(835, -835, 30);
// pidChassis(865, -865); //turn towards goal zone

moveChassis(0, 0);

moveChassis(85, 85);
pros::delay(400);
moveChassis(0, 0);

intakeLeft.move(-127);
intakeRight.move(-127);
pros::delay(100);
intakeLeft.move(0);
intakeRight.move(0);

tilter.move(-100);
pros::delay(2000);
tilter.move(-40);
pros::delay(200);
tilter.move_absolute(tilter.get_position(), 200);

moveChassis(40, 40);
pros::delay(400);
moveChassis(0, 0);

pros::delay(300);

intakeLeft.move(-127);
intakeRight.move(-127);

slowPidChassis(-1500, -1500);

intakeLeft.move(0);
intakeRight.move(0);

autonDone = true;
}


void smallBlue() {
std::string text("Blue for small goal zoen");

//let the tray flip out
flipOut();

intakeLeft.move(127);
intakeRight.move(127);

pros::delay(1000);

lift.move(-30);

pros::delay(300);

slowPidChassis(3000, 3000); //grab dem first few cubes

pros::delay(800);

lift.move(0);

//pros::delay(400);

intakeLeft.move(0);
intakeRight.move(0);

intakeLeft.move(-127);
intakeRight.move(-127);
pros::delay(60);
intakeLeft.move(0);
intakeRight.move(0);

intakeLeft.move(0);
intakeRight.move(0);

pidChassis(-1800, -1800);

// pidChassis(-960, 960);

builtInPid(-860, 860, 30);

intakeLeft.move(-127);
intakeRight.move(-127);
pros::delay(100);
intakeLeft.move(0);
intakeRight.move(0);

moveChassis(80, 80);
pros::delay(335);
moveChassis(0, 0);

intakeLeft.move(-20);
intakeRight.move(-20);
tilter.move(-100);
pros::delay(2000);
tilter.move(-40);
pros::delay(200);
tilter.move_absolute(tilter.get_position(), 200);
moveChassis(0, 0);

moveChassis(40, 40);
pros::delay(650);

intakeLeft.move(0);
intakeRight.move(0);


intakeLeft.move(-127);
intakeRight.move(-127);

moveChassis(-50, -50);
pros::delay(1000);
moveChassis(0, 0);
intakeLeft.move(0);
intakeRight.move(0);

autonDone = true;
}

void oneCube() {
flipOut();

intakeLeft.move(-127);
intakeRight.move(-127);
moveChassis(120, 120);

pros::delay(300);

moveChassis(-120, -120);
intakeLeft.move(0);
intakeRight.move(0);

pros::delay(500);
moveChassis(0, 0);

autonDone = true;
}

void skills() {
  flipOut();

  intakeLeft.move(127);
  intakeRight.move(127);

  lift.move(-30);

  pros::delay(300);

  slowPidChassis(3000, 3000); //grab dem first few cubes

  pros::delay(1000);

  lift.move(0);

  //pros::delay(400);

  intakeLeft.move(0);
  intakeRight.move(0);

  intakeLeft.move(-127);
  intakeRight.move(-127);
  pros::delay(60);
  intakeLeft.move(0);
  intakeRight.move(0);

  intakeLeft.move(0);
  intakeRight.move(0);

  pidChassis(-1900, -1900);

  builtInPid(835, -835, 30);
  // pidChassis(865, -865); //turn towards goal zone

  moveChassis(0, 0);

  moveChassis(85, 85);
  pros::delay(400);
  moveChassis(0, 0);

  intakeLeft.move(-127);
  intakeRight.move(-127);
  pros::delay(100);
  intakeLeft.move(0);
  intakeRight.move(0);

  tilter.move(-100);
  pros::delay(2000);
  tilter.move(-40);
  pros::delay(200);
  tilter.move_absolute(tilter.get_position(), 200);

  moveChassis(40, 40);
  pros::delay(400);
  moveChassis(0, 0);

  pros::delay(300);

  intakeLeft.move(-127);
  intakeRight.move(-127);

  slowPidChassis(-1000, -1000);

  intakeLeft.move(0);
  intakeRight.move(0);

  tilter.move(100);
  pros::delay(1500);
  tilter.move(0);

  builtInPid(900, -900, 15);

  moveChassis(-40, -40);
  pros::delay(3000);
  moveChassis(0, 0);

  slowPidChassis(2000, 2000);

  intakeLeft.move(127);
  intakeRight.move(127);

  slowPidChassis(2000, 2000);

  pros::delay(500);
  intakeLeft.move(0);
  intakeRight.move(0);

  pidChassis(-800, -800);

  intakeLeft.move(-80);
  intakeRight.move(-80);
  pros::delay(500);
  intakeLeft.move(0);
  intakeRight.move(0);

  lift.move_absolute(2300, 100);

  pros::delay(1500);

  pidChassis(800, 800);

  pros::delay(500);

  intakeLeft.move(-127);
  intakeRight.move(-127);
  pros::delay(500);
  intakeLeft.move(0);
  intakeRight.move(0);

  pros::delay(500);

  pidChassis(-1300, -1300);

  pros::delay(1000);

  builtInPid(350,-350, 15);

  pros::delay(1000);

  moveChassis(-40, -40);
  pros::delay(2000);
  moveChassis(0, 0);

  lift.move(-100);
  pros::delay(700);
  lift.move(0);

pros::delay(1000);

  slowPidChassis(2000, 2000);

  intakeLeft.move(127);
  intakeRight.move(127);

  slowPidChassis(2000, 2000);

  pros::delay(500);
  intakeLeft.move(0);
  intakeRight.move(0);

  pidChassis(-800, -800);

  intakeLeft.move(-80);
  intakeRight.move(-80);
  pros::delay(500);
  intakeLeft.move(0);
  intakeRight.move(0);

  lift.move_absolute(2300, 100);

  pros::delay(1500);

  pidChassis(800, 800);

  pros::delay(500);

  intakeLeft.move(-127);
  intakeRight.move(-127);
  pros::delay(500);
  intakeLeft.move(0);
  intakeRight.move(0);

  pros::delay(500);

  pidChassis(-1450, -1450);
  autonDone = true;
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
autonDone = false;
int gyrVal;

//frontBlue();
while(!autonDone) {
// smallRed();
 //smallBlue();
//largeBlue();
//turnRight(90);
//flipOut();
//autonDone = true;
//largeRed();

//largeBlue();

skills();

// oneCube();
// flipOut();
}


}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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
 void motion_profile(int x) {
  tilter.move_velocity(setVelocity * cos(M_PI/(2*x)));
  //setting the velocity of the tilter to the preset velocity times the cosine of pi/(2*the scaled position)
 }

void opcontrol() {
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor left_mtr(1);
pros::Motor right_mtr(2);
int gyrVal;

while (true) {
gyrVal = tilter.get_position();
pros::lcd::set_text(1, std::to_string(gyrVal));

int left = master.get_analog(ANALOG_LEFT_Y);
int right = master.get_analog(ANALOG_RIGHT_Y);

//still -127 to 127... left joystick corresponds to left side and right joystick corresponds to right side
leftFront = left;
leftBack = left;
rightFront = right;
rightBack = right;

scaledPosition = tilter.get_position()/maxPosition;

if (master.get_digital(DIGITAL_L1)) { //L1 and L2 control the lift
liftStopped = false;
lift.move(120);
} else if (master.get_digital(DIGITAL_L2)) {
liftStopped = false;
lift.move(-120);
} else {
liftStopped = true;
lift.move(0);
}

if (master.get_digital(DIGITAL_R1)) { //R1 and R2 control the intake
intakeStopped = false;
intakeLeft.move(120);
intakeRight.move(120);
} else if (master.get_digital(DIGITAL_R2)) {
intakeStopped = false;
intakeLeft.move(-120);
intakeRight.move(-120);
} else {
intakeStopped = true;
intakeLeft.move(0);
intakeRight.move(0);
}


if (master.get_digital(DIGITAL_X)) { //X brings the tray back in
tilterStopped = false;
//intakeStopped = false;
tilter.move(120);
} else if (master.get_digital(DIGITAL_Y) && tilter.get_position() > -3089) { //Y pushes the tray out at a fast speed
//intakeStopped = false;
tilterStopped = false;
tilter.move(-120);
}  else if (master.get_digital(DIGITAL_Y) && tilter.get_position() < -3089) { //B pushes the tray out at a slow speed
//intakeStopped = false;
tilterStopped = false;
tilter.move(-50);
}  else {
tilterStopped = true;
tilter.move(0);
}

if(tilterStopped) { //keeps the tilter from falling back down
tilter.move_absolute(tilter.get_position(), 200);
} else {
//tilter.move_velocity(-200 * cos(M_PI/(2*scaledPosition)));
}

if(intakeStopped) { //keeps the intake from letting go of the cubes
intakeLeft.move_absolute(intakeLeft.get_position(), 400);
intakeRight.move_absolute(intakeRight.get_position(), 400);
}

if(liftStopped) { //keeps the lift from falling back down
lift.move_absolute(lift.get_position(), 100);
}


pros::delay(20);
}
}
