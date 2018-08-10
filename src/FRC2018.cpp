/* --- OPERATOR CONTROLS ---
 * Cube lift up: B
 * Cube lift down: A
 * Grabber in: LB
 * Grabber out: RB
 * Lift up: Y
 * Lift down: X
 * -------------------------*/

#include <CameraServer.h>
#include <WPILib.h>
#include <IterativeRobot.h>
#include "ctre/Phoenix.h"
#include <iostream>

//Xbox DPAD
const int D_UP = 0;
const int D_UP_RIGHT = 45;
const int D_RIGHT = 90;
const int D_DOWN_RIGHT = 135;
const int D_DOWN = 180;
const int D_DOWN_LEFT = 225;
const int D_LEFT = 270;
const int D_UP_LEFT = 315;

using namespace ctre::phoenix::motorcontrol;

class Robot: public IterativeRobot {
	WPI_TalonSRX* FL = new WPI_TalonSRX(0);
	WPI_TalonSRX* FR = new WPI_TalonSRX(2);
	WPI_TalonSRX* RL = new WPI_TalonSRX(6);
	WPI_TalonSRX* RR = new WPI_TalonSRX(7);

	DifferentialDrive* driveTrain = new DifferentialDrive(*FR, *FL);

	WPI_TalonSRX* cubeLift = new WPI_TalonSRX(5);
	WPI_TalonSRX* grabberL = new WPI_TalonSRX(3);
	WPI_TalonSRX* grabberR = new WPI_TalonSRX(4);

	WPI_TalonSRX* lift = new WPI_TalonSRX(1);

	Compressor* compressor = new Compressor();
	DoubleSolenoid* grabber = new DoubleSolenoid(0, 1);

	DigitalInput* lsCubeLiftTop = new DigitalInput(7);
	DigitalInput* lsCubeLiftBottom = new DigitalInput(6);
	DigitalInput* lsLiftTop = new DigitalInput(9);
	DigitalInput* lsLiftBottom = new DigitalInput(8);

	DigitalInput* autoSwitch0 = new DigitalInput(1);
	DigitalInput* autoSwitch1 = new DigitalInput(0);

	Joystick* stickMain = new Joystick(0);
	Joystick* stickRot = new Joystick(1);
	Joystick* stickXbox = new Joystick(2);

	bool xboxA, xboxB, xboxX, xboxY, xboxLB, xboxRB, xboxBack, xboxStart,
	xboxLS, xboxRS;
	bool grabbingCube;
	float driveForward, driveRot, xboxLeftY;
	std::string teamSides;
	int binToDec, autoCount, grabberCount;

public:
	void RobotInit() {

		cs::UsbCamera camera =	CameraServer::GetInstance()->StartAutomaticCapture();
		camera.SetResolution(640, 480);
		camera.SetFPS(30);

		RR->Follow(*FR);
		RL->Follow(*FL);

		compressor->Start();

		driveTrain->SetExpiration(1000);
	}

	void AutonomousInit() {
		teamSides = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		binToDec = (!autoSwitch1->Get() * 1) + (!autoSwitch0->Get() * 2);
		std::cout << "Auto buttons: " << binToDec << std::endl;
		autoCount = 0;
	}
	void AutonomousPeriodic() {
		if (binToDec == 0) {			//Nothing
			//Does nothing
		}

		else if (binToDec == 2) {		//Starting left
			if (teamSides[1] == 'L') {		//Scale
				scaleLeft();
			} else if (teamSides[0] == 'L') {	//Switch
				closeSwitchLeft();
			} else {
				lineLeft();
			}
		}

		else if (binToDec == 3) {		//Starting middle
			if (teamSides[0] == 'R') {		//Switch
				lineMiddleSpit();
			} else {
				lineMiddle();
			}
		}

		else if (binToDec == 1) {		//Starting right
			if (teamSides[1] == 'R') {		//Scale
				scaleRight();
			} else if (teamSides[0] == 'R') {	//Switch
				closeSwitchRight();
			} else {
				lineRight();
			}
		}

		Wait(0.01);
	}

	void TeleopInit() {
		grabbingCube = false;
	}
	void TeleopPeriodic() {
		//------------ XBOX -------------
		xboxA = stickXbox->GetRawButton(1);
		xboxB = stickXbox->GetRawButton(2);
		xboxX = stickXbox->GetRawButton(3);
		xboxY = stickXbox->GetRawButton(4);
		xboxLB = stickXbox->GetRawButton(5);
		xboxRB = stickXbox->GetRawButton(6);
		xboxBack = stickXbox->GetRawButton(7);
		xboxStart = stickXbox->GetRawButton(8);
		xboxLS = stickXbox->GetRawButton(9);
		xboxRS = stickXbox->GetRawButton(10);

		xboxLeftY = -1 * stickXbox->GetY();
		if(xboxLeftY > 0.15){
			xboxLeftY = (xboxLeftY - 0.15)/0.85;
		}
		else if(xboxLeftY < -0.15){
			xboxLeftY = (xboxLeftY + 0.15)/0.85;
		}
		else{
			xboxLeftY = 0;
		}
		//---------------------------------------


		//---------- CUBE LIFT ----------
		if(xboxLeftY > 0 && lsCubeLiftTop->Get()){
			cubeLift->Set(ControlMode::PercentOutput, xboxLeftY);
		}
		else if(xboxLeftY < 0 && lsCubeLiftBottom->Get()){
			cubeLift->Set(ControlMode::PercentOutput, xboxLeftY);
		}
		else{
			cubeLift->Set(ControlMode::PercentOutput, 0);
		}

		//-------- CUBE GRABBER ---------

		if(xboxRB){		//Out
			grabberL->Set(ControlMode::PercentOutput, 1);
			grabberR->Set(ControlMode::PercentOutput, -1);
		}
		else if(stickXbox->GetZ() > 0.3){	//Out slow
			grabberL->Set(ControlMode::PercentOutput, -0.3);
			grabberR->Set(ControlMode::PercentOutput, 0.3);
		}
		else if(xboxLB){		//In
			grabberL->Set(ControlMode::PercentOutput, -1);
			grabberR->Set(ControlMode::PercentOutput, 1);
		}
		else if(!grabbingCube) {		//Off
			grabberL->Set(ControlMode::PercentOutput, 0);
			grabberR->Set(ControlMode::PercentOutput, 0);
		}

		if (xboxB) {				//Open
			grabber->Set(DoubleSolenoid::kForward);
		}
		else if (xboxX) {	//Close
			grabber->Set(DoubleSolenoid::kReverse);
		}

		if(xboxA){
			grabber->Set(DoubleSolenoid::kForward);	//Open
			grabberL->Set(ControlMode::PercentOutput, 1);	//In
			grabberR->Set(ControlMode::PercentOutput, -1);	//In
			grabbingCube = true;
			grabberCount = 0;
		}
		else{
			if(grabbingCube){
				if(grabberCount <= 15){
					grabber->Set(DoubleSolenoid::kReverse);	//Close
					grabberL->Set(ControlMode::PercentOutput, 1);	//In
					grabberR->Set(ControlMode::PercentOutput, -1);	//In
					grabberCount++;
				}
				else{
					grabberL->Set(ControlMode::PercentOutput, 0);	//Off
					grabberR->Set(ControlMode::PercentOutput, 0);	//Off
					grabbingCube = false;
				}
			}
		}
		//-------------------------------

		//------------- LIFT ------------
		if (xboxStart && lsLiftTop->Get()) {			//Up
			lift->Set(ControlMode::PercentOutput, 1);
		} else if (xboxBack && lsLiftBottom->Get()) {		//Down
			lift->Set(ControlMode::PercentOutput, -1);
		} else {				//Off
			lift->Set(ControlMode::PercentOutput, 0);
		}
		//-------------------------------

		//------------- DRIVING -----------------
		driveForward = stickMain->GetY();
		driveRot = stickRot->GetX();

		driveTrain->ArcadeDrive(driveForward, 0.7 * driveRot, true);
		//---------------------------------------
		Wait(0.01);
	}

	void TestPeriodic() {
	}

private:
	//---- AUTONOMOUS METHODS ----
	void closeSwitchLeft() {
		//Put Power Cube in the left side of the closest switch starting from left side of the field

		if (autoCount == 0) {
			cubeLift->Set(ControlMode::PercentOutput, 0.6);		//Cube lift up
		}
		if (autoCount == 20) {
			cubeLift->Set(ControlMode::PercentOutput, -0.6);	//Cube lift down
		}
		if (autoCount == 40) {
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
			grabberL->Set(ControlMode::PercentOutput, 0.3);		//Grab cube
			grabberR->Set(ControlMode::PercentOutput, -0.3);	//Grab cube
		}

		if (autoCount == 60) {
			grabberL->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			grabberR->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			cubeLift->Set(ControlMode::PercentOutput, 0.7);		//Cube lift up
		}
		if(autoCount == 165){
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
		}
		if(autoCount == 245){
			grabberL->Set(ControlMode::PercentOutput, -0.7);	//Spit cube
			grabberR->Set(ControlMode::PercentOutput, 0.7);		//Spit cube
		}
		if(autoCount == 285){
			grabberL->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			grabberR->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
		}


		if(autoCount == 60){
			driveTrain->ArcadeDrive(-0.8, 0.0);					//Drive forward
		}
		if(autoCount == 165){
			driveTrain->ArcadeDrive(-0.5, 0.0);					//Slow down
		}
		if(autoCount == 195){
			driveTrain->ArcadeDrive(-0.3, 0.0);					//Slow down
		}
		if(autoCount == 205){
			driveTrain->ArcadeDrive(0.0, 0.0);					//Stop
		}
		if(autoCount == 215){
			driveTrain->ArcadeDrive(0.0, 0.7);					//Rotate clockwise
		}
		if(autoCount == 240){
			driveTrain->ArcadeDrive(0.0, 0.0);					//Stop
		}

		autoCount++;
	}
	void scaleLeft() {
		//Put Power Cube in the left side of the scale starting from the left side of the field

		if (autoCount == 0) {
			cubeLift->Set(ControlMode::PercentOutput, 0.6);		//Cube lift up
		}
		if (autoCount == 20) {
			cubeLift->Set(ControlMode::PercentOutput, -0.6);	//Cube lift down
		}
		if (autoCount == 40) {
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
			grabberL->Set(ControlMode::PercentOutput, 0.3);		//Grab cube
			grabberR->Set(ControlMode::PercentOutput, -0.3);	//Grab cube
		}

		if (autoCount == 60) {
			grabberL->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			grabberR->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			cubeLift->Set(ControlMode::PercentOutput, 0.7);		//Cube lift up
		}

		if(!lsCubeLiftTop->Get()){
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
		}

		if(autoCount == 60){
			driveTrain->ArcadeDrive(-0.8, 0.0);					//Drive forward
		}
		if(autoCount == 270){
			driveTrain->ArcadeDrive(-0.5, 0.0);					//Slow down
		}
		if(autoCount == 300){
			driveTrain->ArcadeDrive(-0.3, 0.0);					//Slow down
		}
		if(autoCount == 310){
			driveTrain->ArcadeDrive(0.0, 0.0);					//Stop
		}
		if(autoCount == 320){
			driveTrain->ArcadeDrive(0.0, 0.7);					//Rotate clockwise
		}
		if(autoCount == 345){
			driveTrain->ArcadeDrive(0.0, 0.0);					//Stop
		}

		if(autoCount == 350){
			grabberL->Set(ControlMode::PercentOutput, -0.7);	//Spit cube
			grabberR->Set(ControlMode::PercentOutput, 0.7);		//Spit cube
		}
		if(autoCount == 390){
			grabberL->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			grabberR->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
		}

		autoCount++;
	}
	void lineLeft() {
		//Cross the autonomous line starting from the left side of the field

		if (autoCount == 0) {
			cubeLift->Set(ControlMode::PercentOutput, 0.6);		//Cube lift up
		}
		if (autoCount == 20) {
			cubeLift->Set(ControlMode::PercentOutput, -0.6);	//Cube lift down
		}
		if (autoCount == 40) {
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
			grabberL->Set(ControlMode::PercentOutput, 0.3);		//Grab cube
			grabberR->Set(ControlMode::PercentOutput, -0.3);	//Grab cube
		}

		if (autoCount == 60) {
			grabberL->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			grabberR->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			cubeLift->Set(ControlMode::PercentOutput, 0.7);		//Cube lift up
		}
		if(autoCount == 145){
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
		}
		if(autoCount == 200){
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
		}
		if (autoCount == 300) {
			driveTrain->ArcadeDrive(0.0, 0.0);					//Drive stop
		}


		if(autoCount == 60){
			driveTrain->ArcadeDrive(-0.8, 0.0);					//Drive forward
		}
		if(autoCount == 135){
			driveTrain->ArcadeDrive(-0.5, 0.0);					//Slow down
		}
		if(autoCount == 160){
			driveTrain->ArcadeDrive(-0.3, 0.0);					//Slow down
		}
		if(autoCount == 170){
			driveTrain->ArcadeDrive(0.0, 0.0);					//Stop
		}

		autoCount++;
	}

	void closeSwitchRight() {
		//Put Power Cube in the right side of the closest switch starting from right side of the field

		if (autoCount == 0) {
			cubeLift->Set(ControlMode::PercentOutput, 0.6);		//Cube lift up
		}
		if (autoCount == 20) {
			cubeLift->Set(ControlMode::PercentOutput, -0.6);	//Cube lift down
		}
		if (autoCount == 40) {
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
			grabberL->Set(ControlMode::PercentOutput, 0.3);		//Grab cube
			grabberR->Set(ControlMode::PercentOutput, -0.3);	//Grab cube
		}

		if (autoCount == 60) {
			grabberL->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			grabberR->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			cubeLift->Set(ControlMode::PercentOutput, 0.7);		//Cube lift up
		}
		if(autoCount == 165){
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
		}
		if(autoCount == 245){
			grabberL->Set(ControlMode::PercentOutput, -0.7);	//Spit cube
			grabberR->Set(ControlMode::PercentOutput, 0.7);		//Spit cube
		}
		if(autoCount == 285){
			grabberL->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			grabberR->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
		}


		if(autoCount == 60){
			driveTrain->ArcadeDrive(-0.8, 0.0);					//Drive forward
		}
		if(autoCount == 165){
			driveTrain->ArcadeDrive(-0.5, 0.0);					//Slow down
		}
		if(autoCount == 195){
			driveTrain->ArcadeDrive(-0.3, 0.0);					//Slow down
		}
		if(autoCount == 205){
			driveTrain->ArcadeDrive(0.0, 0.0);					//Stop
		}
		if(autoCount == 215){
			driveTrain->ArcadeDrive(0.0, -0.7);					//Rotate counter clockwise
		}
		if(autoCount == 240){
			driveTrain->ArcadeDrive(0.0, 0.0);					//Stop
		}

		autoCount++;
	}
	void scaleRight() {
		//Put Power Cube in the right side of the scale starting from the right side of the field

		if (autoCount == 0) {
			cubeLift->Set(ControlMode::PercentOutput, 0.6);		//Cube lift up
		}
		if (autoCount == 20) {
			cubeLift->Set(ControlMode::PercentOutput, -0.6);	//Cube lift down
		}
		if (autoCount == 40) {
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
			grabberL->Set(ControlMode::PercentOutput, 0.3);		//Grab cube
			grabberR->Set(ControlMode::PercentOutput, -0.3);	//Grab cube
		}

		if (autoCount == 60) {
			grabberL->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			grabberR->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			cubeLift->Set(ControlMode::PercentOutput, 0.7);		//Cube lift up
		}

		if(!lsCubeLiftTop->Get()){
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
		}

		if(autoCount == 60){
			driveTrain->ArcadeDrive(-0.8, 0.0);					//Drive forward
		}
		if(autoCount == 270){
			driveTrain->ArcadeDrive(-0.5, 0.0);					//Slow down
		}
		if(autoCount == 300){
			driveTrain->ArcadeDrive(-0.3, 0.0);					//Slow down
		}
		if(autoCount == 310){
			driveTrain->ArcadeDrive(0.0, 0.0);					//Stop
		}
		if(autoCount == 320){
			driveTrain->ArcadeDrive(0.0, -0.7);					//Rotate counter clockwise
		}
		if(autoCount == 345){
			driveTrain->ArcadeDrive(0.0, 0.0);					//Stop
		}

		if(autoCount == 350){
			grabberL->Set(ControlMode::PercentOutput, -0.7);	//Spit cube
			grabberR->Set(ControlMode::PercentOutput, 0.7);		//Spit cube
		}
		if(autoCount == 390){
			grabberL->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			grabberR->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
		}

		autoCount++;
	}
	void lineRight() {
		//Cross the autonomous line starting from the right side of the field

		if (autoCount == 0) {
			cubeLift->Set(ControlMode::PercentOutput, 0.6);		//Cube lift up
		}
		if (autoCount == 20) {
			cubeLift->Set(ControlMode::PercentOutput, -0.6);	//Cube lift down
		}
		if (autoCount == 40) {
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
			grabberL->Set(ControlMode::PercentOutput, 0.3);		//Grab cube
			grabberR->Set(ControlMode::PercentOutput, -0.3);	//Grab cube
		}

		if (autoCount == 60) {
			grabberL->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			grabberR->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			cubeLift->Set(ControlMode::PercentOutput, 0.7);		//Cube lift up
		}
		if(autoCount == 145){
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
		}
		if(autoCount == 200){
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
		}
		if (autoCount == 300) {
			driveTrain->ArcadeDrive(0.0, 0.0);					//Drive stop
		}


		if(autoCount == 60){
			driveTrain->ArcadeDrive(-0.8, 0.0);					//Drive forward
		}
		if(autoCount == 135){
			driveTrain->ArcadeDrive(-0.5, 0.0);					//Slow down
		}
		if(autoCount == 160){
			driveTrain->ArcadeDrive(-0.3, 0.0);					//Slow down
		}
		if(autoCount == 170){
			driveTrain->ArcadeDrive(0.0, 0.0);					//Stop
		}

		autoCount++;
	}

	void lineMiddle() {
		//Cross the autonomous line starting from the middle of the field

		if (autoCount == 0) {
			cubeLift->Set(ControlMode::PercentOutput, 0.6);		//Cube lift up
		}
		if (autoCount == 20) {
			cubeLift->Set(ControlMode::PercentOutput, -0.6);	//Cube lift down
		}
		if (autoCount == 40) {
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
			grabberL->Set(ControlMode::PercentOutput, 0.3);		//Grab cube
			grabberR->Set(ControlMode::PercentOutput, -0.3);	//Grab cube
		}

		if (autoCount == 60) {
			grabberL->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			grabberR->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			cubeLift->Set(ControlMode::PercentOutput, 0.7);		//Cube lift up
		}
		if(autoCount == 145){
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
		}
		if(autoCount == 200){
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
		}
		if (autoCount == 300) {
			driveTrain->ArcadeDrive(0.0, 0.0);					//Drive stop
		}


		if(autoCount == 60){
			driveTrain->ArcadeDrive(-0.8, 0.0);					//Drive forward
		}
		if(autoCount == 135){
			driveTrain->ArcadeDrive(-0.5, 0.0);					//Slow down
		}
		if(autoCount == 160){
			driveTrain->ArcadeDrive(-0.3, 0.0);					//Slow down
		}
		if(autoCount == 170){
			driveTrain->ArcadeDrive(0.0, 0.0);					//Stop
		}

		autoCount++;
	}
	void lineMiddleSpit() {
		//Put a Power Cube in the right side of the closest switch starting from the middle of the field

		if (autoCount == 0) {
			cubeLift->Set(ControlMode::PercentOutput, 0.6);		//Cube lift up
		}
		if (autoCount == 20) {
			cubeLift->Set(ControlMode::PercentOutput, -0.6);	//Cube lift down
		}
		if (autoCount == 40) {
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
			grabberL->Set(ControlMode::PercentOutput, 0.3);		//Grab cube
			grabberR->Set(ControlMode::PercentOutput, -0.3);	//Grab cube
		}

		if (autoCount == 60) {
			grabberL->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			grabberR->Set(ControlMode::PercentOutput, 0.0);		//Grabber stop
			cubeLift->Set(ControlMode::PercentOutput, 0.7);		//Cube lift up
		}
		if (autoCount == 145) {
			cubeLift->Set(ControlMode::PercentOutput, 0.0);		//Cube lift off
		}
		if(autoCount == 170){
			grabberL->Set(ControlMode::PercentOutput, -0.5);	//Spit cube
			grabberR->Set(ControlMode::PercentOutput, 0.5);		//Spit cube
		}
		if(autoCount == 210){
			grabberL->Set(ControlMode::PercentOutput, 0);		//Spit cube
			grabberR->Set(ControlMode::PercentOutput, 0);		//Spit cube
		}


		if(autoCount == 60){
			driveTrain->ArcadeDrive(-0.8, 0.0);					//Drive forward
		}
		if(autoCount == 135){
			driveTrain->ArcadeDrive(-0.5, 0.0);					//Slow down
		}
		if(autoCount == 160){
			driveTrain->ArcadeDrive(-0.3, 0.0);					//Slow down
		}
		if(autoCount == 170){
			driveTrain->ArcadeDrive(0.0, 0.0);					//Stop
		}

		autoCount++;
	}
	//----------------------------

};

START_ROBOT_CLASS(Robot)
