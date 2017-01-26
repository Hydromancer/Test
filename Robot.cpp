#include "WPILib.h"
#include <cmath>
#include "WiggleCode.h"
//ID NUMBERS
#define LEFTMOTOR 30
#define RIGHTMOTOR 1
#define TOPLIMIT 0
#define BOTTOMLIMIT 1
#define LEFTSLAVE 6
#define RIGHTSLAVE 8
#define INTAKEMOTOR 31
#define RSHOOTERMOTOR 5 //Shoots the ball.
#define LSHOOTERMOTOR 7
#define POSITIONMOTOR 9//This will always be in PID mode. Lowers the intake motors to the specified encoder position.
#define FCLIMBMOTOR 0
#define BCLIMBMOTOR 1
#define GEARSHIFT 0
#define BALLBLOCK 2
#define LIFT 4
//ID NUMBERS END
enum Modes {Intake, Move, Fire, Lifting};
Modes RobotState=Fire;
class Robot: public IterativeRobot {
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	Joystick *RStick = new Joystick(0);
	Joystick *LStick = new Joystick(1); //TankDrive testing
	Joystick *Gunner = new Joystick(2);
	TalonSRX *Fake0 = new TalonSRX(2);//PWM port numbers, mind you, so 0-9.
	TalonSRX *Fake1 = new TalonSRX(3);
	RobotDrive *Drive = new RobotDrive(Fake0,Fake1);

	CANTalon *LeftMotor = new CANTalon(LEFTMOTOR);
	CANTalon *RightMotor = new CANTalon(RIGHTMOTOR);
	CANTalon *LeftSlave = new CANTalon(LEFTSLAVE);
	CANTalon *RightSlave = new CANTalon(RIGHTSLAVE);
	CANTalon *IntakeMotor = new CANTalon(INTAKEMOTOR);
	CANTalon *RShooterMotor = new CANTalon(RSHOOTERMOTOR);
	CANTalon *LShooterMotor = new CANTalon(LSHOOTERMOTOR);
	CANTalon *PositionMotor = new CANTalon(POSITIONMOTOR);//Lifts the intake motors. Only one. Use three set positions. One's zero, dummy!
	Talon *FClimbMotor = new Talon(FCLIMBMOTOR);
	Talon *BClimbMotor = new Talon(BCLIMBMOTOR);

	Compressor *Pressor = new Compressor(0);
	DoubleSolenoid *GearShift = new DoubleSolenoid(0,GEARSHIFT,(GEARSHIFT+1));
	DoubleSolenoid *BallBlock = new DoubleSolenoid(0,BALLBLOCK,(BALLBLOCK+1));
	Solenoid *Lift = new Solenoid(0,LIFT);
	Timer *Clock = new Timer;
	Timer *TacticalNuke = new Timer;
	DigitalInput *Photoeye = new DigitalInput(2);
	AnalogGyro *gyro = new AnalogGyro(0);

	IMAQdxSession session;
	Image *frame;
	IMAQdxError imaqError;

	float Miku{0}, gyrodegree{0}, EncCount{0};
	short Mode{0};
	bool ShiftHeld{false}, ShooterHeld{false}, IntakeBool{false}, ShiftReverse{true}, Wiggling{false}, SpitUpBool{false}, Nuke{false}, MathCount{false};//ShiftHeld for gear shift, ShooterHeld for the shooters, IntakeBool for the intake, ShiftReverse to enable the GearShift to turn off.

	void RobotInit() {
		Drive->SetSafetyEnabled(false);
		LeftMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
		RightMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
		LeftMotor->ConfigEncoderCodesPerRev(2048);
		RightMotor->ConfigEncoderCodesPerRev(2048);
		LeftMotor->SelectProfileSlot(0);
		RightMotor->SelectProfileSlot(0);
		LeftSlave->SetControlMode(CANTalon::kFollower);
		LeftSlave->Set(LEFTMOTOR);
		LeftSlave->SetClosedLoopOutputDirection(false);
		RightSlave->SetControlMode(CANTalon::kFollower);
		RightSlave->Set(RIGHTMOTOR);
		RightSlave->SetClosedLoopOutputDirection(false);



	//	IntakeMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
	//	IntakeMotor->SetControlMode(CANTalon::kVoltage);
	//	IntakeMotor->SelectProfileSlot(0);
	//	IntakeMotor->SetPID(.3, 0, 0);
	//	IntakeMotor->SetClosedLoopOutputDirection(false);
	//	IntakeMotor->SetVoltageCompensationRampRate(6); //0 to 12 volts in 200ms.

		RShooterMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
		RShooterMotor->SetControlMode(CANTalon::kSpeed);
		RShooterMotor->SelectProfileSlot(0);
	//	RShooterMotor->SetPID(.3,0,0, 1.2375);
		RShooterMotor->SetClosedLoopOutputDirection(true);
		RShooterMotor->ConfigEncoderCodesPerRev(20); //CIMCoders.
	/*	LShooterMotor->SetFeedbackDevice(CANTalon::QuadEncoder);//
		LShooterMotor->SetControlMode(CANTalon::kSpeed);
		LShooterMotor->SelectProfileSlot(0);
	//  LShooterMotor->SetPID(.3,0,0, 1.2375); p, i, d, feed forward*/
		LShooterMotor->SetClosedLoopOutputDirection(true);/*
		LShooterMotor->ConfigEncoderCodesPerRev(20);*/
		LShooterMotor->SetControlMode(CANTalon::kFollower);
		LShooterMotor->Set(RSHOOTERMOTOR);
		LShooterMotor->SetClosedLoopOutputDirection(true);

		PositionMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
		PositionMotor->SetControlMode(CANTalon::kPosition);
		PositionMotor->ConfigEncoderCodesPerRev(7*71); //All-In-One motors.
		PositionMotor->SelectProfileSlot(0);
	//	PositionMotor->SetPID(.3,0,0);
		PositionMotor->SetClosedLoopOutputDirection(false);
		PositionMotor->SetSensorDirection(true);
		PositionMotor->SetPosition(0);

		Pressor->SetClosedLoopControl(true);//REMEMBER TO KEEP TRUE!
		gyro->SetSensitivity(.007);
		gyro->Calibrate();
		gyro->Reset();

		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		imaqError = IMAQdxOpenCamera("cam1", IMAQdxCameraControlModeController, &session);
		if(imaqError != IMAQdxErrorSuccess) {
			DriverStation::ReportError("IMAQdxOpenCamera error: " + std::to_string((long)imaqError) + "\n");
		}
		imaqError = IMAQdxConfigureGrab(session);
		if(imaqError != IMAQdxErrorSuccess) {
			DriverStation::ReportError("IMAQdxConfigureGrab error: " + std::to_string((long)imaqError) + "\n");
		}
		IMAQdxStartAcquisition(session);

	}

	void DisabledPeriodic() override { /*Exactly What It Says On The Tin.*/
		IMAQdxGrab(session, frame, true, NULL);
		CameraServer::GetInstance()->SetImage(frame);
		Wait(0.01);
        if (SmartDashboard::GetNumber("COG_X", 0.0)!=0)
		Miku = SmartDashboard::GetNumber("COG_X", 0.0);
		SmartDashboard::PutNumber("X-Axis of Target", Miku);
		SmartDashboard::PutNumber("Photoeye", Photoeye->Get());
		SmartDashboard::PutNumber("PositionMotor Position", PositionMotor->GetPosition());
		SmartDashboard::PutNumber("Gyro", gyro->GetAngle());
		}

	void AutonomousInit() {
	//	LeftMotor->SetControlMode(CANTalon::kPosition);
	//	RightMotor->SetControlMode(CANTalon::kPosition);
		LeftMotor->SetPosition(0);
		RightMotor->SetPosition(0);
	//	PositionMotor->SetPosition(0);
		gyro->Reset();
	}

	void AutonomousPeriodic() {
		IMAQdxGrab(session, frame, true, NULL);
		CameraServer::GetInstance()->SetImage(frame);
	    if (SmartDashboard::GetNumber("COG_X", 0.0)!=0)
			Miku=320-(SmartDashboard::GetNumber("COG_X", 0.0));
		switch (Mode){
				case 0:
			         LeftMotor->Set(0.0);
			         RightMotor->Set(0.0);
			         Mode=1;//TESTING ONLY FOR WIGGLE. SET TO 1 OTHERWISE!
			         break;
				case 1:
					Drive->Drive(.85, -(gyro->GetAngle())*.06);//Drive at a .9 throttle forward in low gear to Dukes of Hazard.
					LeftMotor->Set(Fake0->Get());
					RightMotor->Set(Fake1->Get());
					SmartDashboard::PutNumber("Gyro", gyro->GetAngle());
					SmartDashboard::PutNumber("LeftMotor Position", (LeftMotor->GetPosition()));
					SmartDashboard::PutNumber("RightMotor Position", (RightMotor->GetPosition()));
					if(((-LeftMotor->GetPosition())+RightMotor->GetPosition())/2>=12.5){//Test value. 12ish feet. The encoder values match up to feet.
						LeftMotor->StopMotor();
						RightMotor->StopMotor();
					Mode=2;
					}
					break;
				case 2:
				    LeftMotor->SetControlMode(CANTalon::kSpeed);
				    RightMotor->SetControlMode(CANTalon::kSpeed);
					LeftMotor->ConfigPeakOutputVoltage(5, -5);
					RightMotor->ConfigPeakOutputVoltage(5, -5);
				    Mode=3;
		            break;
				case 3:
					if(Miku>=0){
			        LeftMotor->Set(-Miku);
			        RightMotor->StopMotor();
					}
					else{
				        LeftMotor->StopMotor();
				        RightMotor->Set(-Miku);
					}
			        if(abs(Miku)<=10){
			        	LeftMotor->ClearIaccum();
			        	RightMotor->ClearIaccum();
			        	}
			        if(LeftMotor->GetOutputVoltage()<1.5){
			        	Clock->Start();
			        	RShooterMotor->SetSetpoint(4500);
			        }
			        else{
			        	Clock->Stop();
			        	Clock->Reset();
			        }
			        if(Clock->Get()>.4){
					Mode=4;
					LeftMotor->StopMotor();
					RightMotor->StopMotor();
					Clock->Reset();
			        }
					break;
				case 4:
					if(Clock->Get()>3.5){
						BallBlock->Set(DoubleSolenoid::Value::kForward);
						if(PositionMotor->GetPosition()>=.060)
						IntakeMotor->Set(.35);
						PositionMotor->SelectProfileSlot(0);
						PositionMotor->Set(.086+EncCount);
						RobotState=Fire;
					}
					break;
		}
		Wait(.03);
	}

	void TeleopInit() {
		LeftMotor->SetControlMode(CANTalon::kPercentVbus);
		RightMotor->SetControlMode(CANTalon::kPercentVbus);
		IntakeMotor->Set(0);
		LeftMotor->SetPosition(0);
		RightMotor->SetPosition(0);
	//	LShooterMotor->StopMotor();
		RShooterMotor->StopMotor();
        PositionMotor->Set(0);
	}
	void TeleopPeriodic() {//Rotation in degrees is PositionCount/20*360 degrees.
		IMAQdxGrab(session, frame, true, NULL);
		CameraServer::GetInstance()->SetImage(frame);
    	if (SmartDashboard::GetNumber("COG_X", 0.0)!=0)
    	 Miku=320.0-SmartDashboard::GetNumber("COG_X", 0.0);


		Drive->TankDrive(RStick, LStick); //TankDrive testing
		LeftMotor->Set(Fake1->Get());
		RightMotor->Set(Fake0->Get());//Why it's not reversed is beyond me.
		RShooterMotor->ConfigEncoderCodesPerRev(20);
		SmartDashboard::PutNumber("LeftMotor", (LeftMotor->Get()));
		SmartDashboard::PutNumber("RightMotor", (RightMotor->Get()));
		SmartDashboard::PutNumber("LSlave Volts", (LeftSlave->GetOutputVoltage()));
		SmartDashboard::PutNumber("RSlave Volts", (RightSlave->GetOutputVoltage()));
		SmartDashboard::PutNumber("LeftMotor Position", (LeftMotor->GetPosition()));
		SmartDashboard::PutNumber("RightMotor Position", (RightMotor->GetPosition()));
		SmartDashboard::PutNumber("Robot State", RobotState);
		SmartDashboard::PutNumber("RShooter Velocity", RShooterMotor->GetSpeed());
		SmartDashboard::PutNumber("LShooter Get", LShooterMotor->GetEncVel());
		SmartDashboard::PutNumber("IntakeMotor Get", (IntakeMotor->Get()));
		SmartDashboard::PutNumber("Hatsune Miku!!!", Miku);
		SmartDashboard::PutNumber("PositionMotor Position", PositionMotor->GetPosition());
		SmartDashboard::PutNumber("Photoeye", Photoeye->Get());
		SmartDashboard::PutNumber("Gyro", gyro->GetAngle());
		SmartDashboard::PutNumber("Clock", Clock->Get());
		SmartDashboard::PutNumber("Encoder Position Mod", MathCount);

		if (LStick->GetRawButton(10)){
			if (ShiftReverse==true&&ShiftHeld!=true){
				ShiftHeld=true;
				GearShift->Set(DoubleSolenoid::Value::kForward);
				ShiftReverse=false;
			}
			if (ShiftReverse!=true&&ShiftHeld!=true){
				ShiftHeld=true;
				GearShift->Set(DoubleSolenoid::Value::kReverse);
				ShiftReverse=true;
			}
		}
		else{
			ShiftHeld=false;
			GearShift->Set(DoubleSolenoid::Value::kOff);
		}

		if (LStick->GetRawButton(1)){ //Set to intake mode. Lower intake arms via position motor setpoint, activate the intake.
			if(RobotState!=Intake&&IntakeBool!=true){
			IntakeMotor->Set(.40); //6 volts.
			PositionMotor->SelectProfileSlot(1);
			PositionMotor->Set(-.090+EncCount);
			BallBlock->Set(DoubleSolenoid::Value::kReverse);
			RobotState=Intake;
			IntakeBool=true;
			}
			if(RobotState==Intake&&IntakeBool!=true){
				PositionMotor->SelectProfileSlot(0);
				PositionMotor->Set(0);
				IntakeMotor->Set(0);
				RobotState=Fire;
				IntakeBool=true;
			}
		}
		else
			IntakeBool=false;
		if(RobotState==Intake){
			if (Photoeye->Get()&&PositionMotor->GetPosition()<=-.035){
				IntakeMotor->Set(.30);
				Clock->Start();
				if(Clock->Get()>1.8){
				PositionMotor->SelectProfileSlot(0);
				PositionMotor->Set(0+EncCount);
				RobotState=Move;
				Clock->Reset();
				Clock->Stop();
				}
			}
		}
		if (LStick->GetRawButton(3)){//Wiggle.
			if(LeftMotor->GetControlMode()!=CANTalon::kSpeed){
			Wiggling=true;
			LeftMotor->SetControlMode(CANTalon::kSpeed);
			RightMotor->SetControlMode(CANTalon::kSpeed);
			LeftMotor->ConfigPeakOutputVoltage(5, -5);
			RightMotor->ConfigPeakOutputVoltage(5, -5);
			}
			else{
			if(Miku>=0){
	        LeftMotor->Set(-Miku);
	        RightMotor->StopMotor();
			}
			else{
		        LeftMotor->StopMotor();
		        RightMotor->Set(-Miku);
			}
	        if(abs(Miku)<=10){
	        	LeftMotor->ClearIaccum();
	        	RightMotor->ClearIaccum();
	        	}
			}
		}
		else{//That is, if you aren't holding down the button.
			if(Wiggling==true){
				LeftMotor->SetControlMode(CANTalon::kPercentVbus);
				RightMotor->SetControlMode(CANTalon::kPercentVbus);
			//	LeftMotor->ConfigPeakOutputVoltage(10, -10);
			//	RightMotor->ConfigPeakOutputVoltage(10, -10);
				Wiggling=false;
			}
		}
/*
 * All your theoretical Wiggle Code goes here.
 * 		if (LStick->GetRawButton(3)){
			if(LeftMotor->GetControlMode()!=CANTalon::kSpeed){
			Wiggling=true;
			LeftMotor->SetControlMode(CANTalon::kSpeed);
			RightMotor->SetControlMode(CANTalon::kSpeed);
			LeftMotor->ConfigPeakOutputVoltage(5.5, -5.5);
			RightMotor->ConfigPeakOutputVoltage(5.5, -5.5);
			gyro->Reset();
			gyrodegree=Miku*99;//I need to find out the pixels
			}
			else{
			if(Miku>=0){
	        LeftMotor->Set(-Miku);
	        RightMotor->Set(0);
			}
			else{
		        LeftMotor->Set(0);
		        RightMotor->Set(-Miku);
			}
	        if(abs(Miku)<=10){
	        	LeftMotor->ClearIaccum();
	        	RightMotor->ClearIaccum();
	        	}
			}
		}
		else{//That is, if you aren't holding down the button.
			if(Wiggling==true){
				LeftMotor->SetControlMode(CANTalon::kPercentVbus);
				RightMotor->SetControlMode(CANTalon::kPercentVbus);
			//	LeftMotor->ConfigPeakOutputVoltage(10, -10);
			//	RightMotor->ConfigPeakOutputVoltage(10, -10);
				Wiggling=false;
			}
		}
 */


	/*	if (LStick->GetRawButton(3)){//Set to move mode. Raise intake motors and halt them. Make sure to activate that piston Quin just installed.
			if (Clock->Get()<2)
			IntakeMotor->Set(3);
			PositionMotor->SelectProfileSlot(0);
			PositionMotor->Set(0);
		}*/
		if (RStick->GetRawButton(1)&&RShooterMotor->GetOutputVoltage()<=-6.0){//PUSH!
			BallBlock->Set(DoubleSolenoid::Value::kForward);
			if(PositionMotor->GetPosition()>=.060)
			IntakeMotor->Set(.35);
			PositionMotor->SelectProfileSlot(0);
			PositionMotor->Set(.086+EncCount);
			RobotState=Fire;
		}
		if(RStick->GetRawButton(2)){
			if((RShooterMotor->GetSetpoint()!=4500)&&(ShooterHeld==false)){
			RShooterMotor->SetSetpoint(4500);//90%.
		//	LShooterMotor->Set(4750*60/100*2);
			ShooterHeld=true;
			}
			if((RShooterMotor->GetSetpoint()==4500)&&(ShooterHeld==false)){
			RShooterMotor->SetSetpoint(0);
		//	LShooterMotor->Set(4750*60/100*2);
			ShooterHeld=true;
			}
		}
		else if(RStick->GetRawButton(3)){
			if((RShooterMotor->GetSetpoint()!=6000)&&(ShooterHeld==false)){
			RShooterMotor->SetSetpoint(6000);
		//	LShooterMotor->Set(6000*60/100*2);
			ShooterHeld=true;
			}
			if((RShooterMotor->GetSetpoint()==6000)&&(ShooterHeld==false)){
			RShooterMotor->SetSetpoint(0);
		//	LShooterMotor->Set(6000*60/100*2);
			ShooterHeld=true;
			}
		}
		else
			ShooterHeld=false;
		if(LStick->GetRawButton(2)){
			IntakeMotor->Set(-1); //6 volts.
			PositionMotor->SelectProfileSlot(1);
			PositionMotor->Set(-.090+EncCount);
			RobotState=Fire;
		}
		else
			if((RobotState==Move&&PositionMotor->GetPosition()>=-.03)||(RobotState==Fire&&PositionMotor->GetPosition()>=.05)||(IntakeMotor->Get()<=-.75)){
				IntakeMotor->Set(0);
				IntakeMotor->StopMotor();
			}
		if(LStick->GetRawButton(3)){
			if(MathCount!=true)
			EncCount+=.001;
			MathCount=true;
		}
		else if(LStick->GetRawButton(4)){
			if(MathCount!=true)
			EncCount-=.001;
			MathCount=true;
		}
		else
			MathCount=false;
		if(LStick->GetRawButton(5)&&RStick->GetRawButton(5)){
		TacticalNuke->Start();
		if(TacticalNuke->Get()>.5){
			Lift->Set(true);
		    RobotState=Lifting;
			}
		}
		else{
			TacticalNuke->Stop();
			TacticalNuke->Reset();
			Lift->Set(false);
		}
		if(RStick->GetRawButton(4)&&RobotState==Lifting){
			FClimbMotor->Set(1);
			BClimbMotor->Set(-1);
		}
		else if(RStick->GetRawButton(5)&&RobotState==Lifting){
			FClimbMotor->Set(-1);
			BClimbMotor->Set(1);
		}
		else{
			FClimbMotor->Set(0);
			BClimbMotor->Set(0);
		}
		Wait(.01);
	}

	void TestPeriodic() {
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
