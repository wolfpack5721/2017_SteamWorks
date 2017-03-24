#include "ahrs.h"
#include "Profile.h"
#include "wpilib.h"

using namespace frc;

/*
 * 2017 Robot for Team 5721 - Steamworks
 *
 *PWM 0		Left Side Motors
 *PWM 1		Right Side Motors
 *PWM 2		Winch Motor
 *DIO 0,1  	Encoder Left Side
 *PCM 0     Solenoid for gear hanger  (wire in between PDP and RoboRio on CAN bus)
 *Camera	Axis visual camera, ethernet to radio, smartdashboard url = http://10.57.21.19:5800/mjpg/video.mjpg
 *NavX		MXP port on RoboRIO
 *
 *
 *TankDrive			Two joysticks
 *Hang Gear			Right joystick trigger
 *Winch IN			Left joystick button 3
 *Winch OUT			Both joystick button 4 - not for use in competition
 *Drive Direction	Left joystick throttle   up for forward - down for reverse
 *Winch Speed   	Right joystick throttle  up for forward - down for reverse
 *
 *
 *Version 1.0  - 1/20/2017 - CRM  baseline program - untested
 *Version 1.1  - 1/29/2017 - CRM  added shooter speed control
 *Version 1.2  - 2/02/2017 - CRM  adde3d GetHeading function to normalize yaw from navx
 *								  added ZeroHeading function to provide offset
 *								  corrected steering error in Profile.cpp
 *								  assigned better buttons for attack3 joystick
 *Version 1.3  - 2/06/2017 - CRM  hard-coded servo positions and removed from preferences
 *								  added autonomous shooting profiles
 *Version 1.4  - 2/09/2017 - CRM  changed drive direction control to left throttle
 *								  changed joystick controls for extreme 3d joysticks
 *Version 1.5  - 2/26/2017 - CRM  autonomous steer gains must follow motor direction
 *					              pixy steer gain is fixed
 *Version 1.6  - 2/28/2017 - CRM  corrected steer direction indication
 *Version 1.7  - 3/01/2017 - CRM  made corrections for shooter delay in autonomous
 *								  fixed problem in steering direction
 *								  fixed problem with starting backwards by swapping normal drive direction (swap pwm wires)
 *Version 1.8  - 3/06/2017 - CRM  made corrections for distances and angles in autonomous
 *								  corrected strafe directions for reversed
 *Version 1.9  - 3/07/2017 - CRM  made right stick 3 button run servo and agitator motor
 *                                added call to getpixy in teleop
 *                                more changes to distances and angles after testing at batesville
 *                                slow down for 12" before gear hang
 *Version 1.91 - 3/12/2017 - CRM  cleanup preferences
 *Version 1.92 - 3/14/2017 - CRM  Add pneumatics for new gear hanger
 *								  removed shooter, agitator, intake, feed servo and strafe motor
 *Version 1.93 - 3/17/2017 - CRM  add gear hang in autonomous
 */

class Robot: public IterativeRobot
{
private:
	LiveWindow* LW = LiveWindow::GetInstance();
	VictorSP MotorLeftSide, MotorRightSide;
	VictorSP MotorWinch;
	RobotDrive DriveTrain;
	Encoder LeftEncoder,RightEncoder;
	Joystick StickLeft, StickRight;
	PowerDistributionPanel PDP;
	Compressor PCM;
	Solenoid GearHanger;
	SendableChooser<std::string> Chooser;
	const std::string AutoDoNothing = "AutoDoNothing";
	const std::string AutoCrossBaseLine1 = "AutoCrossBaseLine1";
	const std::string AutoCrossBaseLine2 = "AutoCrossBaseLine2";
	const std::string AutoCrossBaseLine3 = "AutoCrossBaseLine3";
	const std::string AutoHangGear1 = "AutoHangGear1";
	const std::string AutoHangGear2 = "AutoHangGear2";
	const std::string AutoHangGear3 = "AutoHangGear3";
	std::string AutoSelected;

	Preferences *Prefs;
	float WinchSpeed = -0.50f;     		// speed of motor for winch
	//e4t = 1440 PPR
	// DistancePerPulse = (wheeldiameter * 3.1415)/360
	// = 4 * 3.1415/360 = .03491
	float DistancePerPulse = 0.03491f; 	// distance per pulse for e4t encoder

	Profile AutoSteer;
	std::string LastError = "";
	std::string NowError = "";
	uint64_t AutoTime = 0;
	uint64_t GearHangTime = 0;
	uint64_t ElapsedTime = 0;
	bool SteerReversed = false;
	float HeadingOffset = 0.0f;
	AHRS *ahrs; //navX MXP

public:
	Robot() :
	MotorLeftSide(0),
	MotorRightSide(1),
	MotorWinch(2),
	DriveTrain(MotorLeftSide,MotorRightSide),
	LeftEncoder(0, 1, true, CounterBase:: k4X),    //DIO 0 and 1 channels
	RightEncoder(2, 3, true, CounterBase:: k4X),    //DIO 2 and 3 channels
	StickLeft(0),
	StickRight(1),
	PDP(0),
	PCM(1),          //pneumatic control module - CAN ID = 1
	GearHanger(1,0), //single solenoid on pcm ID 1, channel 0
	Chooser(),
	Prefs()
	{
		//instantiate the navx
		try
		{
	        ahrs = new AHRS(SPI::Port::kMXP);
        }
		catch (std::exception& ex )
        {
            std::string err_string = "Error instantiating navX MXP:  ";
            err_string += ex.what();
            DriverStation::ReportError(err_string.c_str());
        }
        if ( ahrs )
        {
            LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
        }
	}

	void RobotInit()
	{
		printf("[RobotInit]\n");
		SmartDashboard::PutString("Code Version","1.93");
		DriveTrain.SetSafetyEnabled(false);
		MotorWinch.SetSafetyEnabled(false);
		//choices for autonomous mode
		Chooser.AddDefault(AutoDoNothing, AutoDoNothing);
		Chooser.AddObject(AutoCrossBaseLine1, AutoCrossBaseLine1);
		Chooser.AddObject(AutoCrossBaseLine2, AutoCrossBaseLine2);
		Chooser.AddObject(AutoCrossBaseLine3, AutoCrossBaseLine3);
		Chooser.AddObject(AutoHangGear1, AutoHangGear1);
		Chooser.AddObject(AutoHangGear2, AutoHangGear2);
		Chooser.AddObject(AutoHangGear3, AutoHangGear3);
		SmartDashboard::PutData("Auto Modes", &Chooser);

		Prefs = Preferences::GetInstance();
		//INVERT MOTOR DIRECTION HERE
		MotorLeftSide.SetInverted(true);
		MotorRightSide.SetInverted(true);
		//must invert gains to follow motor direction
		if(MotorLeftSide.GetInverted())
		{
			AutoSteer.SteerKp = -0.01;
			AutoSteer.TurnKp = -0.05;
		}
		else
		{
			AutoSteer.SteerKp = 0.01;
			AutoSteer.TurnKp = 0.05;
		}
		AutoSteer.Initialize();
		MotorWinch.SetInverted(false);

		//turn on pneumatic controller
		PCM.SetClosedLoopControl(true);
		//retract gear hanger
		GearHanger.Set(false);
		//setup the axis camera feed to smartdashboard
		//cs::AxisCamera camera = CameraServer::GetInstance()->AddAxisCamera("10.60.55.11");
		//camera.SetResolution(320, 240);
		//camera.SetFPS(15);
		SmartDashboard::PutNumber("Heading",0);
		SmartDashboard::PutNumber("Distance",0);
		SmartDashboard::PutString("Drive Direction","???");
		LeftEncoder.SetDistancePerPulse(DistancePerPulse);
		RightEncoder.SetDistancePerPulse(DistancePerPulse);
	}

	double GetHeading()
	{
		double offsetYaw = ahrs->GetYaw() - HeadingOffset;
		if(offsetYaw > 180) offsetYaw -= 360;
		else if(offsetYaw < -180) offsetYaw  += 360;
		if(offsetYaw < 0) offsetYaw += 360;
		return offsetYaw;
	}

	void ZeroHeading()
	{
		double offset;
		offset = ahrs->GetYaw();
		if(offset > 180) offset -= 360;
		else if(offset < -180) offset  += 360;
		if(offset < 0) offset += 360;
		HeadingOffset = offset;
	}

	void AutonomousInit() override
	{
		printf("[AutonomousInit]\n");
		//zero the gyro to current heading
		ZeroHeading();
		//zero the encoders
		LeftEncoder.Reset();
		RightEncoder.Reset();
		//get autonomous selection from smartdashboard
		AutoSelected = Chooser.GetSelected();
		std::cout << "Auto selected: " << AutoSelected << std::endl;
		AutoSteer.ClearProfile();
		AutoSteer.ProfileLoaded = false;
		//DEFINE AUTONOMOUS PROFILES HERE
		//baseline is 93"
		//1 and 3 positions are 90" from center of field to center of robot
		//robot always uses 0-360 degree, with 0 being direction pointed at startup
		if(AutoSelected == AutoHangGear1)
		{
			AutoSteer.AddStep(1,-0.5,78,0,0);    //MOVE, half speed, 78 inches
			AutoSteer.AddStep(2,-0.35,60,0,0);   //TURN, slow speed, 60 deg right
			AutoSteer.AddStep(1,-0.5,43,0,0);    //MOVE, half speed, 34 inches
			AutoSteer.AddStep(1,-0.35,4,0,0);     //MOVE, slow speed, 8 inches
			AutoSteer.ProfileLoaded = true;
		}
		if(AutoSelected == AutoHangGear2)
		{
			AutoSteer.AddStep(1,-0.5,64,0,0);    //MOVE, half speed, 64 inches
			AutoSteer.AddStep(1,-0.35,8,0,0);     //MOVE, slow speed, 12 inches
			AutoSteer.ProfileLoaded = true;
		}
		if(AutoSelected == AutoHangGear3)
		{
			AutoSteer.AddStep(1,-0.5,75,0,0);    //MOVE, half speed, 80 inches
			AutoSteer.AddStep(2,-0.35,300,0,0);  //TURN, slow speed, 60 deg left
			AutoSteer.AddStep(1,-0.50,34,0,0);   //MOVE, half speed, 34 inches
			AutoSteer.AddStep(1,-0.35,4,0,0);     //MOVE, slow speed, 8 inches
			AutoSteer.ProfileLoaded = true;
		}
		if(AutoSelected == AutoCrossBaseLine1 || AutoSelected == AutoCrossBaseLine3)
		{
			AutoSteer.AddStep(1,-0.5,78,0,0);    //MOVE, half speed, past baseline
			AutoSteer.ProfileLoaded = true;
		}
		if(AutoSelected == AutoCrossBaseLine2)
		{
			AutoSteer.AddStep(1,-0.5,69,0,0);    //MOVE, half speed, past baseline
			AutoSteer.ProfileLoaded = true;
		}
		AutoTime = GetFPGATime();
	}

	uint16_t filter(uint16_t raw, uint16_t current, double lpf)
	{
		return (uint16_t)(lpf * raw) + ((1-lpf) * current);
	}

	void AutonomousPeriodic()
	{
		double angle = GetHeading();
		double outputM, curve;
		static uint8_t gearHangState = 0;
		double distance = LeftEncoder.GetDistance();

		SmartDashboard::PutNumber("Heading",angle);
		SmartDashboard::PutNumber("Distance",distance);
		//execute autonomous profile setup in AutonomousInit
		if(AutoSteer.ProfileLoaded)
		{
			AutoSteer.ExecuteProfile(angle,distance,0,false);
			//these will be set to 0 after profile is finished
			//can be set below to move if needed
			outputM = AutoSteer.OutputMagnitude;
			curve = AutoSteer.Curve;
			//if we need to hang gear after movements are done
			if((AutoSelected == AutoHangGear1 || AutoSelected == AutoHangGear2 || AutoSelected == AutoHangGear3) && AutoSteer.StepsDone)
			{
				if(gearHangState == 0)
				{
					gearHangState = 1;
					GearHangTime = GetFPGATime();
					printf("GearHang Extend\n");
					GearHanger.Set(true);
				}
				if(gearHangState == 1)  //wait for gear hanger to reach release position
				{
					ElapsedTime = GetFPGATime();
					ElapsedTime = (ElapsedTime - GearHangTime) / 1000;
					if(ElapsedTime >= 750 )//<<< this is amount of time to stay "pushed out" in milliseconds
					{
						GearHanger.Set(false);
						gearHangState = 2;
						printf("GearHang Retract\n");
					}
				}
				/*if(gearHangState == 2)  //back the robot away
				{
					ElapsedTime = GetFPGATime();
					ElapsedTime = (ElapsedTime - GearHangTime) / 1000;
					if(ElapsedTime < 1750 )
					{
						outputM = 0.5;
						curve = 0.0;
					}
					else
					{
						gearHangState = 3;
						printf("Stop Reversing\n");
					}

				}*/
			}
			DriveTrain.Drive(outputM,curve);
		}
		else   //not running auto profile so turn drive and shooter stuff off
		{
			DriveTrain.Drive(0.0,0.0);
		}
		//turn these off all the time in auto
		MotorWinch.Set(0.0);
	}

	void TeleopInit()
	{
		printf("[TeleopInit]\n");

	}

	void TeleopPeriodic()
	{
		float leftRaw = 0.0f;         // left side joystick input
		float rightRaw = 0.0f;        // right side joystick input
		float tmpWinchSpd = 0.0f;

		//extend gear hanger when right joystick trigger pulled
		if (StickRight.GetRawButton(1))
			GearHanger.Set(true);
		else
			GearHanger.Set(false);

		//climb rope when left joystick button 3 pressed
		//winch speed is determined by right joystick throttle
		tmpWinchSpd = 0.0f;
		if (StickLeft.GetRawButton(3))
		{
			//normalize throttle to 0 to -1 - forward only
			tmpWinchSpd = StickRight.GetRawAxis(3);
			tmpWinchSpd = (tmpWinchSpd - 1)/2;
		}
		//reverse winch when both joystick button 4 pressed
		if (StickLeft.GetRawButton(4) && StickLeft.GetRawButton(4))
			tmpWinchSpd = 0.4;
		MotorWinch.Set(tmpWinchSpd);

		//toggle drive direction
		SteerReversed = StickLeft.GetRawAxis(3) > 0;  //throttle motor down +1
		if (SteerReversed) SmartDashboard::PutString("Drive Direction","REVERSED");
		else SmartDashboard::PutString("Drive Direction","NORMAL");

		//get raw joystick values for driving
		leftRaw = StickLeft.GetRawAxis(1);
		rightRaw = StickRight.GetRawAxis(1);
		//protect from out of range inputs
		if (leftRaw > 0.99) leftRaw = 0.99;
		if (leftRaw <-0.99) leftRaw = -0.99;
		if (rightRaw > 0.99) rightRaw = 0.99;
		if (rightRaw < -0.99) rightRaw = -0.99;
		if(SteerReversed)
		{
			leftRaw = leftRaw * -1;
			rightRaw = rightRaw * -1;
			DriveTrain.TankDrive(rightRaw,leftRaw);
		}
		else DriveTrain.TankDrive(leftRaw,rightRaw);
	}

	void TestInit() override
	{
		printf("[TestInit]\n");
	}

	void TestPeriodic()
	{
		LW->Run();
		DriveTrain.TankDrive(StickLeft.GetRawAxis(1),StickRight.GetRawAxis(1));
	}

	void DisabledInit() override
	{
		printf("[Robot was DISABLED]\n");
	}
};

START_ROBOT_CLASS(Robot)
