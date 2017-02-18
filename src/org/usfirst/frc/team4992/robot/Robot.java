
package org.usfirst.frc.team4992.robot;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CANSpeedController.ControlMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import org.usfirst.frc.team4992.robot.commands.Climber;
import org.usfirst.frc.team4992.robot.subsystems.Climb;
import org.usfirst.frc.team4992.robot.subsystems.Drive;
import org.usfirst.frc.team4992.robot.subsystems.ExampleSubsystem;
import org.usfirst.frc.team4992.robot.subsystems.GearLifter;
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
		
	//variables
	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();//remove this then you remove the example subsystem
	public static final Climb objClimb = new Climb();
	public static final Drive drive = new Drive();
	public static final GearLifter gear = new GearLifter();
	public static OI oi;	
	
	protected static final int ImageWidth = 320;
	protected static final int ImageHeight = 240;
	
	//senors and random stuff
    Command autonomousCommand;
    UsbCamera camera;
    UsbCamera secondCamera;
    NetworkTable visionTable;
    Compressor comp;
    double COGX;
	double COG_Y;
	double COG_SIZE;
	double COG_SIZEThresh;
    
    //Motor controllers
    CANTalon motorLeftBack; 
	CANTalon motorRightBack;
	CANTalon motorLeftFront ;
	CANTalon motorRightFront;
	CANTalon climberMotor;
	public static RobotDrive driveRobot;
	Talon motorLeftBackTest; 
	Talon motorRightBackTest;
	Jaguar motorLeftFrontTest;
	Jaguar motorRightFrontTest;
	public static RobotDrive driveRobotTest;
	public static boolean reverseDriveActive = false;
	public boolean slowDrive =false;
	public double drivePrefix = 1;
	public boolean aButton = false;
	public boolean bButton = false;
	public boolean xButton = false;
	public boolean yButton = false;
	int autoSteps;
	int ticks = 0;
	boolean run = true;
	
	//air stuff
	DoubleSolenoid arms;
	boolean armsOn = false;
	DoubleSolenoid plate;
	boolean plateOn = false;
	//NetworkTable visionTable; 
    
  //------------------------FRC methods-------------------------
    public void robotInit() {

    	oi = new OI();
    	comp = new Compressor();
    	comp.clearAllPCMStickyFaults();
    	comp.stop();    	
    	arms = new DoubleSolenoid(RobotMap.solenoidForwardArms,RobotMap.solenoidReverseArms);
    	plate = new DoubleSolenoid(RobotMap.solenoidPlatformForwards,RobotMap.solenoidPlatformReverse);
    	
        motorLeftBack = new CANTalon(RobotMap.backLeftMotor);
    	motorRightBack = new CANTalon(RobotMap.backRightMotor);
    	motorLeftFront = new CANTalon(RobotMap.frontLeftMotor);
    	motorRightFront = new CANTalon(RobotMap.frontRightMotor);
    	climberMotor = new CANTalon(RobotMap.climberGear);

    	
    	driveRobot = new RobotDrive(motorLeftFront,motorLeftBack,motorRightFront,motorRightBack);
    	motorLeftBackTest = new Talon(1);
    	motorRightBackTest = new Talon(2);
    	motorLeftFrontTest = new Jaguar(3);
    	motorRightFrontTest = new Jaguar(4);
    	driveRobotTest = new RobotDrive(motorLeftFrontTest,motorLeftBackTest,motorRightFrontTest,motorRightBackTest);
    	//motorLeftBackTest= new Ja
    	//driveTestBot
    	reverseDriveActive = false;
    	
    	//Camera setup for vision
    	camera = CameraServer.getInstance().startAutomaticCapture();
    	secondCamera = CameraServer.getInstance().startAutomaticCapture("cam1", 1);
    	MjpegServer server = new MjpegServer(CameraServer.getInstance().toString(),1181);
    	
    	
    }
    
	public void testInit() {
		run = true;
		motorRightBack.setEncPosition(0);
		while(motorRightBack.getEncPosition()<1440*2){
			//ticks += motorLeftFront.getEncPosition();
			System.out.println( motorRightBack.getEncPosition() );
			driveRobot.arcadeDrive(-0.5,0);	
			//System.out.println(ticks);
		}
		driveRobot.arcadeDrive(0,0);	
	}

    public void testPeriodic() {
    	
    	//System.out.println( "Encoder " + motorRightBack.getEncPosition() );
    	/*
    if (run){
    	while(motorLeftFront.getEncPosition()<1440*3){
			//ticks += motorLeftFront.getEncPosition();
			System.out.println( motorLeftFront.getEncPosition() );
			driveRobot.arcadeDrive(0.5,0);	
			//System.out.println(ticks);
		}
    	run = false;
    	
    }
    else{
    	System.out.println ("Right Pos: " + motorRightBack.getEncPosition() + "/t   Right Vel: " +  motorRightBack.getEncVelocity());
    	System.out.println ("Left Pos: "+motorLeftFront.getEncPosition()  + "/t  Left Vel: " +  motorLeftFront.getEncVelocity() );
    	driveRobot.arcadeDrive(0,0);	
    }
    	*/	
        	
     }
    public void autonomousInit() {
    	autoSteps = 0;
    	visionTable= NetworkTable.getTable("RoboRealm");
		COGX = visionTable.getNumber("COG_X", 0.0);
    	COG_Y = visionTable.getNumber("COG_Y", 0.0);
		COG_SIZE = visionTable.getNumber("COG_BOX_SIZE", 0.0);
		COG_SIZEThresh = 100;
    	System.out.println("COG X:" + COGX + "\tCOG Y" + COG_Y + "\tCOG SIZE" + COG_SIZE);
    	run = true;
    }

    public void autonomousPeriodic() {
    	//if in middle position. drive straight until ultrasonic picks up a certain distance
    	//if in side positions, place against wall and:
    		//drive forward a set distance
    		//turn to face peg using gyro
    		//drive forward to peg
    	

    	//driveRobotTest.arcadeDrive(oi.stick);
    	/*
    	if(COG_SIZE<COG_SIZEThresh){
    		driveRobot.arcadeDrive(0.2, 0.0);
    		System.out.println(COG_SIZEThresh);
    	} else {
    		System.out.println("Stoping");
    		driveRobot.arcadeDrive(0.0, 0.0);
    	}
    	*/
		}

    public void teleopInit() {
    	driveRobot.arcadeDrive(0,0);
    	comp.clearAllPCMStickyFaults();
    	comp.setClosedLoopControl(true);
    	motorLeftFront.setEncPosition(0);
    	motorRightFront.setEncPosition(0);
    	//motorLeftFront.configEncoderCodesPerRev(1440);
    	//motorRightBack.configEncoderCodesPerRev(1440);
    	//---------------------------Z

    }
    boolean leftStick = false;
    public void teleopPeriodic() {
    	//comp.stop();

    	//---------------------------------------------
    	System.out.println ("Right Pos: " + motorRightBack.getEncPosition() + "/t   Right Vel: " +  motorRightBack.getEncVelocity());
    	System.out.println ("Left Pos: "+motorLeftFront.getEncPosition()  + "/t  Left Vel: " +  motorLeftFront.getEncVelocity() );
    	
    	if (OI.rightBumper.get() && OI.leftBumper.get()){
    		climberMotor.set(1);
    	} else if(OI.leftBumper.get()){
    		climberMotor.set(-1);
    		
    	} else {
    		climberMotor.set(0);

    	}
    	
    	//A button (reverse drive)
    	if(OI.buttonA.get() && !aButton ){
    		System.out.println("A");
    		reverseDriveActive = switchReverseDrive(reverseDriveActive);
    		aButton = true;
    	} else if (!OI.buttonA.get() ){
    		aButton = false;
    	}
    	//b Button (resever drive)
    	if(OI.buttonB.get() && !bButton ){
    		System.out.println("B");
    		slowDrive = switchReverseDrive(slowDrive);
    		bButton = true;
    	} else if (!OI.buttonB.get() ){
    		bButton = false;
    	}
    	//Button x (plate for the arms)
    	if(OI.buttonX.get() && !xButton ){
    		System.out.println("X");
    		plateOn= switchReverseDrive(plateOn);
    		xButton = true;
    	} else if (!OI.buttonX.get() ){
    		xButton = false;
    	}
    	
    	if(OI.buttonY.get() && !yButton ){
    		System.out.println("Y");
    		armsOn= switchReverseDrive(armsOn);
    		yButton = true;
    	} else if (!OI.buttonY.get() ){
    		yButton = false;
    	}
    	
    	//Sets the controls and stuff based off the booleans which are switched above
    	if(!slowDrive){
    		drivePrefix =1;
    		//System.out.println("fast");
    	} else {
    		drivePrefix =0.3;
    	}
    	
    	if(!reverseDriveActive){
    		driveRobot.arcadeDrive(OI.stick.getRawAxis(1)*drivePrefix,-OI.stick.getRawAxis(0)*drivePrefix );
    	} else {
    		driveRobot.arcadeDrive(-OI.stick.getRawAxis(1)*drivePrefix ,OI.stick.getRawAxis(0)*drivePrefix );
    	}
    	
    	//piston stuff
    	if(plateOn){
    		plate.set(DoubleSolenoid.Value.kForward);
    	} else {
    		plate.set(DoubleSolenoid.Value.kReverse);
    	}
    	
    	if(armsOn){
    		arms.set(DoubleSolenoid.Value.kForward);
    	} else {
    		arms.set(DoubleSolenoid.Value.kReverse);
    	}
    	


    	
    	
    }
    
    
    
    //-------------Other non FRC provided methods-------------------------
    
    public boolean switchReverseDrive(boolean currentReverseDriveBooleanValue){//use this method by setting the revsere boolean to this method and put itself in the parameter EXAMPLE: reverseOn = switchReverseDrive(reverseOn); 
    	if(currentReverseDriveBooleanValue){
    		return false;
    	} else{
    		return true;
    	}//end of if-else
    }//end of switchReverseDrive method
    
	public void driveToDist(double meters){
		
    	double rotations = meters*2.0833;
		while(ticks<1440*rotations){
			ticks = motorLeftFront.getEncPosition();
			System.out.println( motorLeftFront.getEncPosition() );
			driveRobot.arcadeDrive(0.5,0);	
			System.out.println(ticks);
		}
		/*
		  double distance = meter*2992;
		  while(ticks<distance){
			driveRobot.arcadeDrive(0.5,0);	
			ticks = motor
			LeftFront.getEncPosition();
		}
		 */
		driveRobot.arcadeDrive(0,0);	
	}
	
    //Sets the rumble of the joystick - smallRotateVal is the rumble for the left side - largeRotateVal is for the right side
    public void rumbleJoystick(float smallRumbleVal,float largeRumbleVal){
    	OI.stick.setRumble(RumbleType.kLeftRumble, smallRumbleVal);
		OI.stick.setRumble(RumbleType.kRightRumble, largeRumbleVal);
    }//end of rumble method
    
}
