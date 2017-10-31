
package org.usfirst.frc.team4992.robot;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
//import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
//import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;


import org.usfirst.frc.team4992.robot.subsystems.Climb;
import org.usfirst.frc.team4992.robot.subsystems.Drive;
import org.usfirst.frc.team4992.robot.subsystems.ExampleSubsystem;
import org.usfirst.frc.team4992.robot.subsystems.GearLifter;
import com.ctre.CANTalon;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	// variables
	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();// remove
																					// this
																					// then
																					// you
																					// remove
																					// the
																					// example
																					// subsystem
	public static final Climb objClimb = new Climb();
	public static final Drive drive = new Drive();
	public static final GearLifter gear = new GearLifter();
	public static OI oi;
	//
	public static final int ImageWidth = 320;
	protected static final int ImageHeight = 240;
	private static final double INIT_DIST = 2.5;
	private static final double PEG_DIST = 16;
	//
	static ADXRS450_Gyro gyro;
	Ultrasonic ultra;
	// senors and random stuff
	Command autonomousCommand;
	UsbCamera camera;
	UsbCamera secondCamera;
	//NetworkTable visionTable;
	Compressor comp;
	public static double COG_X;
	double COG_Y;
	public static double COG_SIZE;
	double COG_SIZEThresh;
	boolean visionAvailable;

	CANTalon motorLeftBack;
	CANTalon motorRightBack;
	CANTalon motorLeftFront;
	CANTalon motorRightFront;
	CANTalon climberMotor;
	public static RobotDrive driveRobot;
	Talon motorLeftBackTest;
	Talon motorRightBackTest;
	Jaguar motorLeftFrontTest;
	Jaguar motorRightFrontTest;
	public static RobotDrive driveRobotTest;
	public static boolean reverseDriveActive = false;
	public boolean slowDrive = false;
	public double drivePrefix = 1;
	public boolean aButton = false;
	public boolean bButton = false;
	public boolean xButton = false;
	public boolean yButton = false;
	public boolean startButtons = false;
	static double  leftPower = 0.0;
	double rightPower = 0.0;
	static double maxTurnSpeed = 0.2;
	static int autoSteps;
	int ticks = 0;
	boolean run = true;
	boolean continueToDropGear;//Set in Automus init

	// Pneumatic Variables
	public static DoubleSolenoid arms;
	boolean armsOn = false;
	public static DoubleSolenoid plate;
	boolean plateOn = false;
	// NetworkTable visionTable;
	boolean leftStick = false;
	// Variables and constants to control the climber
	double maxClimberCurrent = 0;
	double[] climberCurrent = {0.0,0.0,0.0};
	boolean climberEnable = false;
	boolean allowClimberDown = true;
	// Debug Statement control
	static boolean DEBUG_DRIVE = false;
	static boolean DEBUG_CLIMBER = true;
	int[] ultraSmooth = new int[5];
	boolean dropGear;
	boolean goSlow;
	double pos;
	//drivestriaght varibles
	double angleStr =0;
	double distStr =0;
	double speedStr =0;
	boolean readyStr = true;
	
	double gyroRead1 = 0;
	double gyroRead2 = 0;
	// ------------------------FRC methods-------------------------
	public void robotInit(){
		
		gyro = new ADXRS450_Gyro();
		gyro.calibrate();
		gyro.reset();
		oi = new OI();
		comp = new Compressor();
		comp.clearAllPCMStickyFaults();
		comp.stop();
		arms = new DoubleSolenoid(RobotMap.solenoidForwardArms, RobotMap.solenoidReverseArms);
		plate = new DoubleSolenoid(RobotMap.solenoidPlatformForwards, RobotMap.solenoidPlatformReverse);
	
		motorLeftBack = new CANTalon(RobotMap.backLeftMotor);
		motorRightBack = new CANTalon(RobotMap.backRightMotor);
		motorLeftFront = new CANTalon(RobotMap.frontLeftMotor);
		motorRightFront = new CANTalon(RobotMap.frontRightMotor);
		climberMotor = new CANTalon(RobotMap.climberGear);

		driveRobot = new RobotDrive(motorLeftFront, motorLeftBack, motorRightFront, motorRightBack);
		motorLeftBackTest = new Talon(1);
		motorRightBackTest = new Talon(2);
		motorLeftFrontTest = new Jaguar(3);
		motorRightFrontTest = new Jaguar(4);
		driveRobotTest = new RobotDrive(motorLeftFrontTest, motorLeftBackTest, motorRightFrontTest, motorRightBackTest);
		// driveTestBot
		reverseDriveActive = false;

		// Camera setup for vision
		camera = CameraServer.getInstance().startAutomaticCapture();
		MjpegServer server = new MjpegServer(CameraServer.getInstance().toString(), 1181);
		visionAvailable = true;

		ultra = new Ultrasonic(1, 2);
		ultra.setAutomaticMode(true);

	}

	public void testInit() {
		
		System.out.println("Up date done ready to tell");
		motorRightBack.setEncPosition(0);
		
		//driveDist(gyro.getAngle(), 8 , 0.5);
	}

	public void testPeriodic() {
		if(OI.buttonX.get()){
			
			gyro.reset();
		}
		
		if(OI.buttonA.get()){
			
			gyro.calibrate();
		}
		
		//System.out.println("\nGyro Reading\n" + (gyro.getAngle()-gyroRead1) + " : " + (gyroRead1-gyroRead2) + " : " + gyro.getAngle() + "\n");
		
		gyroRead2 =  gyroRead1;
		gyroRead1 = gyro.getAngle();
		//System.out.println(gyro.getAngle() + " space " + gyro.getAngle()%360 );
		//System.out.println("LB" +motorLeftBack.getEncPosition());
		//System.out.println("RB" +motorRightBack.getEncPosition());//*
		//System.out.println("LF" + motorLeftFront.getEncPosition());//*
		//System.out.println("RF" + motorRightFront.getEncPosition());
		
		/*
		NetworkTable table = NetworkTable.getTable("Preferences");
		double tempPower = table.getDouble("power",0.0);
		double tempturn = table.getDouble("turn",0.0);
		
		driveRobot.arcadeDrive(-OI.stick.getY(),-OI.stick.getX());
		
		if(OI.buttonA.get()){
			driveRobot.arcadeDrive(tempPower,tempturn);
		}
		*/
		
		//Reverse the climber motor
		// Climber motor contorls
		if (OI.rightBumper.get() && OI.leftBumper.get() ) {
			climberMotor.set(1);
		} else if (OI.leftBumper.get()) {
			climberMotor.set(-1);
		} else {
			climberMotor.set(0);
		}
		
		if(OI.startRightButton.get() ){
			driveDist(gyro.getAngle(),2,0.5);
			System.out.println("reset" + gyro.getAngle() );
		}
		
		if(OI.buttonY.get()){
			autoAssitDrive();
		} else {
			driveRobot.arcadeDrive(0,0);
		}
		
	}

	public void autonomousInit() {
		
		
		//DO NOT DROP THE GEAR IN AUTOMUS (true = drop , false = do not drop)
		
		run = true;
		
		
	}

	public void autonomousPeriodic() {
		
		
		if(run){
			//driveToDist(1.5, gyro.getAngle() ) ;
			driveRobot.setSafetyEnabled(false);
			driveRobot.arcadeDrive(0.4, 0.0);
			Timer.delay(2.0);
			//driveRobot.arcadeDrive(0.0, 0.0);
			run=false;
		}
		driveRobot.setSafetyEnabled(true);
		
	
	}

	public void teleopInit() {
		comp.setClosedLoopControl(true);
		driveRobot.arcadeDrive(0, 0);
		motorLeftFront.setEncPosition(0);
		motorRightBack.setEncPosition(0);
		// ---------------------------Z
		// Climber Variable Initialization
		maxClimberCurrent = 0;
		climberEnable = true;
		allowClimberDown = false;
		NetworkTable table = NetworkTable.getTable("Preferences");
		goSlow = table.getBoolean("goSlow",false);


	}

	public void teleopPeriodic() {
		//System.out.println(motorRightBack.getEncPosition());
		

		// comp.stop();
		// BEGIN Climber Code
		// If left trigger and climber enable, then activate climber
		if (OI.rightBumper.get() && climberEnable) {

			// Calculate average climber current over 3 ticks
			double tempCurrent = climberMotor.getOutputCurrent();
			climberCurrent[2] = climberCurrent[1];
			climberCurrent[1] = climberCurrent[0];
			climberCurrent[0] = tempCurrent;
			double avgCurrent = (climberCurrent[2] + climberCurrent[1] + climberCurrent[0]) / 3;

			// DEBUG - Find max climber current
			if (tempCurrent > maxClimberCurrent) {
				maxClimberCurrent = tempCurrent;
				if (DEBUG_CLIMBER) {
					System.out.println("maxClimberCurrent: " + maxClimberCurrent);
				}
			}

			// DEBUG Output climber debug statements
			if (DEBUG_CLIMBER) {
				System.out.println("Current: " + tempCurrent);
				System.out.println("AVG Current: " + avgCurrent);
				// System.out.println(" Voltage: " +
				// climberMotor.getOutputVoltage());

			}

			// THIS IS THE CURRENT CUT OFF LIMIT
			// Turn off motor if avg current threshold reached
			if (avgCurrent >= RobotMap.currentThreshHold) {
				climberMotor.set(0);
				climberEnable = false;
			} // otherwise turn on climber
			else {
				climberMotor.set(-1);//forwards
			}

			// Turn off the climber
		} else {
			climberMotor.set(0);
		}//end of if else for climber motor

		// END Climber Code

		if (DEBUG_DRIVE) {
			System.out.println("Right Pos: " + motorRightBack.getEncPosition() + "/t   Right Vel: "
					+ motorRightBack.getEncVelocity());
			System.out.println("Left Pos: " + motorLeftFront.getEncPosition() + "/t  Left Vel: "
					+ motorLeftFront.getEncVelocity());
		}


		// A button (reverse drive)
		if (OI.buttonA.get()) {
			System.out.println("A");
			
		}//end of a button pressed
				
		
		// b Button (resever drive)
		if (OI.buttonB.get() && !bButton) {
			System.out.println("B");
			reverseDriveActive = !reverseDriveActive;
			bButton = true;
		} else if (!OI.buttonB.get()) {
			bButton = false;
		}
		

		// Button x (plate for the arms)
		// Changes values of plateOn every time button is pressed
		if (OI.buttonX.get() && !xButton) {
			System.out.println("X");
			if (plateOn){
				plateOn = false;
			}
			else {
				plateOn = !plateOn;
			}

			xButton = true;
		} else if (!OI.buttonX.get()) {
			xButton = false;
		}

		// Button Y (Gear Gripping Arms)
		// Changes values of armsOn iff plateOn is true (ie extended)
		if (OI.buttonY.get() && !yButton ) {
			System.out.println("Y");
			armsOn = !armsOn;
			yButton = true;
		} else if (!OI.buttonY.get()) {
			yButton = false;
		}

	
		// The POV
	
		double slowPrefix;
		System.out.println(goSlow);
		if (goSlow){
			slowPrefix = 2;
		}
		else{
			slowPrefix = 1;
		}
		if (OI.stick.getPOV() == 180) {//down
			drivePrefix = 0.5;
		} else if (OI.stick.getPOV() == 90) {//right
			drivePrefix = 0.7/slowPrefix;
		} else if (OI.stick.getPOV() == 0) {//up
			drivePrefix = 1/slowPrefix;
		} else if (OI.stick.getPOV() == 270) {//left
			drivePrefix = 0.7/slowPrefix;
		}

		
		
	//end of if for POV stick-unless(will modifications)
		if (reverseDriveActive) {
			drivePrefix = -Math.abs(drivePrefix);
		} else {
			drivePrefix = Math.abs(drivePrefix);
		}
	
		// ---------------End of button ------------------

		//

		//

		// BEGIN Pneumatic Code
		// piston stuff
		if (plateOn) {
			plate.set(DoubleSolenoid.Value.kForward);
		} else {
			plate.set(DoubleSolenoid.Value.kReverse);
		}

		if (armsOn) {
			arms.set(DoubleSolenoid.Value.kForward);
		} else {
			arms.set(DoubleSolenoid.Value.kReverse);
		}

		// END Pneumatic Code
		
		
		driveRobot.arcadeDrive(-OI.stick.getRawAxis(1) * drivePrefix, -OI.stick.getRawAxis(0) * drivePrefix);
	}// end of teleoperation periodic

	// -------------Other non FRC provided methods-------------------------

	public boolean switchReverseDrive(boolean currentReverseDriveBooleanValue) {// use
																				// this
																				// method
																				// by
																				// setting
																				// the
																				// revsere
																				// boolean
																				// to
																				// this
																				// method
																				// and
																				// put
																				// itself
																				// in
																				// the
																				// parameter
																				// EXAMPLE:
																				// reverseOn
																				// =
																				// switchReverseDrive(reverseOn);
		if (currentReverseDriveBooleanValue) {
			return false;
		} else {
			return true;
		} // end of if-else
	}// end of switchReverseDrive method
	
	//turn to given angle (0-360) (still need a "slow down")
	public void turnToAngle(double absoluteHeading){
		double roundedGyro = gyro.getAngle();//temp varible for what the modulus value of the gyro
		//% the gyro reading
		if(roundedGyro>0){
			roundedGyro%= 360;
		} else {
			roundedGyro = roundedGyro%360 + 360;
		}
		//calcuation which side is better to turn to 
		double temp = absoluteHeading - roundedGyro;
		if(temp>-15 && temp<15){
			driveRobot.arcadeDrive(0,0);//dead zone is + or - 15
		}else if(temp>0 && temp<180 || temp<-180){
			driveRobot.arcadeDrive(0, -0.4);//right hopefully
		} else {
			driveRobot.arcadeDrive(0, 0.4);//left hopefully
		}
		//if you are still confused and read all the comments then ask me I guess
	}
	
	public void autoAssitDrive(){
		//System.out.println(readyStr);
		if(readyStr){
			driveDist();
		} 
	}
	
	public void driveDist(){
		//rotations = meters*2.0833; so 2.0833 Roatation = 1 meter
		//1 r = 1440 ticks
		//System.out.println(motorRightBack.getEncPosition()/(2.0883*1440) + " " + distStr);
		//System.out.println( (motorRightBack.getEncPosition()/(2.0883*1440) < distStr) + "  " + speedStr);
		if(Math.abs(motorRightBack.getEncPosition()/(2.0883*1440)) < distStr){///Warming might not be right wheel
			double angleOff= -(angleStr-gyro.getAngle())/2.0;
			System.out.println((int)angleStr + " " + ((int)gyro.getAngle())  + "raw:" + angleOff );
			if(Math.abs(angleOff)>Math.abs(speedStr)/2){
				angleOff=speedStr*Math.signum(angleOff)/2;
			}
			driveRobot.arcadeDrive(speedStr,angleOff);//might need to add a constant on second parameter
		} else {
			readyStr = false;
			driveRobot.arcadeDrive(0,0);
			//System.out.println("Done!!!!!!!!");
		}
	}
	

	public void driveDist(double inputAngle, double dist, double speed){
		angleStr = inputAngle;
		distStr = dist;
		speedStr = speed;
		readyStr = true;
		motorRightBack.setEncPosition(0);
	}
	
	//this funcation take input from joystick (1 to 180,-1 to -180) to normal (0 to 360)
	public void turnToAngleWithJoyAngle(double absoluteHeading){
		if(absoluteHeading<0){
			turnToAngle(absoluteHeading+360);
		} else {
			turnToAngle(absoluteHeading);
		}
		
	}
	
	
	
	// Sets the rumble of the joystick - smallRotateVal is the rumble for the
	// left side - largeRotateVal is for the right side
	public void rumbleJoystick(float smallRumbleVal, float largeRumbleVal) {
		OI.stick.setRumble(RumbleType.kLeftRumble, smallRumbleVal);
		OI.stick.setRumble(RumbleType.kRightRumble, largeRumbleVal);
	}// end of rumble method

	
}
