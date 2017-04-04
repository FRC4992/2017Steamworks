
package org.usfirst.frc.team4992.robot;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
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

//writing to a text file imports(not use at the current moment
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.Writer;
import java.net.URI;
import java.nio.file.Files;
import java.nio.file.Paths;

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
	//
	static ADXRS450_Gyro gyro;
	Ultrasonic ultra;
	// senors and random stuff
	Command autonomousCommand;
	UsbCamera camera;
	UsbCamera secondCamera;
	NetworkTable visionTable;
	Compressor comp;
	public static double COG_X;
	double COG_Y;
	public static double COG_SIZE;
	double COG_SIZEThresh;
	boolean visionAvailable;

	// Motor OI.sticks
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
	
	//Print to text file
	
	
	//File file = new File("C:/Users/4992/Desktop/log.txt");
	//  PrintWriter basicWrite;
	
	//FileWriter logger = new FileWriter (file.getAbsoluteFile());
	/*
	File fileSecond = new File("log.txt");
	FileOutputStream stream = new FileOutputStream ( file );
	*/
	

	// ------------------------FRC methods-------------------------
	public void robotInit(){
		/*
		try{
			basicWrite = new PrintWriter( file );
		} catch(Exception e){
			
		}
		*/
		
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
		// motorLeftBackTest= new Ja
		// driveTestBot
		reverseDriveActive = false;

		// Camera setup for vision
		camera = CameraServer.getInstance().startAutomaticCapture();
		secondCamera = CameraServer.getInstance().startAutomaticCapture("cam1", 1);
		MjpegServer server = new MjpegServer(CameraServer.getInstance().toString(), 1181);
		visionAvailable = true;
		try {
			visionTable = NetworkTable.getTable("RoboRealm");
		} catch (Exception e) {
			System.out.println("Cannot Connect to RoboRealm");
			visionAvailable = false;
		}
		ultra = new Ultrasonic(1, 2);
		ultra.setAutomaticMode(true);

	}

	public void testInit() {
		motorRightBack.setEncPosition(0);
		
	}

	public void testPeriodic() {
		motorRightBack.setEncPosition(0);
		driveToDist(-2, gyro.getAngle() );
		Timer.delay(5);
		
		System.out.println( motorRightBack.getEncPosition() );
		
		ultra.getRangeInches();
		double leftPower = 0.0;
		double rightPower = 0.0;
		while (OI.buttonA.get() && visionAvailable) {
			COG_X = visionTable.getNumber("COG_X", 0.0);
			if (Robot.COG_X > Robot.ImageWidth / 4) {
				leftPower = 0.2;
				// leftPower = 0.5 * ( (Robot.ImageHeight/4) /(Robot.COG_X) ) -
				// 1;
			} else if (Robot.COG_X < Robot.ImageWidth / 4) {
				rightPower = 0.2;
				// rightPower = 0.5 * (1 - (COG_X/ImageWidth) );
			}
			driveRobot.arcadeDrive(0.4, rightPower - leftPower);
			System.out.println("COGX:" + COG_X);
			System.out.println("Half of image widht:" + ImageWidth / 4);
			leftPower = 0.0;
			leftPower = 0.0;
		}
		System.out.println("inches" + ultra.getRangeInches());
		if (ultra.getRangeInches() < 30) {
			Robot.plate.set(DoubleSolenoid.Value.kReverse);
			Timer.delay(100);
			Robot.arms.set(DoubleSolenoid.Value.kReverse);
			Timer.delay(100);
			Robot.plate.set(DoubleSolenoid.Value.kForward);
			Timer.delay(100);
			Robot.driveRobot.arcadeDrive(-0.5, 0);
			Timer.delay(500);
			Robot.driveRobot.arcadeDrive(0, 0);
		}
		
		//Reverse the climber motor
		// Climber motor contorls
		if (OI.rightBumper.get() && OI.leftBumper.get() ) {
			climberMotor.set(1);
		} else if (OI.leftBumper.get()) {
			climberMotor.set(-1);
		} else {
			climberMotor.set(0);
		}

	}

	public void autonomousInit() {
		
		
		//DO NOT DROP THE GEAR IN AUTOMUS (true = drop , false = do not drop)
		continueToDropGear = false;
		run = true;
		comp.setClosedLoopControl(true);
		NetworkTable table = NetworkTable.getTable("Preferences");
		motorRightBack.setEncPosition(0);
		autoSteps = (int) (table.getNumber("autoMode",4992));
		autoSteps -= 2;
		System.out.println("The sendable choser over the network tables " + autoSteps);
		//gyro.calibrate();
		//gyro.reset();
		
		if (visionAvailable) {
			COG_X = visionTable.getNumber("COG_X", 0.0);
			COG_Y = visionTable.getNumber("COG_Y", 0.0);
			COG_SIZE = visionTable.getNumber("COG_BOX_SIZE", 0.0);
			COG_SIZEThresh = 100;
			System.out.println("COG X:" + COG_X + "\tCOG Y" + COG_Y + "\tCOG SIZE" + COG_SIZE);
		}
		
		
		
	}

	public void autonomousPeriodic() {
		/*
		//Get to the line
		if(run){
			driveToDist( 3.0, gyro.getAngle() );
			run = false;
		}
		*/
		//driveRobot.arcadeDrive(-0.5, 0.0);
		
		switch (autoSteps) {
		
		//Bugs - Randomly drops gear(Done)
				//Stops for 5 seconds(Ish)
				//Super slow(should be fixed)
		
		// The negative cases is used to get the robot close to the peg
		// depending where it start		
		case -3://For starting on the left side
			driveToDist(2.9, gyro.getAngle() );//Drives out striaght alittle 
			goToHeading(60, 0.5);//Trun toward the peg
			while (COG_X == 0) {// Keeps turning until the robot sees the tape
				COG_X = visionTable.getNumber("COG_X", 0.0);
				driveRobot.arcadeDrive(0, 0.3);
			}//end of while
			System.out.println("Switching to case 0 from -3");
			autoSteps = 0;//Swicth to vision case
			break;
		case -2://for staring in the middle
			driveToDist(2, gyro.getAngle());//Currently set to a speed of 0.5 is the driveToDist
			while (ultra.getRangeInches() > 55) {// Keeps going stright until the robot is close the the airship(ultra sonic)
				driveRobot.arcadeDrive(0.3, 0.0);//drive slow
			}//end of while
			System.out.println("Switching to case 0 from -2");
			autoSteps = 0;//Set it to the vision 
			break;
		case -1://For staring on the right side
			driveToDist(2.9, gyro.getAngle());//Drives out a bit
			goToHeading(300, 0.5);//Turn to ward the peg
			while (COG_X == 0) {// Keeps turning until the robot sees the tape
				COG_X = visionTable.getNumber("COG_X", 0.0);
				driveRobot.arcadeDrive(0, -0.3);
			}//end of while
			System.out.println("Switching to case 0 from -1");
			autoSteps = 0;//set to the vison case
			break;
			
		// The generic automus stuff(These(0-3) are called all the  
		case 0:// vision move toward
			//resets the varible so that any turning does not carry over
			double leftPower = 0.0;
			double rightPower = 0.0;
			double maxTurnSpeed = 0.2;

			if (visionAvailable) {// checks to see if vision is available
				COG_X = visionTable.getNumber("COG_X", 0.0);//Updata COG X varible from the network tables
				if (Robot.COG_X > Robot.ImageWidth / 4) {// turn to the right
					leftPower = maxTurnSpeed;
				} else if (Robot.COG_X < Robot.ImageWidth / 4) {// turn to the left
					rightPower = maxTurnSpeed;
				}//end of if-else if(desideing turn direction)
				driveRobot.arcadeDrive(0.4, rightPower - leftPower);//Do the driving with the turing that is desided in the above if-else if statment
				if (ultra.getRangeInches() < 20) {//If close to airship
					System.out.println("Switching to case " + (autoSteps++) + " from 0");
					autoSteps++;
				}
			} else {
				System.out.println("Switching to case 404 from 0");
				autoSteps = 404;
			}

			break;
		case 1://Places the gear on the airship(not tested yet)
			driveRobot.arcadeDrive(0.3,0);//drive a bit
			Timer.delay(1/5);
			driveRobot.arcadeDrive(0,0);//Stop drivin
			plate.set(DoubleSolenoid.Value.kForward);//push arms forwards
			Timer.delay(0.75);
			arms.set(DoubleSolenoid.Value.kForward);//drop gear
			Timer.delay(0.75);
			System.out.println("Switching to case 2 from 1");
			autoSteps++;
			break;
		case 2:// back off and close arms
			driveRobot.arcadeDrive(-0.3, 0.0);//added after 1.0
			Timer.delay(0.5);//added after 1.0
			plate.set(DoubleSolenoid.Value.kReverse);
			driveRobot.arcadeDrive(0.8, 0.0 );
			Timer.delay(10.2);//mod after 1.0
			driveRobot.arcadeDrive(-0.5, 0);
			arms.set(DoubleSolenoid.Value.kReverse); 
			
			System.out.println("Switching to case 3 from 2");
			autoSteps = 3;
			break;
		case 3:// Lines up at zero degrees and stop
			driveRobot.arcadeDrive(0, 0);
			while (goToHeading(0, 0.5)) {
				Timer.delay(1 / 1000);
			}
			System.out.println("Switching to case 403 from 3");
			autoSteps = 403;
		case 403:
			//System.out.println("Endding");
			driveRobot.arcadeDrive(0, 0);
			//Add endding of the case switch here
			break;			
		case 404:
			System.out.println("Vision not available");
			break;
		//User input of 420 will try to land the gear without vision
		
		case 418://Moves the robot within 20 inches
			driveToDist(1.8, gyro.getAngle() );
			while( !(ultra.getRangeInches()<20 ) ){
				driveRobot.arcadeDrive(0.3, 0.0);
			}
			goToHeading(0 , 0.5);//Face forwards
			//418
			//419
			driveRobot.arcadeDrive(0,0);
			plate.set(DoubleSolenoid.Value.kForward);
			Timer.delay(1);
			driveRobot.arcadeDrive(0.3,0.0);
			Timer.delay(0.4);
			arms.set(DoubleSolenoid.Value.kForward);
			Timer.delay(1.4);
			//419
			//2
			driveToDist(-0.1, gyro.getAngle());//IF it pulls it off before lifts plate then mod here
			plate.set(DoubleSolenoid.Value.kReverse);
			driveToDist(-1, gyro.getAngle());
			arms.set(DoubleSolenoid.Value.kReverse); 
			//2
			//3
			driveRobot.arcadeDrive(0, 0);
			while (goToHeading(0, 0.5)) {
				Timer.delay(1 / 1000);
			}
			//3
			
			autoSteps = 403;
			break;
			
		case 419:
			driveRobot.arcadeDrive(0,0);
			plate.set(DoubleSolenoid.Value.kForward);
			Timer.delay(1);
			driveRobot.arcadeDrive(0.3,0.0);
			Timer.delay(0.4);
			arms.set(DoubleSolenoid.Value.kForward);
			Timer.delay(1.4);
			autoSteps = 2;//2 will back off and pull in arms then move to 3 and stop it
			break;
			
		case 4992://Drives striangth
			driveToDist(3,0.4);
			if(continueToDropGear){//If it is going for the straight
				autoSteps = 3;
			} else{//Just pass the line
				autoSteps = 403;
			}
			break;
		default:
				
			
		
		}
			//System.out.println("Here ");
		// if in middle position. drive straight until ultrasonic picks up a
		// certain distance
		// if in side positions, place against wall and:
		// drive forward a set distance
		// turn to face peg using gyro
		// drive forward to peg
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

	}

	public void teleopPeriodic() {
		TeleOpdrive();
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

		// -----------------Start of Buttom----------------------------
<<<<<<< HEAD
		// A button (Tele assited for the gear on to the peg)
		if (OI.buttonA.get()) {
			//**myterious self aware bug(only happened once)
			System.out.println("A");
			if (visionAvailable) {// checks to see if vision is available
				double leftPower = 0.0;
				double rightPower = 0.0;
				double maxTurnSpeed = 0.2;
				COG_X = visionTable.getNumber("COG_X", 0.0);//read form network table
=======
		// A button (reverse drive)
		if (OI.buttonA.get()) {
			System.out.println("A");
			if (visionAvailable) {// checks to see if vision is available

				COG_X = visionTable.getNumber("COG_X", 0.0);
>>>>>>> origin/master
				System.out.println("COCOCOCOCOCOCOCO" +COG_X);
				if (Robot.COG_X > Robot.ImageWidth / 4) {// turn to the right
					leftPower = maxTurnSpeed;
				} else if (Robot.COG_X < Robot.ImageWidth / 4) {// turn to the left
					rightPower = maxTurnSpeed;
				}//if - else if for finding the right or left turn
				driveRobot.arcadeDrive(0.4, rightPower - leftPower);//Drive the robot with aproperet turn
				System.out.println("COGX:" + COG_X);
<<<<<<< HEAD
				System.out.println("Half of image widht:" + ImageWidth / 4);
			}//if vision is availble
		}//end of a button pressed
		
		
=======
				System.out.println("Half of image widht:" + ImageWidth / 4);s
				leftPower = 0.0;
				leftPower = 0.0;
				if (ultra.getRangeInches() < 15) {
					autoSteps++;
				}
			
			aButton = true;
			}
		} 
>>>>>>> origin/master
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
		/*
		// Start buttons
		if ((OI.startLeftButton.get() || OI.startRightButton.get()) && !startButtons) {
			reverseDriveActive = !reverseDriveActive;
			startButtons = true;
		} else if (!OI.startLeftButton.get() || !OI.startRightButton.get()) {
			startButtons = false;
		}
		*/

		// The POV
		if (OI.stick.getPOV() == 180) {
			drivePrefix = 0.5;
		} else if (OI.stick.getPOV() == 90) {
			drivePrefix = 0.65;
		} else if (OI.stick.getPOV() == 0) {
			drivePrefix = 0.8;
		} else if (OI.stick.getPOV() == 270) {
			drivePrefix = 1;
		
	}//end of if for POV stick-unless(will modifications)
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

	}// end of teleoperation periodic

	// -------------Other non FRC provided methods-------------------------
	public static boolean goToHeading(double target, double speed) {

		int hedge = 10;// amount of degrees to stop short of turning
		double heading = (gyro.getAngle() % 360);// Makes sure the gyro has an
													// angle between 0-360
		if (heading < 0) {
			heading += 360;// Adds 360 if angle is negative
		}
		if (Math.abs(heading) > 360) {
			heading = heading % 360;
		}

		int turn = (int) (target - heading);
		if (turn > 180) {
			turn -= 360;
		} else if (turn < -180) {
			turn += 360;
		}
		if (turn > 0 && turn < 180) {
			driveRobot.arcadeDrive(0., -speed);
			// currentDir = -1;
		} else {
			driveRobot.arcadeDrive(0., speed);
		}

		return (Math.abs(turn) >= hedge);

	}

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

	public void driveToDist(double meters, double initHeading) {
		double angleThresh = 5;
		motorRightBack.setEncPosition(0);
		ticks = 0;
		double angle = (gyro.getAngle());
		double rotations = Math.abs(meters * 2.0833);
		if (meters > 0) {
			while (ticks < 1440 * rotations) {
				
				ticks = -motorRightBack.getEncPosition();
				// Lines robot back up to where it started, if it veers off
				if (gyro.getAngle() > (angleThresh + angle) || gyro.getAngle() < (angle - angleThresh)) {
					while (goToHeading(initHeading, 0.4)) {

					}
				}
				driveRobot.arcadeDrive(0.5, 0);
				//System.out.println("Postive ticks" + ticks);
			} // end of first while
		} else {
			while (ticks < 1440 * rotations) {
				ticks = motorRightBack.getEncPosition();
				System.out.println( "ticks " + ticks);
				driveRobot.arcadeDrive(-0.5, 0);
				if (gyro.getAngle() > (angleThresh + angle) || gyro.getAngle() < (angle - angleThresh)) {
					while (goToHeading(initHeading, 0.4)) {

					}
				}
				//System.out.println("Negative ticks:" + ticks);
			} // end of second while
		} // end of if-else
		//System.out.println("Done dist");
		driveRobot.arcadeDrive(0, 0);
	}// end of driveToDist

	// Sets the rumble of the joystick - smallRotateVal is the rumble for the
	// left side - largeRotateVal is for the right side
	public void rumbleJoystick(float smallRumbleVal, float largeRumbleVal) {
		OI.stick.setRumble(RumbleType.kLeftRumble, smallRumbleVal);
		OI.stick.setRumble(RumbleType.kRightRumble, largeRumbleVal);
	}// end of rumble method

	public void TeleOpdrive() {
		double stickX2 = OI.stick.getRawAxis(4);
		double stickY2 = OI.stick.getRawAxis(5);
		double stickX = OI.stick.getRawAxis(0);
		double stickY = OI.stick.getRawAxis(1);
		double turnX = 0;
		double turnY = 0;
		boolean turningAutonomous = false;

		// If the joystick is being moved past half it will activate
		if (((Math.abs(stickY2) > 0.5) || (Math.abs(stickX2) > 0.5)) && turningAutonomous == false) {
			turningAutonomous = true;
			turnX = stickX2;
			turnY = -stickY2;
		} // end of if (joystick is being moved past half it will activate)

		// It will do there automus turning
		if (turningAutonomous) {
			double turnAngle = Math.atan2(turnX, turnY);// Get angle witht the
														// given joystick X and
														// Y value
			turnAngle *= 180 / Math.PI;// convert to degrees

			// need to fix the angle
			if (turnAngle < 0) {
				turnAngle += 360;
			}
			if (Math.abs(turnAngle) > 360) {
				turnAngle = turnAngle % 360;
			}

			turnAngle = 15. * (Math.round(turnAngle / 15));

			// turnAngle = stick.getDirectionDegrees();

			System.out.println("Turn angle" + turnAngle);
			if (!OI.leftStick.get()) {
				System.out.println("Left stick:" + OI.leftStick.get());
				turningAutonomous = goToHeading(turnAngle, 0.6);
			}

		} // end of if(doing the automus turning)
		if (!turningAutonomous) {
			driveRobot.arcadeDrive(-OI.stick.getRawAxis(1) * drivePrefix, -OI.stick.getRawAxis(0) * drivePrefix);

		}

	}// end of absult turning method
}
