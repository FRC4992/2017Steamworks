package org.usfirst.frc.team4992.robot;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	//systems - drive, climb, gear
	//CANtalon will be from 70-75
	
	//Drive
	public static int
		backLeftMotor = 42,//CANtalon
		backRightMotor = 43,//CANtalon
		frontLeftMotor = 41,//CANtalon
		frontRightMotor = 44,//CANtalon
		//Climb
		climberGear = 40,//CANtalon
		//Gear
		solenoidForwardArms = 0,//Solenoid
		solenoidReverseArms = 1,//Solenoid
		solenoidPlatformForwards = 2,//Solenoid
		solenoidPlatformReverse =3,//Solenoid
		kicker = 10,//Not sure yet
	
		xBoxContorllerPort = 0;
		
	public static double currentThreshHold = 60;
	

	
}
