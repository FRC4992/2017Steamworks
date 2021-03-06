package org.usfirst.frc.team4992.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public static XboxController xStick = new XboxController(0);
	public static Joystick stick = new Joystick(0);
	public static JoystickButton buttonX = new JoystickButton(stick, 3); //SURE ABOUT THE NUMBER
	public static JoystickButton buttonY = new JoystickButton(stick, 4); //SURE ABOUT THE NUMBER
	public static JoystickButton buttonA = new JoystickButton(stick, 1); //SURE ABOUT THE NUMBER
	public static JoystickButton buttonB = new JoystickButton(stick, 2); //SURE ABOUT THE NUMBER
	public static JoystickButton startLeftButton = new JoystickButton(stick, 7);
	public static JoystickButton startRightButton = new JoystickButton(stick, 8);
	public static JoystickButton leftBumper = new JoystickButton(stick, 5);
	public static JoystickButton rightBumper = new JoystickButton(stick, 6);
	public static JoystickButton leftStick = new JoystickButton(stick, 10);
	
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);
    
    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.
    
    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:
    
    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());
    
    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());
    
    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());
}

