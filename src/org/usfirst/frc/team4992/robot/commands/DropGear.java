package org.usfirst.frc.team4992.robot.commands;

import org.usfirst.frc.team4992.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DropGear extends Command {

    public DropGear() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drive);
    	//requires(Robot.gear);
    	System.out.println("Cons");
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Initializing");
    }
    

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	System.out.println("Doing Shit");
    	Robot.plate.set(DoubleSolenoid.Value.kForward);
    	Timer.delay(1000);
    	Robot.arms.set(DoubleSolenoid.Value.kReverse);
    	Timer.delay(1000);
    	Robot.plate.set(DoubleSolenoid.Value.kForward);
    	Timer.delay(1000);
    	Robot.driveRobot.arcadeDrive(-0.5,0);
    	Timer.delay(500);
    	Robot.driveRobot.arcadeDrive(0,0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
