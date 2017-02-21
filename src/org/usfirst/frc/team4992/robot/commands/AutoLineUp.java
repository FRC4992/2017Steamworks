package org.usfirst.frc.team4992.robot.commands;

import org.usfirst.frc.team4992.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoLineUp extends Command {
	
	double leftPower;
	double rightPower;
	
    public AutoLineUp() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	leftPower = 0.0;
    	rightPower = 0.0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.COG_X> Robot.ImageWidth/2 ){
    		leftPower = 0.2;
    		//leftPower =  (Robot.ImageWidth/2 - Robot.COG_X); 
    	} else if(Robot.COG_X< Robot.ImageWidth ){
    		rightPower = 0.2;
    		//rightPower =  Robot.COG_X - Robot.ImageWidth/2;
    	}
    	
    	Robot.driveRobot.arcadeDrive( 0.3 , rightPower - leftPower );
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
