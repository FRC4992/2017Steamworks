package org.usfirst.frc.team4992.robot.commands;

import org.usfirst.frc.team4992.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 *	
 */
public class StayGoodBoy extends Command {

    public StayGoodBoy() {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Robot.drive.Stop();//This method was deledted and will need to be changed so(possilbe delect this command it is pretty unless)
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
