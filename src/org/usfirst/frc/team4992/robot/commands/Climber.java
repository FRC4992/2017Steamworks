package org.usfirst.frc.team4992.robot.commands;

import org.usfirst.frc.team4992.robot.OI;
import org.usfirst.frc.team4992.robot.Robot;
import org.usfirst.frc.team4992.robot.RobotMap;
import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
	
public class Climber extends Command {
	
	CANTalon motorClimber;
	boolean hasHit;
	int motorPrefix;
    public Climber() {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.objClimb);
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	motorClimber = new CANTalon(RobotMap.climberGear);
    	hasHit = false;
    	motorPrefix = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Get the bumper that is being pressed desides if the motor is going to go forwards or backwards
    	if(!OI.leftBumper.get() && OI.rightBumper.get()){
    		motorPrefix = 1;
    	} else if(OI.leftBumper.get() && !OI.rightBumper.get()){
    		motorPrefix = -1;
    	} else {
    		motorPrefix  = 0;
    	}
    	
    	//Moves the motor unless it is drawing a lot of current(aka it is hitting the top
    	if(motorClimber.getOutputCurrent()< RobotMap.currentThreshHold && !hasHit){
    		motorClimber.set(motorPrefix);
    	} else if(motorPrefix == -1) {
    		motorClimber.set(motorPrefix);
    	} else {
    		motorClimber.set(-1);
    		Timer.delay(0.05);
    		hasHit = true;
    	}
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
