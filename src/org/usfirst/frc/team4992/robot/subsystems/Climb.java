package org.usfirst.frc.team4992.robot.subsystems;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climb extends Subsystem {
	CANTalon motorClimber;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
    	motorClimber = new CANTalon(45);
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void up(){
    	motorClimber.set(1);
    }
    
    public void down(){
    	motorClimber.set(-1);
    }
}

