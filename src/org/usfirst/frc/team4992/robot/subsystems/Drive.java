package org.usfirst.frc.team4992.robot.subsystems;

import org.usfirst.frc.team4992.robot.RobotMap;
import org.usfirst.frc.team4992.robot.commands.StayGoodBoy;
import com.ctre.CANTalon;
import org.usfirst.frc.team4992.robot.*;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Drive extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	setDefaultCommand(new StayGoodBoy());
    }

    //Automus controler
    
    public void SetSpeed(float ForwardValue, float TurnValue){
    	Robot.driveRobot.arcadeDrive(ForwardValue, TurnValue);
    }
    
    //Normal Drive code use for operater control(uses stick information)
    public void arcadeDrive (Boolean ReverseDriving){
    	if(ReverseDriving){//Driving backwards
    		Robot.driveRobot.arcadeDrive( OI.stick.getY(), OI.stick.getX() );
    	} else {//Normal Drive
    		Robot.driveRobot.arcadeDrive( -OI.stick.getY(), -OI.stick.getX() );
    	}
    }
    
}

