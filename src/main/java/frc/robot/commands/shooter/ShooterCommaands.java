package main.java.frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;

import java.rmi.registry.RegistryHandler;
import java.util.Timer;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import main.java.frc.robot.subsystems.shooter.*;
import main.java.frc.robot.subsystems.shooter.ShooterConstants.BackMotor;
import main.java.frc.robot.subsystems.shooter.ShooterConstants.FrontMotor;

public class ShooterCommaands extends Command
{
    private double startTime;

    boolean seen = false;

    @Override
    public void initialize()
    {
        
    }
    
    @Override
    public void execute()
    {
        if (seen = false) 
        {
            if (isGpIn() == true)
            {
                FrontMotor.frontSpin(FRONTPOWER);
                BackMotor.backSpin(BACKPOWER);
                seen = true;
                startTime = 0;
            }
        }
        else
        {
            if (isGpIn() == false)
            {
                startTime++;
                if (startTime <= timeForStop)
                {
                    frontPower(powerStop);
                    backPower(powerStop);
                }
            }
        }

    }

    @Override
    public void end()
    {

    }

    @Override
    public boolean isFinished()
    {
        if(startTime == timeForStop)
        {
            
        }
    }
}
