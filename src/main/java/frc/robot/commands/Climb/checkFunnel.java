package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.funnel.funnel;

public class checkFunnel extends Command {

    public class MoveFunnel extends Command {
    public double funnelAngle;
    public final funnel subsystem;
    
    @Override
    public void initialize() {
        subsystem.readMotorAngle(null)
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(){
    }

    @Override
    public boolean isFinished(){
    }
}
