package main.java.frc.robot.commands.Climb;

import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelIO;

import com.ctre.phoenix.motorcontrol.IFollower;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.funnel.FunnelConstants;

public class MoveFunnel extends Command {
    public double funnelAngle;
    public final Funnel subsystem;
    public MoveFunnel (Funnel subsystemFunnel){
        subsystem = subsystemFunnel;
    }

    @Override
    public void initialize() {
        if (Timer.getMatchTime() < FunnelConstants.START_ENDGAME) cancel();

        if (subsystem.readMotor() < FunnelConstants.FUNNEL_ANGLE_HIGH) {
            subsystem.setMotorPosition(FunnelConstants.FUNNEL_ANGLE_HIGH);
        }
        if (subsystem.readMotor() == FunnelConstants.FUNNEL_ANGLE_HIGH) {
            subsystem.setMotorPosition(FunnelConstants.FUNNEL_ANGLE_LOW);
        }

        subsystem.setMotorPosition(FunnelConstants.FUNNEL_ANGLE_HIGH);

        
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean isInterrupted){
    }

    @Override
    public boolean isFinished(){
    }
}
