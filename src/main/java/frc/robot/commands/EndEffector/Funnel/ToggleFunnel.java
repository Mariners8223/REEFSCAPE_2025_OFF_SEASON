package frc.robot.commands.EndEffector.Funnel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;


public class ToggleFunnel extends Command {
    private final EndEffector endEffector;
    private double target = EndEffectorConstants.FunnelMotor.CLIMB_POSITION;

    public ToggleFunnel(EndEffector endEffector) {
        this.endEffector = endEffector;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        if(endEffector.getFunnelPosition() < -0.4)
            target = EndEffectorConstants.FunnelMotor.COLLECT_POSITION;
        else
            target = EndEffectorConstants.FunnelMotor.CLIMB_POSITION;

        endEffector.moveFunnel(target);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(endEffector.getFunnelPosition() - target) < EndEffectorConstants.FunnelMotor.TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
