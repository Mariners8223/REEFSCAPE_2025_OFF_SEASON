
package frc.robot.commands.EndEffector.Funnel;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.EndEffector;


public class YeetFunnel extends Command {

   private final EndEffector endEffector;
   private double startTime;

    public YeetFunnel(EndEffector endEffector) {
        this.endEffector = endEffector;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
      startTime = Timer.getFPGATimestamp();
        endEffector.setFunnelVoltage(-3);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > 0.25;
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.moveFunnel(0);
    }
}
