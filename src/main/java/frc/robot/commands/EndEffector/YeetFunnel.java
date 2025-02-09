
package frc.robot.commands.EndEffector;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
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
      startTime = RobotController.getMeasureTime().in(Units.Seconds);
        endEffector.setFunnelVoltage(-3);
    }

    @Override
    public boolean isFinished() {
        return RobotController.getMeasureTime().in(Units.Seconds) - startTime > 0.25;
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.moveFunnel(0);
    }
}
