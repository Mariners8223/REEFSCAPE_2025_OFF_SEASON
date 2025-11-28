
package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
public class CollectBeamBreak extends Command {
  private final Shooter shooter;
  public CollectBeamBreak(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
