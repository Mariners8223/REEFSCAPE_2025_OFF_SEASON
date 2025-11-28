
package frc.robot.commands.Shooter;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
public class CollectNOBreak extends Command {
  private final Shooter shooter;
  private int timer = 0;
  public CollectNOBreak(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.spinBackMotor(ShooterConstants.RPM);
    shooter.spinFrontMotor(ShooterConstants.RPM);
  }

  @Override
  public void execute() {
    timer++;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotors();
  }

  @Override
  public boolean isFinished() {
    if(timer>=50) return true;
    return false;
  }
}
