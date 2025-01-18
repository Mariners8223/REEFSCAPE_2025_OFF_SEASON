package frc.robot.commands.BallDropping;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallDropping.BallDropping;
import frc.robot.subsystems.BallDropping.BallDroppingConstants;
import frc.robot.subsystems.BallDropping.BallDroppingConstants.AngleMotor;


public class BallDropOff extends Command {
  private final BallDropping ballDrop;

  public BallDropOff(BallDropping ballDrop) {
    this.ballDrop = ballDrop;
    addRequirements(ballDrop);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballDrop.stopDropperMotor();
    ballDrop.reachAngle(AngleMotor.AngleToReset);
  }

/* 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
*/

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(ballDrop.getAngle() - AngleMotor.AngleToReset) <= BallDroppingConstants.AngleTolarance;
  }
}