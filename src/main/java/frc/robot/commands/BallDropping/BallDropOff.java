package frc.robot.commands.BallDropping;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallDropping.BallDropping;
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
    ballDrop.reachAngle(AngleMotor.ANGLE_TO_RESET);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(ballDrop.getAngle() - AngleMotor.ANGLE_TO_RESET) <= AngleMotor.ANGLE_TOLERANCE;
  }
}