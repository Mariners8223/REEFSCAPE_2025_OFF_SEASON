package frc.robot.commands.BallDrop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallDrop.BallDrop;
import frc.robot.subsystems.BallDrop.BallDropConstants;

public class BallDropOff extends Command {
    BallDrop ballDrop;
public BallDropOff(BallDrop ballDrop) {
    this.ballDrop = ballDrop;
    addRequirements(ballDrop);
  }
  @Override
  public void initialize() {
    ballDrop.stopDropperMotor();
    ballDrop.setAngle(BallDropConstants.ArmMotor.ANGLE_TO_RESET);
    System.out.println("why we here");
  }

  @Override
  public void execute(){
   
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return Math.abs(ballDrop.getAngle() - BallDropConstants.ArmMotor.ANGLE_TO_RESET) <= BallDropConstants.ArmMotor.ANGLE_TOLERANCE;
  }
}
