// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BallDrop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallDrop.BallDrop;
import frc.robot.subsystems.BallDrop.BallDropConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BallDropHigh extends Command {
  private final BallDrop ballDrop;

  public BallDropHigh(BallDrop ballDrop) {
    this.ballDrop = ballDrop;
    addRequirements(ballDrop);
  }

  @Override
  public void initialize() {
    ballDrop.setVoltageWheel(BallDropConstants.DropperMotor.POWER_TO_REACH);
    ballDrop.setAngle(BallDropConstants.ArmMotor.ANGLE_TO_REACH_TOP);
    System.out.println("did init");
  }
  

  @Override
  public boolean isFinished() {
    System.out.println("did is finished");
    return Math.abs(BallDropConstants.ArmMotor.ANGLE_TO_REACH_TOP - ballDrop.getAngle()) <= BallDropConstants.ArmMotor.ANGLE_TOLERANCE;
  }
}
