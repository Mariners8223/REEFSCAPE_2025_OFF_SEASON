// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;

public class moveFunnel extends Command {
  private final EndEffector endEffector;

  public moveFunnel(EndEffector endEffector){
    this.endEffector = endEffector;
    addRequirements(endEffector);
  }

  @Override
  public void initialize(){
    endEffector.moveFunnel();
  }

  @Override
  public void end(boolean interrupted){
    endEffector.stopFunnelMotor();
  }

  @Override
  public boolean isFinished(){
    return Math.abs(endEffector.getFunnelPosition() - EndEffectorConstants.FunnelMotor.target) < EndEffectorConstants.FunnelMotor.tolerance;
  }
}
