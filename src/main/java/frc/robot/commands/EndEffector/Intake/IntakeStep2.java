// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.EndEffector;
public class IntakeStep2 extends Command {
  private final EndEffector endEffector;

  public IntakeStep2(EndEffector endEffector) {
    this.endEffector = endEffector;
    addRequirements(endEffector);
  }


  @Override
  public void end(boolean interrupted) {
    endEffector.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return !endEffector.isGpDetected();
  }
}