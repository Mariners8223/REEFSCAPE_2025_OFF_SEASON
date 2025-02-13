// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants.MotorPower;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MiniEject extends Command {
  private final EndEffector endEffector;
  private final MotorPower motorPower;
  /** Creates a new MiniEject. */
  public MiniEject(EndEffector endEffector, MotorPower motorPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.endEffector = endEffector;
    this.motorPower = motorPower;

    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffector.setRightMotorPower(0.8);
    endEffector.setLeftMotorPower(0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.stopEndEffectorMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
