// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralScoring extends Command {
   private final EndEffector endEffector;
   private final EndEffector EndEffectorConstants;  /** Creates a new CoralScoring. */
  public CoralScoring(EndEffector endEffector, EndEffector endEffectorConstants) {
    this.endEffector=endEffector;
    this.endEffector=endEffector;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffector.spinMotorOppositeWays(0); // add a constent folder
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double spinningTime=spinningTime+1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (spinningTime <= 50)
    return false;
  }
}
