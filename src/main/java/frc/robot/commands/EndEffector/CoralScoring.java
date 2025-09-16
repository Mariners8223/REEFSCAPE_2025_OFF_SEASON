// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralScoring extends Command {
   private final EndEffector endEffector;  /** Creates a new CoralScoring. */
  public CoralScoring(EndEffector endEffector){
    this.endEffector=endEffector;
    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffector.spinMotorOppositeWaysL1(EndEffectorConstants.spinningSpeed); // add a constent folder
    EndEffectorConstants.spinningTime=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    EndEffectorConstants.spinningTime = EndEffectorConstants.spinningTime+1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (EndEffectorConstants.spinningTime >= 50){
      return false;
    }
    else return true;
    
  }
}
