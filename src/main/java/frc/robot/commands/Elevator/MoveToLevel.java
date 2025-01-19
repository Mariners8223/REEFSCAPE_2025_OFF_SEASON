// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToLevel extends Command {
  /** Creates a new MoveToSetpoint. */
  private final Elevator elevator;
  private ElevatorLevel desiredLevel;

  public MoveToLevel(Elevator elevator, ElevatorLevel level) {
    this.elevator = elevator;
    this.desiredLevel = level;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  public void changeDesiredlevel(ElevatorLevel level){
    this.desiredLevel = level;
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.moveMotorByPosition(desiredLevel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      elevator.moveMotorByPosition(ElevatorLevel.Bottom);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.getCurrentLevel() == desiredLevel;
  }
}
