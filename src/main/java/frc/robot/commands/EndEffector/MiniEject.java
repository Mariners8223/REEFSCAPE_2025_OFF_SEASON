// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MasterCommand.MasterCommand;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;

import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MiniEject extends Command {
  private final EndEffector endEffector;
  private final Supplier<ElevatorConstants.ElevatorLevel> levelSupplier;
  private ElevatorConstants.ElevatorLevel currentLevel;
  /** Creates a new MiniEject. */
  public MiniEject(EndEffector endEffector, Supplier<ElevatorConstants.ElevatorLevel> levelSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.endEffector = endEffector;
    this.levelSupplier = levelSupplier;

    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double rightValue = 0.8;
    double leftValue = 0.8;

    currentLevel = levelSupplier.get();

    if(currentLevel != null && currentLevel != ElevatorLevel.Bottom){
      EndEffectorConstants.MotorPower motorPower = MasterCommand.getMotorPower(currentLevel);

      rightValue = motorPower.rightMotorPower;
      leftValue = motorPower.leftMotorPower;
    }

    endEffector.setRightMotorPower(rightValue);
    endEffector.setLeftMotorPower(leftValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.stopEndEffectorMotors();
    if(currentLevel != null) endEffector.setLoadedValue(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
