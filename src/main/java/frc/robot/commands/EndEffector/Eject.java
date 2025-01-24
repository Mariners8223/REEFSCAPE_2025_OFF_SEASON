// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants.MotorPower;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;

public class Eject extends Command {
  private final EndEffector endEffector;
  private double startTime;
  private MotorPower motorPower;

  public Eject(MotorPower motorPower, EndEffector endEffector) {
    this.endEffector = endEffector;
    this.motorPower = motorPower;

    addRequirements(endEffector);
  }

  public void setLevel(MotorPower motorPower){
    this.motorPower = motorPower;
  }

  @Override
  public void initialize() {
    endEffector.setLeftMotorPower(motorPower.leftMotorPower);
    endEffector.setRightMotorPower(motorPower.rightMotorPower);

    startTime = RobotController.getMeasureTime().in(Units.Seconds);
  }

  @Override
  public void end(boolean interrupted) {
    endEffector.stopEndEffectorMotors();
    endEffector.setLoadedValue(false);
  }

  @Override
  public boolean isFinished() {
    return RobotController.getMeasureTime().in(Units.Seconds) - startTime >= motorPower.ejectTime;
  }
}
