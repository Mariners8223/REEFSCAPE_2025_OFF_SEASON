// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
  public final EndEffectorIO io;
  /** Creates a new EndEffector. */
  public EndEffector() {
    io = new EndEffectorIOReal();
  }

  public void spinMotorOppositeWays(double MotorSpeed){
    io.setMotorDutyCycleLeft(MotorSpeed);
    io.setMotorDutyCycleRight(MotorSpeed * -1);
  }
  public void StopMotors(){
    io.stopMotorLeft();
    io.stopMotorRight();
  }
  public void SetMotorPossition(Double MotorPossition){
    io.setMotorPositionLeft(MotorPossition);
    io.setMotorPositionRight(MotorPossition * -1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
