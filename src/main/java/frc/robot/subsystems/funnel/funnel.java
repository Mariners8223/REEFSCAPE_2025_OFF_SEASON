// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climb.ClimbIO;
import frc.robot.subsystems.funnel.funnelIO.raiseFunnel;

public class funnel extends SubsystemBase {
  /** Creates a new funnel. */
  public final ClimbIO io;
  public funnel() {
  io = new IOReal();
  }
  public void setMotorDutyCycle(double MotorPosition){
    io.setMotorDutyCycle(MotorPosition);
  }
  public void setMotorPosition(double MotorPosition){
    io.setMotorPosition(MotorPosition);
  }
  public void stopMotor(boolean motorRun){
    io.stopMotor(motorRun);
  }
  public void raiseFunnel (double motorAngle){
    io.setMotorPosition(motorAngle);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

