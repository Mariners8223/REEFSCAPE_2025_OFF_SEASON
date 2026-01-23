// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.funnel;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climb.ClimbIO;
import frc.robot.subsystems.funnel.funnelIO;
import frc.robot.subsystems.funnel.funnelConstants.FunnelConstants;

public class funnel extends SubsystemBase {
  /** Creates a new funnel. */
  public final funnelIO io;
  public funnel() {
  io = new funnelIOReal();
  }
  public void setMotorDutyCycle(double MotorPosition){
    io.setMotorDutyCycle(MotorPosition);
  }
  public void setMotorPosition(double MotorPosition){
    io.setMotorPosition(MotorPosition);
  }
  public void stopMotor(){
    io.stopMotor();
  }
  public void raiseFunnel (double motorAngle){
    io.setMotorPosition(motorAngle);
  }
  public boolean isFunnelUp(){
    if (io.readMotorAngle() > FunnelConstants.FUNNEL_ANGLE_HIGH) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

