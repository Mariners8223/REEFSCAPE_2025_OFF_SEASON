// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
  public final EndEffectorIO io;
  public boolean IsBeamBroke = false;
  /** Creates a new EndEffector. */
  public EndEffector() {
    io = new EndEffectorIOReal();
  }

  public void spinMotorOppositeWays(double MotorSpeed){
    io.setMotorDutyCycleLeft(MotorSpeed);
    io.setMotorDutyCycleRight(MotorSpeed * -1);
    IsBeamBroke = false;

  }
  public void StopMotors(double MotorStop){
    io.stopMotorLeft(MotorStop);
    io.stopMotorRight(MotorStop);
  }
  public void SetMotorPossition(Double MotorPossition){
    io.setMotorPositionLeft(MotorPossition);
    io.setMotorPositionRight(MotorPossition * -1);
  }
  
  public boolean IsCoralInRobot(boolean IsBeamBroke){
    if (!IsBeamBroke){
      IsBeamBroke = io.getBeamBreak();

    }

    return IsBeamBroke;
     

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
