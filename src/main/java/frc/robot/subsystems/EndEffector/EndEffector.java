// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj.SensorUtil;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EndEffector.EndEffectorIO.EndEffectorInputs;

public class EndEffector extends SubsystemBase {
  public final EndEffectorIO io;
  public final EndEffectorInputsAutoLogged inputs = new EndEffectorInputsAutoLogged();

  /** Creates a new EndEffector. */
  public EndEffector() {
    io = new EndEffectorIOReal(); 
  }

  public void spinMotorOppositeWays(double MotorSpeed){
    io.setMotorDutyCycleLeft(MotorSpeed);
    io.setMotorDutyCycleRight(MotorSpeed * -1);
  }
  public void spinMotorOppositeWaysL1(double MotorSpeed){
    io.setMotorDutyCycleLeft(MotorSpeed);
    io.setMotorDutyCycleRight(MotorSpeed * -0.4);
  }
  public void StopMotors(){
    io.stopMotor();
  }
  public void brakeMotors(){
    io.brakeMotor();
  }
  
  public boolean IsCoralInRobot(){
    return inputs.BeamBreak;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.update(inputs);
  }
}
