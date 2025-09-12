// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EndEffector.EndEffectorIO.EndEffectorInputs;

public class EndEffector extends SubsystemBase {
  public final EndEffectorIO io;
  public final EndEffectorInputsAutoLogged inputs = EndEffectorInputsAutoLogged();

  /** Creates a new EndEffector. */
  public EndEffector() {
    io = new EndEffectorIOReal(); 
  }

  public void spinMotorOppositeWays(double MotorSpeed){
    io.setMotorDutyCycleLeft(MotorSpeed);
    io.setMotorDutyCycleRight(MotorSpeed * -1);
  }

  public void StopMotors(double MotorStop){
    io.stopMotorLeft(MotorStop);
    io.stopMotorRight(MotorStop);
  }
  public void brakeMotors(Boolean Motorbrake){
    io.brakeMotorLeft(Motorbrake);
    io.brakeMotorRight(Motorbrake);
  }
  
  public boolean IsCoralInRobot(){
    return inputs.BeamBrake;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
