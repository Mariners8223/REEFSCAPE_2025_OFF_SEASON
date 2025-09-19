// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BallDrop;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.BallDrop.BallDropIO.BallDroppingInputs;

public class BallDrop extends SubsystemBase {
  /** Creates a new BallDrop. */
  BallDropIO io;
  BallDroppingInputsAutoLogged inputs = new BallDroppingInputsAutoLogged();
  
  public BallDrop() {
    io = new BallDropIOReal();
    
  }

  public void setVoltageWheel(double voltage)
  {
    io.setVoltageWheel(voltage);
  }

  public void resetMotorEncoder()
  {
    io.resetMotorEncoder();
  }
  public double getAngle()
  {
    return io.getAngle();
  }
  
  public void setAngle(double angle)
  {
    io.setAngle(angle);
    System.out.println("bro");
  }
  public void stopDropperMotor(){
    setVoltageWheel(0);
  }

  @Override
  public void periodic() {
    io.Update(inputs);
  }
}
