// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BallDrop;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.BallDrop.BallDropIO.BallDroppingInputs;

public class BallDrop extends SubsystemBase {
  /** Creates a new BallDrop. */
  private final BallDropIO io;
  public BallDrop() {
    io = new BallDropIOReal();
    
  }

  public void SetVoltageWheel(double voltage)
  {
    io.SetVoltageWheel(voltage);
  }

  public void resetMotorEncoder()
  {
    io.ResetMotorEncoder();
  }
  public double GetAngle()
  {
    return io.GetAngle();
  }
  
  public void SetAngle(double angle)
  {
    io.SetAngle(angle);
  }

  public void Update(BallDroppingInputs inputs)
  {
    io.Update(inputs);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
