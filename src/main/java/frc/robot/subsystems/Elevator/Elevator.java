// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.Elevator.ElevatorIO.ElevatorInputs;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  ElevatorIO io;
  ElevatorInputs inputs = new ElevatorInputs(); /// TODO: Check about AutoLogged
  ElevatorLevel level;

  public Elevator() {
    // io = new IO();
    level = ElevatorLevel.NULL;
  }

  public void resetMotorEncoder(){
    io.resetMotorEncoder();
    level = ElevatorLevel.Bottom;
  }

  public void moveMotorByPosition(double position){
    io.moveMotorByPosition(position);
  }

  public void setLevelVariable(ElevatorLevel level){
    this.level = level;
  }
  public ElevatorLevel getLevel(){
    return this.level;
  }

  public double getCurrentPosition(){
    return io.getCurrentPosition();
  }

  public double getCurrentHeight(){
    return getCurrentPosition() * ElevatorConstants.GEAR_RATIO; // TODO: Make actual formula
  }

  @Override
  public void periodic() {
    io.Update(inputs);
  }
}
