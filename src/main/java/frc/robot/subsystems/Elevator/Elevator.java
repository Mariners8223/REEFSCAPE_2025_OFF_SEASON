// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final ElevatorIO io;
  private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();
  private ElevatorLevel currentLevel;
  private ElevatorLevel desiredLevel;

  public Elevator() {
    io = RobotBase.isReal() ? new ElevatorIOVortex() : new ElevatorIOSim();

    currentLevel = null;

    this.resetMotorEncoder();
  }

  public void resetMotorEncoder(){

    io.resetMotorEncoder();
    currentLevel = ElevatorLevel.Bottom;
  }

  public void moveMotorByPosition(ElevatorLevel desiredLevel){
    if (desiredLevel == null) return;
    
    this.desiredLevel = desiredLevel;
    //Logger.recordOutput("Elevator/Desired Level", desiredLevel.name());
    Logger.recordOutput("Elevator/Desired Height", desiredLevel.getHeight());
    io.moveMotorByPosition(desiredLevel.getHeight());
  }

  public ElevatorLevel getCurrentLevel(){
    return this.currentLevel;
  }

  public double getCurrentHeight(){
    return inputs.elevatorHeight;
  }

  public ElevatorLevel getDesiredLevel(){
    return this.desiredLevel;
  }

  public boolean isAtDesiredLevel(){
    return currentLevel == desiredLevel;
  }

  public void setVoltage(double voltage) { io.setVoltage(voltage); }

  @Override
  public void periodic() {
    io.Update(inputs);
    Logger.processInputs(getName(), inputs);

    currentLevel = ElevatorLevel.findNearestLevel(getCurrentHeight());

    Logger.recordOutput("Elevator/CurrentLevel", currentLevel == null ? "Unknown" : currentLevel.name());
    Logger.recordOutput("Elevator/Current Command", this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "None");
  }
}

