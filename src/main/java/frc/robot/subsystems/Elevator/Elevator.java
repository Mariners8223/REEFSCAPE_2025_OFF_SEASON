// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;

public class Elevator extends SubsystemBase {
  private static final double TOLERANCE = 0.5;
    
      ElevatorIO io;
    
      /** Creates a new Elevator. */
      public Elevator() {
        io = new ElevatorIOReal();
      }
    
      public void setElevatorLevel(ElevatorLevel level){
        io.SetToDesiredHeight(level.getHeight());
      }
    
      public ElevatorLevel getCurrentLevel(){
        
        double height = io.getHeight();
    
        if (height == ElevatorLevel.L1.getHeight()) return ElevatorLevel.L1;
        if (height == ElevatorLevel.L2.getHeight()) return ElevatorLevel.L2;
        if (height == ElevatorLevel.L3.getHeight()) return ElevatorLevel.L3;
        if (height == ElevatorLevel.L4.getHeight()) return ElevatorLevel.L4;
            return null;
    

      ElevatorConstants.TOLERANCE > Math.abs(height - ElevatorLevel.L1.getHeight());
      ElevatorConstants.TOLERANCE > Math.abs(height - ElevatorLevel.L2.getHeight());
      ElevatorConstants.TOLERANCE > Math.abs(height - ElevatorLevel.L3.getHeight());
      ElevatorConstants.TOLERANCE > Math.abs(height - ElevatorLevel.L4.getHeight());
      
   }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
