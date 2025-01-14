// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;

/** Add your docs here. */
public interface ElevatorIO 
{
    @AutoLog
    public static class ElevatorInputs{
        Pose3d elevator3DPose;
        double elevatorHeight;
        ElevatorLevel currentLevel;
    }

    public void resetMotorEncoder();
    public void moveMotorByPosition(double position);

    public double getCurrentPosition();

    public void Update(ElevatorInputs inputs);
}
