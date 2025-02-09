// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public interface ElevatorIO 
{
    @AutoLog
    class ElevatorInputs{
        Pose3d elevator3DPose;
        double elevatorHeight;
    }

    void resetMotorEncoder();
    void moveMotorByPosition(double position);

    void setVoltage(double voltage);

    void Update(ElevatorInputs inputs);
}
