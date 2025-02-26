// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BallDropping;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface BallDroppingIO {
    @AutoLog
    class BallDroppingInputs {
        double dropperPower;
        double angle;
        Pose3d pose;
    }

    //angle motor io
    void resetAngleEncoder();
    void reachAngle(double angleToReach);

    //dropping motor io
    void setDropperMotorPower(double dropperPower);
    void setVoltage(double voltage);

    void Update(BallDroppingInputs inputs);
}
