// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BallDropping;

import org.littletonrobotics.junction.AutoLog;

public interface BallDroppingIO {
    @AutoLog
    public static class balldroppingInputs{
        double dropperPower;
        double angle;
    }

    //angle motor io
    public void resetAngleEncoder();
    public void reachAngle(double angleToReach);

    //dropping motor io
    public void setDropperMotorPower(double dropperPower);
    public void stopDropperMotor();

    public void Update(balldroppingInputs inputs);
}
