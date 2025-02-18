// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public interface ClimbIO {
    @AutoLog
    class ClimbInputs{
        double height;
        Pose3d pose;
    }

    void setPower(double power);

    void resetPosition();
    double getPosition();

    void setBrakeMode(boolean isBrake);

    void Update(ClimbInputs inputs);
}
