// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorInputs{
        double RightPower;
        double LeftPower;
        boolean beamBreakValue;
    }

    public void setRightMotorPower(double PowerToSet);

    public void setLeftMotorPower(double PowerToSet);
    
    public void Update(EndEffectorInputs inputs);  
}
