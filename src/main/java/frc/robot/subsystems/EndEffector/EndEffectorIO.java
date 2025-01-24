// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorInputs{
        double rightPower;
        double leftPower;
        double funnelPosition;
        boolean beamBreakValue;
    }

    public void setRightMotorPower(double powerToSet);

    public void setLeftMotorPower(double powerToSet);

    public void resetFunnelEncoder();

    public void moveFunnel();

    public void stopFunnel();
    
    public void Update(EndEffectorInputs inputs);  
}
