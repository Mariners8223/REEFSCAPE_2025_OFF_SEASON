// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class EndEffectorIOSim implements EndEffectorIO{
    private double rightMotorPower;
    private double leftMotorPower;
    private double funnelPosition;

    public EndEffectorIOSim(){
        SmartDashboard.putBoolean("beam break value", false);
    }

    @Override
    public void setRightMotorPower(double powerToSet) {
        this.rightMotorPower = powerToSet;
    }

    @Override
    public void setLeftMotorPower(double powerToSet) {
        this.leftMotorPower = powerToSet;
    }

    @Override
    public void moveFunnel(){
        return; 
    }

    @Override
    public void stopFunnel(){
        return;
    }

    @Override
    public void Update(EndEffectorInputs inputs) {
        inputs.leftPower = leftMotorPower;
        inputs.rightPower = rightMotorPower;
        inputs.funnelPosition = funnelPosition;
        inputs.beamBreakValue = SmartDashboard.getBoolean("beam break value", false);
    }

}
