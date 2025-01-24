// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.EndEffector.EndEffectorIO.EndEffectorInputs;
import frc.robot.Robot;

public class EndEffector extends SubsystemBase {
    private final EndEffectorIO io;
    private final EndEffectorInputsAutoLogged inputs = new EndEffectorInputsAutoLogged();
    private boolean isGpLoaded;

    public EndEffector() {
        if (Robot.isSimulation()) {
            io = new EndEffectorIOSim();
        } else {
            io = new EndEffectorIOReal();
        }
    }

    public void setRightMotorPower(double PowerToSet) {
        io.setRightMotorPower(PowerToSet);
    }

    public void setLeftMotorPower(double PowerToSet) {
        io.setRightMotorPower(PowerToSet);
    }

    public void moveFunnel() {
        io.moveFunnel();
    }

    public void stopEndEffectorMotors() {
        io.setLeftMotorPower(0);
        io.setRightMotorPower(0);
    }

    public void stopFunnelMotor(){
        io.stopFunnel();
    }

    public void setLoadedValue(boolean value) {
        Logger.recordOutput("EndEffector/gp loaded", value);
        isGpLoaded = value;
    }

    public double getRightMotorPower(){
        return inputs.rightPower;
    }

    public double getLeftMotorPower(){
        return inputs.leftPower;
    }

    public double getFunnelPosition(){
        return inputs.funnelPosition;
    }

    public boolean isGpLoaded() {
        return isGpLoaded;
    }

    public boolean isGpDetected() {
        return inputs.beamBreakValue;
    }

    @Override
    public void periodic() {
        io.Update(inputs);
        Logger.processInputs(getName(), inputs);
    }
}
