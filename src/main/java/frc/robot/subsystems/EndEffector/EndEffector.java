// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
    private final EndEffectorIO io;
    private final EndEffectorInputsAutoLogged inputs = new EndEffectorInputsAutoLogged();

    private boolean gpLoaded = true;

    public EndEffector() {
        io = new EndEffectorIOReal();
    }

    public void setRightMotorPower(double PowerToSet) {
        io.setRightMotorPower(PowerToSet);
    }

    public void setLeftMotorPower(double PowerToSet) {
        io.setLeftMotorPower(PowerToSet);
    }

    public boolean gpLoaded() {
        return gpLoaded;
    }

    public void setGpLoaded(boolean gpLoaded) {
        Logger.recordOutput("EndEffector/game piece loaded", gpLoaded);
        this.gpLoaded = gpLoaded;
    }

    public void stopMotors() {
        io.setLeftMotorPower(0);
        io.setRightMotorPower(0);
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
