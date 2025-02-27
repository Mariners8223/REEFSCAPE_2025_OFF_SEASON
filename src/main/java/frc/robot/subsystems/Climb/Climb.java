// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

public class Climb extends SubsystemBase {
    private final ClimbIO io;
    private final ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();

    /**
     * Creates a new Climb.
     */
    public Climb() {
        io = Robot.isReal() ? new ClimbIOFalcon() : new ClimbIOSim();
        io.resetPosition();

        io.setBrakeMode(false);

        new Trigger(RobotState::isEnabled).whileTrue(new StartEndCommand(
            () -> io.setBrakeMode(true),
            () -> io.setBrakeMode(false)).ignoringDisable(true));
    }

    public void setMotorPower(double power) {
        io.setPower(power);
    }

    public boolean isAtLimit() {
        return (io.getPosition() <= ClimbConstants.SOFT_MINIMUM);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.Update(inputs);
        Logger.processInputs(getName(), inputs);

        String currentCommandName = getCurrentCommand() == null ? "Null" : getCurrentCommand().toString();
        Logger.recordOutput("Climb/Current Command", currentCommandName);
    }
}
