// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climb.ClimbIO.ClimbInputs;

public class Climb extends SubsystemBase {
	private final ClimbIO io;
	private final ClimbInputs inputs = new ClimbInputs();

	/** Creates a new Climb. */
	public Climb() {
		io = new ClimbIOFalcon();
		io.resetPosition();
	}

	public void setMotorPower(double power) { io.setPower(power); }
	public double getMotorPosition() { return io.getPosition(); }
	public boolean isAtLimit() { return (io.getPosition() <= ClimbConstants.SOFT_MINIMUM) || (io.getPosition() >= ClimbConstants.SOFT_MAXIMUM); }

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		io.Update(inputs);
	}
}
