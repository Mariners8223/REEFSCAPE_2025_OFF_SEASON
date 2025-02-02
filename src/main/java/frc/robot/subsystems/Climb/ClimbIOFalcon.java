// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControllerLocation;

/** Add your docs here. */
public class ClimbIOFalcon implements ClimbIO{
    MarinersTalonFX motor;

    public ClimbIOFalcon(){
        motor = configureMotor();
    }

    public MarinersTalonFX configureMotor(){
        MarinersTalonFX motor = new MarinersTalonFX(
            "Climb Motor",
            ControllerLocation.MOTOR,
            ClimbConstants.MOTOR_ID,
            ClimbConstants.GEAR_RATIO * ClimbConstants.ROTATIONS_TO_METERS);
        
        motor.enableSoftLimits(ClimbConstants.SOFT_MINIMUM, ClimbConstants.SOFT_MAXIMUM);
        motor.setMotorIdleMode(true);
        motor.setMotorInverted(ClimbConstants.IS_INVERTED);
        motor.setMaxMinOutput(0, -0.4 * 12);

        return motor;
    }

    public void setPower(double power) { motor.setDutyCycle(power); }

    public void resetPosition() { motor.setMotorEncoderPosition(ClimbConstants.START_POSITION); }
    public double getPosition() { return motor.getPosition(); }

    public void Update(ClimbInputs inputs){
        inputs.height = getPosition();
    }
}
