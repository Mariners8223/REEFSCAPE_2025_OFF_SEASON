// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.util.MarinersController.MarinersSimMotor;

/** Add your docs here. */
public class ClimbIOSim implements ClimbIO{
    MarinersSimMotor motor;

    public ClimbIOSim(){
        motor = new MarinersSimMotor("Elevator Motor", 
            DCMotor.getFalcon500(1), 
            ClimbConstants.GEAR_RATIO,
            ClimbConstants.ROTATIONS_TO_METERS,
            ClimbConstants.MOMENT_OF_INERTIA);
        
        motor.enableSoftLimits(ClimbConstants.SOFT_MINIMUM, ClimbConstants.SOFT_MAXIMUM);
        motor.setMotorInverted(ClimbConstants.IS_INVERTED);
    }

    public void setPower(double power) { motor.setDutyCycle(power); }

    public void resetPosition() { motor.setMotorEncoderPosition(ClimbConstants.START_POSITION); }
    public double getPosition() { return motor.getPosition(); } 

    public void Update(ClimbInputs inputs){
        inputs.height = getPosition();
        inputs.pose = new Pose3d(0, 0, 0.2 + inputs.height, new Rotation3d());
    }

    @Override
    public void setBrakeMode(boolean isBrake) {
        System.out.println("orrr norrr");
    }
}
