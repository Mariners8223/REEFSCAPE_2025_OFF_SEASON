// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersController.ControlMode;

/** Add your docs here. */
public class ElevatorIOVortex implements ElevatorIO {
    MarinersSparkBase motorLead;
    MarinersSparkBase motorFollow;

    public ElevatorIOVortex(){
        this.motorLead = configureLeadMotor();
        this.motorFollow = configureFollowMotor();
    }

    private MarinersSparkBase configureLeadMotor(){
        MarinersSparkBase motor;
        motor = new MarinersSparkBase("Lead Elevator Motor", ElevatorConstants.LeadMotor.CONTROLLER_LOCATION, 
            ElevatorConstants.LeadMotor.MOTOR_ID, ElevatorConstants.LeadMotor.IS_BRUSHLESS, 
            ElevatorConstants.LeadMotor.MOTOR_TYPE, ElevatorConstants.LeadMotor.PID_GAINS, ElevatorConstants.LeadMotor.GEAR_RATIO);

        motor.enableSoftLimits(ElevatorConstants.LeadMotor.SOFT_MINIMUM, ElevatorConstants.LeadMotor.SOFT_MAXIMUM);

        motor.setMotorInverted(ElevatorConstants.LeadMotor.IS_INVERTED);
        motor.setMotorIdleMode(true);

        return motor;
    }

    private MarinersSparkBase configureFollowMotor(){
        MarinersSparkBase motor;
        motor = new MarinersSparkBase("Follow Elevator Motor", ElevatorConstants.FollowMotor.CONTROLLER_LOCATION, 
            ElevatorConstants.FollowMotor.MOTOR_ID, ElevatorConstants.FollowMotor.IS_BRUSHLESS, ElevatorConstants.FollowMotor.MOTOR_TYPE);
        
        motor.setMotorAsFollower(this.motorLead, ElevatorConstants.FollowMotor.IS_INVERTED);
        return motor;
    }

    public void resetMotorEncoder(){
        motorLead.setMotorEncoderPosition(0);
    }

    public void moveMotorByPosition(double position){
        motorLead.setReference(position, ControlMode.Position, ElevatorConstants.FEED_FORWARD);
    }

    public double getCurrentPosition(){
        return motorLead.getPosition();
    }

    public void Update(ElevatorInputs inputs){
        inputs.elevatorHeight = getCurrentPosition() * ElevatorConstants.ROTATION_TO_HEIGHT;
        inputs.elevator3DPose = new Pose3d(ElevatorConstants.X_ON_ROBOT, ElevatorConstants.Y_ON_ROBOT, inputs.elevatorHeight, new Rotation3d()); // Check if this is resource intensive
        inputs.currentLevel = ElevatorLevel.findNearestLevel(inputs.elevatorHeight);
    }
}
