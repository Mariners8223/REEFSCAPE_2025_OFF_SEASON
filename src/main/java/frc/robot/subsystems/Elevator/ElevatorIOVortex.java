// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;
import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersController.ControlMode;

/** Add your docs here. */
public class ElevatorIOVortex implements ElevatorIO {
    private final MarinersSparkBase motorLead;
    @SuppressWarnings("unused")
    private final MarinersSparkBase motorFollow;

    public ElevatorIOVortex(){
        this.motorLead = configureLeadMotor();
        this.motorFollow = configureFollowMotor();

        SmartDashboard.putNumber("Elevator P", ElevatorConstants.LeadMotor.PID_GAINS.getP());
        SmartDashboard.putNumber("Elevator I", ElevatorConstants.LeadMotor.PID_GAINS.getI());
        SmartDashboard.putNumber("Elevator D", ElevatorConstants.LeadMotor.PID_GAINS.getD());
    }

    private MarinersSparkBase configureLeadMotor(){
        MarinersSparkBase motor;
        motor = new MarinersSparkBase("Lead Elevator Motor", ElevatorConstants.LeadMotor.CONTROLLER_LOCATION, 
            ElevatorConstants.LeadMotor.MOTOR_ID, ElevatorConstants.LeadMotor.IS_BRUSHLESS, 
            ElevatorConstants.LeadMotor.MOTOR_TYPE, ElevatorConstants.LeadMotor.PID_GAINS,
            ElevatorConstants.LeadMotor.GEAR_RATIO / ElevatorConstants.HEIGHT_TO_ROTATION);

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
        motorLead.setMotorEncoderPosition(ElevatorLevel.Bottom.getHeight());
    }

    public void moveMotorByPosition(double position){
        motorLead.setReference(position, ControlMode.Position, ElevatorConstants.FEED_FORWARD);
    }

    public void setVoltage(double voltage){
        motorLead.setVoltage(voltage);
    }

    public double getVoltage() {return 0;} // TODO: Return correct voltage
    public double getVelocity() { return motorLead.getVelocity(); }

    public void Update(ElevatorInputs inputs){
        inputs.elevatorHeight = motorLead.getPosition();
        inputs.elevator3DPose = new Pose3d(ElevatorConstants.X_ON_ROBOT, ElevatorConstants.Y_ON_ROBOT, inputs.elevatorHeight, new Rotation3d());
        
        double P = SmartDashboard.getNumber("Elevator P", motorLead.getPIDF().getP());
        double I = SmartDashboard.getNumber("Elevator I", motorLead.getPIDF().getI());
        double D = SmartDashboard.getNumber("Elevator D", motorLead.getPIDF().getD());

        motorLead.setPIDF(new PIDFGains(P, I, D));
    }
}
