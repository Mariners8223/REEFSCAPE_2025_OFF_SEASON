package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface EndEffectorIO {
    @AutoLog
    class EndEffectorInputs {
        boolean BeamBrake;
        Pose3d pose;
        double MotorSpeedLeft;
        double MotorDirectionLeft; 
        double MotorSpeedRight; //right is on the elevator side
        double MotorDirectionRight; //right is on the elevator side
    }
    void setMotorDutyCycleRight(double RightMotorDutyCycle);
    void stopMotorRight(double RightMotorStop); //right is on the elevator side
    void brakeMotorRight(Boolean RightMotorbrake); //right is on the elevator side
    void setMotorDutyCycleLeft(double LeftMotorDutyCycle);
    void stopMotorLeft(double LeftMotorStop);
    void brakeMotorLeft(Boolean LeftMotorbrake);
    boolean getBeamBreak();
}
