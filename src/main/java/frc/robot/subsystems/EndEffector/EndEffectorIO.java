package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface EndEffectorIO {
    @AutoLog
    class EndEffectorInputs {
        boolean BeamBreak;
        double MotorPowerLeft; 
        double MotorPowerRight; //right is on the elevator side
    }
    void setMotorDutyCycleRight(double RightMotorDutyCycle);  //right is on the elevator side
    void stopMotor();
    void setMotorDutyCycleLeft(double LeftMotorDutyCycle);
    void brakeMotor();
    boolean getBeamBreak();

    void update(EndEffectorInputs inputs);
}
