package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface EndEffectorIO {
    @AutoLog
    class endEffectorInputs{
        boolean beambrake;
        double rightMotorDutySpeed;
        double leftMotorDutySpeed;
        Pose3d positionEndEffector;

    }
    void setMotorDutyCycleLeft(double MotorSpeed);
    void setMotorDutyCycleRight(double MotorSpeed);
    void update(endEffectorInputs inputs);



} 


