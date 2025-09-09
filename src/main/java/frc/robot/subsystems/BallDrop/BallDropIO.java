package frc.robot.subsystems.BallDrop;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface BallDropIO {
    @AutoLog
    class BallDroppingInputs{
        Pose3d ballDropping3DPose;
        double armAnglePose;
    }
//wheel movment
    void SetVoltageWheel(double voltage);
    
//arm movment
    void resetMotorEncoder();

    void SetVoltageArm(double voltage);

    void SetAngle(double angle);
    
//
    void Update(BallDroppingInputs inputs);


}
