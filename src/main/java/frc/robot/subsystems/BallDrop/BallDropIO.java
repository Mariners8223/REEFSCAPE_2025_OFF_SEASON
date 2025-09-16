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
    void setVoltageWheel(double voltage);
    
//arm movment
    void resetMotorEncoder();

    double getAngle();

    void setAngle(double angle);
    
//
    void Update(BallDroppingInputs inputs);


}
