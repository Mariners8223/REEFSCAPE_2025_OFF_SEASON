package frc.robot.subsystems.shooterGali;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface ShooterIO 
{
    @AutoLog
    class ShooterInputs{
        Pose3d shooter3DPose;
        boolean beamBreak;
    }
    double getPosition();

    void setVoltage(double voltage);

    void setBackWheelsPower(double power)

    void setFrontWheelsPower(double power)

    void Update(ShooterInputs inputs);
    
    void setAngle(double angle);

    void resetAngleEncoder();

    boolean isGPInside();
}
