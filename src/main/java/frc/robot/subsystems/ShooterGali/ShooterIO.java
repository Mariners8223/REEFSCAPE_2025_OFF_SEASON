package frc.robot.subsystems.shooterGali;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface ShooterIO {

    @AutoLog
    class ShooterInputs{
        Pose3d shooter3DPose;
        double angle;
    }
    double getPosition();
    void setVoltage(double voltage);
    void Update(ShooterInputs inputs);
}
