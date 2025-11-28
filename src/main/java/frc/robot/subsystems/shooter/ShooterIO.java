package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface ShooterIO {
    @AutoLog
    class ShooterInputs{
        double frontWheelPower;
        double backWheelPower;
        boolean beamBrakeValue;
        Pose3d pose;
    }

    void spinFrontWheel(double RPM);

    void spinBackWheel(double RPM);

    void Update(ShooterInputs inputs);
}
