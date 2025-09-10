package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.AutoLog;

public interface funnelIO {
    @AutoLog
    class raiseFunnel  {
        double speed;
        double rotations;
        boolean direction;
        double motorAngle;
    }
    void setMotorDutyCycle(double MotorDutyCycle);
    void setMotorPosition(double MotorPosition);
    void stopMotor(boolean motorRun);
}
