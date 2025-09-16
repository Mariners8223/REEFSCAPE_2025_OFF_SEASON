package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.AutoLog;

public interface funnelIO {
    @AutoLog
    class funnelInputs  {
        double funnelAngle;
    }
    void setMotorDutyCycle(double MotorDutyCycle);
    void setMotorPosition(double MotorPosition);
    void stopMotor();
    double readMotorAngle();

    void Update(funnelInputs inputs);
}
