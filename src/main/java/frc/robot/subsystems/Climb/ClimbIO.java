package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    class climbing_motor {
        double speed;
        double rotations;
        boolean direction;
    }
    void setMotorDutyCycle(double MotorDutyCycle);
    void setMotorPosition(double MotorPosition);
    void stopMotor(boolean motorRun);
}
