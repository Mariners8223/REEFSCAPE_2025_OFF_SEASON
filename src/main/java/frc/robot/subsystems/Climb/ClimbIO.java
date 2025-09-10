
package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    class climbing_motor {
        double speed;
        double rotations;
        boolean direction;
        double position;
        double checkFunnelPosition;
    }
    void setMotorDutyCycle(double MotorDutyCycle);
    void setMotorPosition(double MotorPosition);
    void stopMotor(boolean motorRun);
    void readMotor(double checkFunnelPosition);
}
