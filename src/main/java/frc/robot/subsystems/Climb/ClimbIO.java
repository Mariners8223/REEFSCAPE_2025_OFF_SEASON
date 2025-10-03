
package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    class climbing_motor {
        double climbAngle;
        boolean direction;
        double position;
        
    }
    void setMotorDutyCycle(double MotorDutyCycle);
    void setMotorPosition(double MotorPosition);
    void stopMotor(boolean motorRun);
    double readMotor();

    void Update(funnelInputs inputs);
}
