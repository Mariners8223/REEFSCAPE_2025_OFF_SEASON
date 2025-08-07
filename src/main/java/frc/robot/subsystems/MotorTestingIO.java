package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface MotorTestingIO {
    @AutoLog
    public class MotorTestInputs {
        double motorCurrent;
    }
    public void setTestMotorDutyCyle (double dutyCyle);
    public void stopTestMotor ();

//    public void update(MotorTestInputsAutoLogged inputs);

}
