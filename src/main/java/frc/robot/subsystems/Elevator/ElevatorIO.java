package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose3d;
import frc.util.MarinersController.MarinersSparkBase;

public interface ElevatorIO {
    
    @AutoLog
    class ElevatorInputs{
        Pose3d elevator3DPose;
        double elevatorHeight;
    }

    public double GetToDesiredHeight(double Height);
    public void SetElevatorSpeed(double speed);
    public void PauseElevatorMotor(boolean pause);
    


}
