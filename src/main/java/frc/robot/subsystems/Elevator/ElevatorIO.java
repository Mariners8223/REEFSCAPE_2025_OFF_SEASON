package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose3d;
import frc.util.MarinersController.MarinersSparkBase;

public interface ElevatorIO {
    
    @AutoLog
    class ElevatorInputs{
        double elevatorHeight;
        public MarinersSparkBase height;
    }

    public void SetToDesiredHeight(double Height);
    public void PauseElevatorMotor(boolean pause);
    public double getHeight();
    


}
