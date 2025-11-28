
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose3d;


public interface shooterIO
{
    @AutoLog
    class shooterInputes
    {
        double frontPower;
        double backPower;
        boolean beamBreakValue;
        Pose3d location;
    }

    void frontSpin(double power);
    void backSpin(double power);
    boolean isGpIn();

    void update(shooterInputes inputes);
}

