
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose3d;


public interface shooterIO
{
    @AutoLog
    class shooterInputs
    {
        double frontPower;
        double backPower;
        double reachedAngle;
        boolean beamBreakValue;
        Pose3d location;
    }

    void frontSpin(double power);
    void backSpin(double power);
    void setAngleTo(double angle);

    void update(shooterInputs inputs);
}

