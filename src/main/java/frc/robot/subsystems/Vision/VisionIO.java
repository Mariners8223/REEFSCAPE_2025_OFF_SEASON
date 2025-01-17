package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
     record VisionFrame(
            boolean hasTarget,
            double timeStamp,
            double latency,
            Pose3d robotPose,
            double poseAmbiguity,
            EstimationType estimationType,
            double averageTargetDistance
    ){}

    enum EstimationType{
        SINGLE_TARGET,
        MULTIPLE_TARGETS,
    }

    VisionFrame[] emptyFrame =
            {new VisionFrame(false, -1, -1, new Pose3d(), 1, EstimationType.SINGLE_TARGET, 0)};

    @AutoLog
    class VisionInputs{
        boolean isConnected;
        VisionFrame[] visionFrames = new VisionFrame[0];
    }

    void update(VisionInputsAutoLogged inputs);

}
