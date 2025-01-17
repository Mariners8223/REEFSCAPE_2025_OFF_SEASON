// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.VisionConstants.CameraConstants;


public class Vision extends SubsystemBase {

    private final AprilTagFieldLayout fieldLayout;

    private final VisionIO[] cameras;
    private final String[] cameraNames;
    private final VisionInputsAutoLogged[] inputs;

    private final VisionConsumer poseConsumer;

    /**
     * Creates a new Vision.
     */
    public Vision(VisionConsumer poseConsumer) {
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        int numOfCameras = CameraConstants.values().length;

        cameras = new VisionIO[numOfCameras];
        cameraNames = new String[numOfCameras];
        inputs = new VisionInputsAutoLogged[numOfCameras];

        CameraConstants[] constants = CameraConstants.values();

        for (int i = 0; i < numOfCameras; i++) {
            cameras[i] = new VisionIOPhoton(constants[i], fieldLayout);
            cameraNames[i] = constants[i].cameraName;
            inputs[i] = new VisionInputsAutoLogged();
        }

        this.poseConsumer = poseConsumer;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < cameras.length; i++) {
            cameras[i].update(inputs[i]);

            Logger.processInputs(cameraNames[i], inputs[i]);

            ArrayList<Pose3d> acceptedPoses = new ArrayList<>();
            ArrayList<Pose3d> rejectedPoses = new ArrayList<>();

            for (VisionIO.VisionFrame frame : inputs[i].visionFrames) {
                if (!frame.hasTarget()) continue;

                if (!checkPoseLocation(frame.robotPose())) {
                    rejectedPoses.add(frame.robotPose());
                    continue;
                }

                if (!checkPoseAmbiguity(frame.poseAmbiguity(), frame.estimationType())) {
                    rejectedPoses.add(frame.robotPose());
                    continue;
                }

                var stdDevs = getStdDevs(frame.averageTargetDistance(), frame.estimationType());

                poseConsumer.accept(frame.robotPose().toPose2d(), frame.timeStamp(), stdDevs);

                acceptedPoses.add(frame.robotPose());
            }

            Logger.recordOutput("Vision/" + cameraNames[i] + "/Accepted Poses", acceptedPoses.toArray(new Pose3d[0]));

            Logger.recordOutput("Vision/" + cameraNames[i] + "/Rejected Poses", rejectedPoses.toArray(new Pose3d[0]));
        }

    }

    private boolean checkPoseLocation(Pose3d pose) {
        //TODO check if the pose is in a valid location
        return true;
    }

    private boolean checkPoseAmbiguity(double poseAmbiguity, VisionIO.EstimationType estimationType) {
        //TODO check if the pose ambiguity is within acceptable bounds
        return true;
    }

    private Matrix<N3, N1> getStdDevs(double averageTagDistance, VisionIO.EstimationType estimationType) {
        //TODO get the standard deviations for the pose
        return VecBuilder.fill(0.1, 0.1, 0.1);
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d pose, double timeStamp, Matrix<N3, N1> stdDevs);
    }
}
