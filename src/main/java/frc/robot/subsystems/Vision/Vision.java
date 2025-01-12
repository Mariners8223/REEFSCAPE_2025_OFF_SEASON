// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.logging.Logger;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.VisionConstants.CameraConstants;

public class Vision extends SubsystemBase {
  private final ArrayList<PhotonCamera> cameras = new ArrayList<>();
  private final ArrayList<PhotonPoseEstimator> poseEstimators = new ArrayList<>();

  private final Consumer<Pair<Pose2d, Double>> poseConsumer;
  /** Creates a new Vision. */
  public Vision(Consumer<Pair<Pose2d, Double>> poseConsumer) {
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

    for(CameraConstants constants : CameraConstants.values()){
      cameras.add(new PhotonCamera(constants.cameraName));

      PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
        aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        constants.robotToCamera);

      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      poseEstimators.add(poseEstimator);  
    }

    this.poseConsumer = poseConsumer;
  }

  @Override
  public void periodic() {
    for(int i = 0; i < cameras.size(); i++){
      PhotonCamera camera = cameras.get(i);
      PhotonPoseEstimator poseEstimator = poseEstimators.get(i);

      List<PhotonPipelineResult> results = camera.getAllUnreadResults();

      Optional<EstimatedRobotPose> lastPose = Optional.empty();

      for(PhotonPipelineResult result : results){
        if(!result.hasTargets()) continue;

        Optional<EstimatedRobotPose> optionalPose = poseEstimator.update(result);

        if(optionalPose.isEmpty()) continue;

        EstimatedRobotPose pose = optionalPose.get();

        lastPose = Optional.of(pose);

        poseConsumer.accept(new Pair<Pose2d,Double>(pose.estimatedPose.toPose2d(), pose.timestampSeconds));
      }
    }
  }
}
