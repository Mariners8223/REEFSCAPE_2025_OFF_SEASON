// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.VisionConstants.CameraConstants;

public class Vision extends SubsystemBase {
  private ArrayList<PhotonCamera> cameras = new ArrayList<>();
  private ArrayList<PhotonPoseEstimator> photonPoseEstimators = new ArrayList<>();
  /** Creates a new Vision. */
  public Vision() {
    for(CameraConstants constants : CameraConstants.values()){
      cameras.add(new PhotonCamera(constants.cameraName));

      photonPoseEstimators
    }
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

  }

  @Override
  public void periodic() {
    for(PhotonCamera camera : cameras){
      List<PhotonPipelineResult> results = camera.getAllUnreadResults();

      for(PhotonPipelineResult result : results){
        if(!result.hasTargets()) continue;

        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        

    // Construct PhotonPoseEstimator
  

      }
    }
    // This method will be called once per scheduler run
  }
   public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}
