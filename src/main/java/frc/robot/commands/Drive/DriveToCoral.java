// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import org.opencv.objdetect.Objdetect;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveBase;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToCoral extends Command {
    private PhotonCamera camera = new PhotonCamera("EndEffectorCamera");
    private final DriveBase driveBase;
    public double tolerance = 3.0;
  /** Creates a new DriveToCorla. */
  public DriveToCoral(DriveBase driveBase) {
    this.driveBase = driveBase;
    addRequirements(this.driveBase);
   
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var visionResult = camera.getLatestResult();
    boolean hasVisionTargets = visionResult.hasTargets();

    if(hasVisionTargets){
        PhotonTrackedTarget visionTarget = visionResult.getBestTarget();
        double resultYaw = visionTarget.getYaw();
        System.out.println("cozac");

        if (Math.abs(resultYaw) <= tolerance ){
          driveBase.drive(new ChassisSpeeds(1.5, 0, 0));
          tolerance = 10;
        }
        else if (resultYaw >= 0 ){
          driveBase.drive(new ChassisSpeeds(0, 0, -(resultYaw/20.0)));//האם הסיבוב לכיוון הנכון
        }
        else if (resultYaw <= 0 ){
          driveBase.drive(new ChassisSpeeds(0, 0, -(resultYaw/20.0)));//כנל
        }
    }
    // else {
    //   driveBase.drive(new ChassisSpeeds(0,0,1.5));
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tolerance = 3.0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     return false;  
  }
}
