// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveBase;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MinorAdjust extends Command {
  public static enum Direcation{
    LEFT,
    RIGHT,
    FORWARD,
    BACKWARDS
  }

  private final DriveBase driveBase;

  private double outputY = 0;
  private double outputX = 0;
  /** Creates a new MinorAdjust. */
  public MinorAdjust(DriveBase driveBase, Direcation direcation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveBase = driveBase;

    switch (direcation) {
      case LEFT:
        outputY = 0.3;
        break;
    
      case RIGHT:
        outputY = -0.3;
        break;

      case FORWARD:
        outputX = 0.3;
        break;

      case BACKWARDS:
        outputX = -0.3;
        break;
    }

    addRequirements(driveBase);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveBase.drive(new ChassisSpeeds(outputX, outputY, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
