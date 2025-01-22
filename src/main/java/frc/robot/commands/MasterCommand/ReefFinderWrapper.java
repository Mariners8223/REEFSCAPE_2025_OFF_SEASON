// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MasterCommand;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveTrain.DriveBase;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefFinderWrapper extends Command {
  private Command pathCommand;
  private Pose2d targetPose;

  private final DriveBase driveBase;
  /** Creates a new ReefFinderWrapper. */
  public ReefFinderWrapper(DriveBase driveBase, Pose2d targetPose) {
    this.driveBase = driveBase;
    this.targetPose = targetPose;

    pathCommand = driveBase.findPath(targetPose);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void setTargetPose(Pose2d targetPose) {
    this.targetPose = targetPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathCommand = driveBase.findPath(targetPose);

    pathCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return pathCommand.getRequirements();
  }

  /**
   * Whether the given command should run when the robot is disabled. Override to return true if the
   * command should run when disabled.
   *
   * @return whether the command should run when the robot is disabled
   */
  @Override
  public boolean runsWhenDisabled() {
    return pathCommand.runsWhenDisabled();
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return pathCommand.getInterruptionBehavior();
  }
}
