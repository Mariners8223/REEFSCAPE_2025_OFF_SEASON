package frc.robot.subsystems.RobotAuto.MasterCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.RobotAuto.RobotAutoConstants;


public class FeederCommand extends Command {
    private final DriveBase driveBase;
    private Command pathCommand;


    public FeederCommand(DriveBase driveBase) {
        this.driveBase = driveBase;
    }

    @Override
    public void initialize() {
        Pose2d robotPose = driveBase.getPose();

        Pose2d topFeeder = RobotAutoConstants.FeederLocations.TOP.robotPose;
        Pose2d bottomFeeder = RobotAutoConstants.FeederLocations.BOTTOM.robotPose;

        boolean topFeederCloser = robotPose.getTranslation().getDistance(topFeeder.getTranslation()) <
                robotPose.getTranslation().getDistance(bottomFeeder.getTranslation());

        pathCommand = topFeederCloser ? driveBase.findPath(topFeeder) : driveBase.findPath(bottomFeeder);

        pathCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return !pathCommand.isScheduled();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) pathCommand.cancel();
    }
}
