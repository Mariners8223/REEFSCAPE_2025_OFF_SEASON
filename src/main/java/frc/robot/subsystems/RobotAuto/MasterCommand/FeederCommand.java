package frc.robot.subsystems.RobotAuto.MasterCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.RobotAuto.RobotAutoConstants;

import java.util.function.Supplier;


public class FeederCommand extends Command {
    private final DriveBase driveBase;
    private final Supplier<Pose2d> targetPoseSupplier;

    private Command pathCommand;


    public FeederCommand(DriveBase driveBase, Supplier<Pose2d> targetPoseSupplier) {
        this.driveBase = driveBase;
        this.targetPoseSupplier = targetPoseSupplier;
    }

    @Override
    public void initialize() {
        pathCommand = driveBase.findPath(targetPoseSupplier.get());

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
