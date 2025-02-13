package frc.robot.commands.MasterCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveTrain.DriveBase;

import java.util.Set;
import java.util.function.Supplier;

public class PathPlannerWrapper extends Command{
    private Command pathCommand = new InstantCommand();

    private final DriveBase driveBase;
    private final Supplier<Pose2d> targetPoseSupplier;

    public PathPlannerWrapper(DriveBase driveBase, Supplier<Pose2d> targetPoseSupplier) {
        this.driveBase = driveBase;
        this.targetPoseSupplier = targetPoseSupplier;
    }

    @Override
    public void initialize() {
        pathCommand = driveBase.findPath(targetPoseSupplier.get(), 1.5);
        pathCommand.initialize();
    }

    @Override
    public void execute() {
        pathCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        pathCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return pathCommand.getRequirements();
    }

    @Override
    public boolean runsWhenDisabled() {
        return pathCommand.runsWhenDisabled();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return pathCommand.getInterruptionBehavior();
    }
}
