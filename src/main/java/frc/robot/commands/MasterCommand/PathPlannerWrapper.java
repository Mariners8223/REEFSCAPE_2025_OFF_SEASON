package frc.robot.commands.MasterCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.FeederLocation;
import frc.robot.subsystems.DriveTrain.DriveBase;

import java.util.Set;

public class PathPlannerWrapper extends Command{
    private Command pathCommand = new InstantCommand();

    private final DriveBase driveBase;
    private final FeederLocation targetFeeder;

    public PathPlannerWrapper(DriveBase driveBase, FeederLocation targetFeeder) {
        this.driveBase = driveBase;
        this.targetFeeder = targetFeeder;
    }

    @Override
    public void initialize() {
        pathCommand = driveBase.pathFindToPathAndFollow(targetFeeder.getPath());
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
