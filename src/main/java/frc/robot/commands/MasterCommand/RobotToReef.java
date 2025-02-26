package frc.robot.commands.MasterCommand;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain.DriveBase;

import java.util.Set;
import java.util.function.Supplier;

public class RobotToReef extends Command {
    private final ReefFinderWrapper pathCommand;
    private final HomeToReef homeToReef;

    private final Supplier<Constants.ReefLocation> targetReefSupplier;

    private final Command sequence;

    public RobotToReef(DriveBase driveBase, Supplier<Constants.ReefLocation> targetReefSupplier){
        this.targetReefSupplier = targetReefSupplier;

        pathCommand = new ReefFinderWrapper(driveBase, Constants.ReefLocation.REEF_1);
        homeToReef = new HomeToReef(driveBase, Constants.ReefLocation.REEF_1);

        sequence = new SequentialCommandGroup(
            pathCommand,
            homeToReef,
            new InstantCommand(() -> driveBase.drive(new ChassisSpeeds()))
        );
    }

    @Override
    public void initialize(){
        Constants.ReefLocation targetReef = targetReefSupplier.get();

        pathCommand.setTargetPose(targetReef);
        homeToReef.setTargetPose(targetReef);

        sequence.initialize();
    }

    @Override
    public void execute(){
        sequence.execute();
    }

    @Override
    public void end(boolean interrupted){
        sequence.end(interrupted);
    }

    @Override
    public boolean isFinished(){
        return sequence.isFinished();
    }

    public Set<Subsystem> getRequirements() {
        return this.sequence.getRequirements();
    }

    public boolean runsWhenDisabled() {
        return this.sequence.runsWhenDisabled();
    }

    public Command.InterruptionBehavior getInterruptionBehavior() {
        return this.sequence.getInterruptionBehavior();
    }

}
