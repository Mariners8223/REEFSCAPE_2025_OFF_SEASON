package frc.robot.commands.MasterCommand;


import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.ReefLocation;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.EndEffector.Eject;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;

import java.util.Set;
import java.util.function.Supplier;


public class MasterCommand extends Command {
    private final DriveBase driveBase;
    private final Elevator elevator;
    private final EndEffector endEffector;

    private final Supplier<ElevatorConstants.ElevatorLevel> levelSupplier;
    private final Supplier<Constants.ReefLocation> targetReefSupplier;

    private Command coralCommand;
    private final EventTrigger waitUntilMarker;

    public MasterCommand(DriveBase driveBase, Elevator elevator, EndEffector endEffector,
                         Supplier<ElevatorConstants.ElevatorLevel> levelSupplier, Supplier<Constants.ReefLocation> targetReefSupplier) {

        this.targetReefSupplier = targetReefSupplier;
        this.levelSupplier = levelSupplier;

        this.driveBase = driveBase;
        this.elevator = elevator;
        this.endEffector = endEffector;

        waitUntilMarker = new EventTrigger("move to selected level");

        // the main command
        coralCommand = new InstantCommand();
    }

    @Override
    public void initialize() {
        ReefLocation targetReef = targetReefSupplier.get(); // getting the target reef
        ElevatorConstants.ElevatorLevel level = levelSupplier.get(); // getting the target level

        coralCommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        driveBase.pathFindToPathAndFollow(targetReef.getPath()),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(waitUntilMarker),
                                new MoveToLevel(elevator, level)
                        )
                ),
                new HomeToReef(driveBase, targetReef),
                new Eject(endEffector, getMotorPower(level)),
                new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom)
        );

        coralCommand.initialize();
    }

    public static EndEffectorConstants.MotorPower getMotorPower(ElevatorConstants.ElevatorLevel level) {
        return switch (level) {
            case L2, L3 -> EndEffectorConstants.MotorPower.L2_3;
            case L4 -> EndEffectorConstants.MotorPower.L4;
            default -> EndEffectorConstants.MotorPower.L1;
        };
    }


    @Override
    public void execute() {
        coralCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        coralCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return coralCommand.isFinished();
    }

    public Set<Subsystem> getRequirements() {
        return this.coralCommand.getRequirements();
    }

    public boolean runsWhenDisabled() {
        return this.coralCommand.runsWhenDisabled();
    }

    public Command.InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
