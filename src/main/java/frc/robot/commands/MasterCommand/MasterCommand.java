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
    private final Supplier<ElevatorConstants.ElevatorLevel> levelSupplier;
    private final Supplier<Constants.ReefLocation> targetReefSupplier;

    private final Command coralCommand;

    private final ReefFinderWrapper pathCommand;
    private final MoveToLevel moveElevatorCommand;
    private final Eject ejectCommand;
    private final HomeToReef homeToReef;

    public MasterCommand(DriveBase driveBase, Elevator elevator, EndEffector endEffector, EventTrigger moveElevatorMarker,
                         Supplier<ElevatorConstants.ElevatorLevel> levelSupplier, Supplier<Constants.ReefLocation> targetReefSupplier) {

        this.targetReefSupplier = targetReefSupplier;
        this.levelSupplier = levelSupplier;

        // pathfinder phase (finding the path to the selected reef)
        pathCommand = new ReefFinderWrapper(driveBase, Constants.ReefLocation.REEF_1); // setting the default target pose

        // adjustment phase (minor adjustment to the reef and elevator raising)
        homeToReef = new HomeToReef(driveBase, Constants.ReefLocation.REEF_1);
        moveElevatorCommand = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom);

        // eject phase (releasing the game piece)
        ejectCommand = new Eject(endEffector, EndEffectorConstants.MotorPower.L1);

        // elevator to home phase (moving the elevator to the home position)
        Command elevatorToHome = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom);

        WaitUntilCommand waitUntilMarker = new WaitUntilCommand(moveElevatorMarker);

        // the main command
        coralCommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                pathCommand,
                                homeToReef
                        ),
                        new SequentialCommandGroup(
                                waitUntilMarker,
                                moveElevatorCommand
                        )
                ),
                ejectCommand,
                elevatorToHome
        );
    }

    @Override
    public void initialize() {
        ReefLocation targetReef = targetReefSupplier.get(); // getting the target reef
        ElevatorConstants.ElevatorLevel level = levelSupplier.get(); // getting the target level

        // setting the target pose for the path command
        pathCommand.setTargetPose(targetReef);
        //setting the target pose for the adjustment phase
        homeToReef.setTargetPose(targetReef);
        // homeToReefEndless.setTargetPose(targetReef);

        moveElevatorCommand.changeDesiredlevel(level);
        ejectCommand.setLevel(getMotorPower(level));

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
