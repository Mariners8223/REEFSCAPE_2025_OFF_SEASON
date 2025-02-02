package frc.robot.commands.MasterCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.BallDropping.Cycle.BallDropHigh;
import frc.robot.commands.BallDropping.Cycle.BallDropLow;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.EndEffector.Eject;
import frc.robot.subsystems.BallDropping.BallDropping;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;
import frc.robot.subsystems.RobotAuto.RobotAutoConstants;

import java.util.Set;
import java.util.function.Supplier;


public class MasterCommand extends Command {
    private final Supplier<ElevatorConstants.ElevatorLevel> levelSupplier;
    private final Supplier<Constants.ReefLocation> targetReefSupplier;
    private final Supplier<Boolean> shouldDropBallSupplier;

    private final Command coralCommand;

    private final ReefFinderWrapper pathCommand;
    private final MoveToLevel moveElevatorCommand;
    private final Eject ejectCommand;
    private final HomeToReef homeToReef;

    private boolean shouldDropBall = false;
    private Constants.ReefLocation targetReef = null;
    private ElevatorConstants.ElevatorLevel level = null;

    public MasterCommand(DriveBase driveBase, Elevator elevator, EndEffector endEffector, BallDropping ballDropping,
                         Supplier<ElevatorConstants.ElevatorLevel> levelSupplier, Supplier<Constants.ReefLocation> targetReefSupplier,
                         Supplier<Boolean> dropBall) {

        this.targetReefSupplier = targetReefSupplier;
        this.shouldDropBallSupplier = dropBall;
        this.levelSupplier = levelSupplier;

        // pathfinder phase (finding the path to the selected reef)
        pathCommand = new ReefFinderWrapper(driveBase, Constants.ReefLocation.REEF_1.getPose());

        // adjustment phase (minor adjustment to the reef and elevator raising)
        homeToReef = new HomeToReef(driveBase, Constants.ReefLocation.REEF_1.getPose());
        moveElevatorCommand = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Intake);
        Command adjustmentPhase = new ParallelCommandGroup(
                homeToReef,
                moveElevatorCommand,
                createBallDropCommand(ballDropping).onlyIf(() ->
                        shouldDropBall && checkBallDropTime(RobotAutoConstants.BallDropTime.PARALLEL) &&
                                targetReef.isBallDropInSamePose())
        );

        // eject phase (releasing the game piece)
        ejectCommand = new Eject(endEffector, EndEffectorConstants.MotorPower.L1);

        // elevator to home phase (moving the elevator to the home position)
        Command elevatorToHome = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom);

        // the main command
        coralCommand = new SequentialCommandGroup(
                pathCommand,
                createBallDropCommand(ballDropping).onlyIf(() ->
                        (shouldDropBall && checkBallDropTime(RobotAutoConstants.BallDropTime.BEFORE)) ||
                                !targetReef.isBallDropInSamePose()), // ball drop before the reef
                adjustmentPhase,
                ejectCommand,
                elevatorToHome,
                createBallDropCommand(ballDropping).onlyIf(() ->
                        shouldDropBall && checkBallDropTime(RobotAutoConstants.BallDropTime.AFTER) &&
                                targetReef.isBallDropInSamePose()) // ball drop after the reef
        );
    }

    private boolean checkBallDropTime(RobotAutoConstants.BallDropTime time) {
        return getBallDropTime(level) == time;
    }

    private RobotAutoConstants.BallDropTime getBallDropTime(ElevatorConstants.ElevatorLevel level) {
        return switch (level) {
            case L1 -> RobotAutoConstants.BallDropTime.AFTER;
            case L4 -> RobotAutoConstants.BallDropTime.PARALLEL;
            default -> RobotAutoConstants.BallDropTime.BEFORE;
        };
    }

    private Command createBallDropCommand(BallDropping ballDropping) {
        return new SequentialCommandGroup(
                new BallDropLow(ballDropping).onlyIf(() -> !targetReef.isBallInUpPosition()),
                new BallDropHigh(ballDropping).onlyIf(() -> targetReef.isBallInUpPosition())
        );
    }

    @Override
    public void initialize() {
        targetReef = targetReefSupplier.get(); // getting the target reef
        shouldDropBall = shouldDropBallSupplier.get(); // getting the ball drop status
        level = levelSupplier.get(); // getting the target level
        Pose2d pathFinderTarget = targetReef.getPose(); // setting the target pose for the path command


        if (shouldDropBall && !targetReef.isBallDropInSamePose()) {
            pathFinderTarget = Constants.ReefLocation.values()[(targetReef.ordinal() - 1)].getPose();
        }

        // setting the target pose for the path command
        pathCommand.setTargetPose(pathFinderTarget);
        //setting the target pose for the adjustment phase
        homeToReef.setTargetPose(targetReef.getPose());

        moveElevatorCommand.changeDesiredlevel(level);
        ejectCommand.setLevel(getMotorPower(level));

        coralCommand.initialize();
    }

    private EndEffectorConstants.MotorPower getMotorPower(ElevatorConstants.ElevatorLevel level) {
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
        return this.coralCommand.getInterruptionBehavior();
    }
}
