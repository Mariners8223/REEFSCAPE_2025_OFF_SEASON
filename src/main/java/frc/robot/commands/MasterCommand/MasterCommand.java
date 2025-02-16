package frc.robot.commands.MasterCommand;


import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.BallDropping.Sequence.BallDropHigh;
import frc.robot.commands.BallDropping.Sequence.BallDropLow;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.EndEffector.Eject;
import frc.robot.commands.EndEffector.Funnel.YeetFunnel;
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
    private final HomeToReefEndless homeToReefEndless;

    private boolean shouldDropBall = false;
    private Constants.ReefLocation targetReef = null;
    private ElevatorConstants.ElevatorLevel level = null;

    private final Alert alert = new Alert("illegal ball drop", Alert.AlertType.kWarning);

    public MasterCommand(DriveBase driveBase, Elevator elevator, EndEffector endEffector, BallDropping ballDropping,
                         Supplier<ElevatorConstants.ElevatorLevel> levelSupplier, Supplier<Constants.ReefLocation> targetReefSupplier,
                         Supplier<Boolean> dropBall) {

        this.targetReefSupplier = targetReefSupplier;
        this.shouldDropBallSupplier = dropBall;
        this.levelSupplier = levelSupplier;

        // pathfinder phase (finding the path to the selected reef)
        pathCommand = new ReefFinderWrapper(driveBase, Constants.ReefLocation.REEF_1); // setting the default target pose

        // adjustment phase (minor adjustment to the reef and elevator raising)
        homeToReef = new HomeToReef(driveBase, Constants.ReefLocation.REEF_1);
        moveElevatorCommand = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom);
        Command adjustmentPhase = new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    moveElevatorCommand,
                    new WaitCommand(1.8)
                ),
                homeToReef,
                createBallDropCommand(ballDropping, endEffector).onlyIf(() ->
                        checkBallDropTime(RobotAutoConstants.BallDropTime.PARALLEL))
        );

        // eject phase (releasing the game piece)
        ejectCommand = new Eject(endEffector, EndEffectorConstants.MotorPower.L1);

        homeToReefEndless = new HomeToReefEndless(driveBase, Constants.ReefLocation.REEF_1);

        // elevator to home phase (moving the elevator to the home position)
        Command elevatorToHome = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom);

        Command ejectPhase = new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        ejectCommand,
                        elevatorToHome,
                        createBallDropCommand(ballDropping, endEffector).onlyIf(() ->
                                checkBallDropTime(RobotAutoConstants.BallDropTime.AFTER)) // ball drop after reef
                ),
                homeToReefEndless
        );

        // the main command
        coralCommand = new SequentialCommandGroup(
                pathCommand,
                createBallDropCommand(ballDropping, endEffector).onlyIf(() ->
                        checkBallDropTime(RobotAutoConstants.BallDropTime.BEFORE)),// ball drop before the reef
                adjustmentPhase,
                ejectPhase
        );
    }

    private boolean checkBallDropTime(RobotAutoConstants.BallDropTime time) {
        return getBallDropTime(level, targetReef.isBallDropInSamePose(), targetReef.isBallInUpPosition()) == time
                && shouldDropBall;
    }

    private RobotAutoConstants.BallDropTime getBallDropTime(ElevatorConstants.ElevatorLevel level, boolean samePosition, boolean ballDropUp) {
        if (!samePosition) return RobotAutoConstants.BallDropTime.BEFORE;

        if (level == ElevatorConstants.ElevatorLevel.L4 || level == ElevatorConstants.ElevatorLevel.L1)
            return RobotAutoConstants.BallDropTime.PARALLEL;

        if (level == ElevatorConstants.ElevatorLevel.L3) return RobotAutoConstants.BallDropTime.AFTER;

        if (ballDropUp) return RobotAutoConstants.BallDropTime.PARALLEL;

        else return RobotAutoConstants.BallDropTime.NEVER;

    }

    private Command createBallDropCommand(BallDropping ballDropping, EndEffector endEffector) {
        return new SequentialCommandGroup(
                new BallDropLow(ballDropping).onlyIf(() -> !targetReef.isBallInUpPosition()),
                new SequentialCommandGroup(
                        new BallDropHigh(ballDropping),
                        new YeetFunnel(endEffector)
                ).onlyIf(() -> targetReef.isBallInUpPosition())
        );
    }

    @Override
    public void initialize() {
        targetReef = targetReefSupplier.get(); // getting the target reef
        shouldDropBall = shouldDropBallSupplier.get(); // getting the ball drop status
        level = levelSupplier.get(); // getting the target level
        Constants.ReefLocation pathFinderTarget = targetReef; // setting the target pose for the path command

        if(checkBallDropTime(RobotAutoConstants.BallDropTime.NEVER)){
            alert.set(true);

            cancel();
            return;
        }


        if (shouldDropBall && !targetReef.isBallDropInSamePose()) {
            pathFinderTarget = Constants.ReefLocation.values()[(targetReef.ordinal() - 1)];
        }

        // setting the target pose for the path command
        pathCommand.setTargetPose(pathFinderTarget);
        //setting the target pose for the adjustment phase
        homeToReef.setTargetPose(targetReef);
        homeToReefEndless.setTargetPose(targetReef);

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
        return this.coralCommand.getInterruptionBehavior();
    }
}
