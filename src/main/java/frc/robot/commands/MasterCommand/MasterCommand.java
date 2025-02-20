package frc.robot.commands.MasterCommand;


import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.ReefLocation;
import frc.robot.commands.BallDropping.Sequence.BallDropHigh;
import frc.robot.commands.BallDropping.Sequence.BallDropLow;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.EndEffector.Eject;
import frc.robot.commands.EndEffector.Funnel.ToggleFunnel;
import frc.robot.commands.EndEffector.Funnel.YeetFunnel;
import frc.robot.subsystems.BallDropping.BallDropping;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;
import frc.robot.subsystems.RobotAuto.RobotAutoConstants;
import frc.robot.subsystems.RobotAuto.RobotAutoConstants.BallDropTime;

import java.util.Set;
import java.util.function.Supplier;


public class MasterCommand extends Command {
    private final Supplier<ElevatorConstants.ElevatorLevel> levelSupplier;
    private final Supplier<Constants.ReefLocation> targetReefSupplier;
    private final Supplier<Boolean> shouldDropBallSupplier;

    private final Command coralCommand;

    private final ReefFinderWrapper pathCommand;
    private final ReefFinderWrapper finalReefCommand;
    private final MoveToLevel moveElevatorCommand;
    private final Eject ejectCommand;
    private final HomeToReef homeToReef;
    private final HomeToReef homeToBallReef;
    // private final HomeToReefEndless homeToReefEndless;

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
        pathCommand = new ReefFinderWrapper(driveBase, Constants.ReefLocation.REEF_1); // setting the default target pose

        finalReefCommand = new ReefFinderWrapper(driveBase, ReefLocation.REEF_1);

        // adjustment phase (minor adjustment to the reef and elevator raising)
        homeToReef = new HomeToReef(driveBase, Constants.ReefLocation.REEF_1);
        homeToBallReef = new HomeToReef(driveBase, ReefLocation.REEF_1);
        moveElevatorCommand = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom);
        Command adjustmentPhase = new ParallelCommandGroup(
//                moveElevatorCommand,
                homeToReef,
                createBallDropCommand(ballDropping, endEffector).onlyIf(() ->
                        checkBallDropTime(RobotAutoConstants.BallDropTime.PARALLEL))
        );

        // eject phase (releasing the game piece)
        ejectCommand = new Eject(endEffector, EndEffectorConstants.MotorPower.L1);

        // homeToReefEndless = new HomeToReefEndless(driveBase, Constants.ReefLocation.REEF_1);

        // elevator to home phase (moving the elevator to the home position)
        Command elevatorToHome = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom);

        Command ejectPhase = 
                new SequentialCommandGroup(
                        ejectCommand,
                        elevatorToHome,
                        createBallDropCommand(ballDropping, endEffector).onlyIf(() ->
                                checkBallDropTime(RobotAutoConstants.BallDropTime.AFTER)) // ball drop after reef
        );

        // the main command
        coralCommand = new SequentialCommandGroup(
                pathCommand,
                new SequentialCommandGroup(
                    homeToBallReef,
                    createBallDropCommand(ballDropping, endEffector)// ball drop before the reef
                ).onlyIf(() -> checkBallDropTime(RobotAutoConstants.BallDropTime.BEFORE)),
                finalReefCommand.onlyIf(() -> !targetReef.isBallDropInSamePose() && shouldDropBall),
                adjustmentPhase,
                new WaitCommand(0.2),
                ejectPhase
        );
    }

    private boolean checkBallDropTime(RobotAutoConstants.BallDropTime time) {
        return getBallDropTime(level, targetReef.isBallDropInSamePose(), targetReef.isBallInUpPosition()) == time
                && shouldDropBall;
    }

    private RobotAutoConstants.BallDropTime getBallDropTime(ElevatorConstants.ElevatorLevel level, boolean samePosition, boolean ballDropUp) {
        if(!ballDropUp) return BallDropTime.NEVER;

        if (!samePosition) return RobotAutoConstants.BallDropTime.BEFORE;

        switch (level) {
            case L1, L2: return BallDropTime.PARALLEL;

            case L3: return BallDropTime.BEFORE;

            case L4: return BallDropTime.AFTER;

            default: return BallDropTime.AFTER;
        }
    }

    private Command createBallDropCommand(BallDropping ballDropping, EndEffector endEffector) {
        return new SequentialCommandGroup(
            new BallDropHigh(ballDropping),
            new ToggleFunnel(endEffector),
            new ToggleFunnel(endEffector)
        );
    }

    @Override
    public void initialize() {
        targetReef = targetReefSupplier.get(); // getting the target reef
        shouldDropBall = shouldDropBallSupplier.get(); // getting the ball drop status
        level = levelSupplier.get(); // getting the target level
        Constants.ReefLocation pathFinderTarget = targetReef; // setting the target pose for the path command

        if (shouldDropBall && !targetReef.isBallDropInSamePose()) {
            pathFinderTarget = Constants.ReefLocation.values()[(targetReef.ordinal() - 1)];
        }

        // setting the target pose for the path command
        pathCommand.setTargetPose(pathFinderTarget);
        homeToBallReef.setTargetPose(pathFinderTarget);
        //setting the target pose for the adjustment phase
        homeToReef.setTargetPose(targetReef);
        finalReefCommand.setTargetPose(targetReef);
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
        return this.coralCommand.getInterruptionBehavior();
    }
}
