package frc.robot.commands.MasterCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.BallDropping.BallDropOff;
import frc.robot.commands.BallDropping.BallDropOnForHigh;
import frc.robot.commands.BallDropping.BallDropOnForLow;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.EndEffector.Eject;
import frc.robot.subsystems.BallDropping.BallDropping;
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
    private final Supplier<Boolean> shouldDropBallSupplier;

    private final Command coralCommand;

    private final ReefFinderWrapper pathCommand;
    private final MoveToLevel moveElevatorCommand;
    private final Eject ejectCommand;
    private final HomeToReef homeToReef;

    private Constants.ReefLocation targetReef = Constants.ReefLocation.REEF_1;
    private boolean shouldDropBall = false;

    public MasterCommand(DriveBase driveBase, Elevator elevator, EndEffector endEffector, BallDropping ballDropping,
                         Supplier<ElevatorConstants.ElevatorLevel> levelSupplier, Supplier<Constants.ReefLocation> targetReefSupplier,
                         Supplier<Boolean> dropBall) {

        this.targetReefSupplier = targetReefSupplier;
        this.shouldDropBallSupplier = dropBall;
        this.levelSupplier = levelSupplier;

        // pather finder phase (finding the path to the selected reef)
        pathCommand = new ReefFinderWrapper(driveBase, Constants.ReefLocation.REEF_1.getPose());
        // ball dropping phase (optinal)
        Command ballDroppingCommand = createBallDroppingCommand(ballDropping).onlyIf(() -> shouldDropBall);
        // adjustment phase (minor adjustment to the reef and elevator raising)
        homeToReef = new HomeToReef(driveBase, Constants.ReefLocation.REEF_1.getPose());
        moveElevatorCommand = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Intake);
        Command adjustmentPhase = new ParallelCommandGroup(
                homeToReef,
                moveElevatorCommand
        );
        // eject phase (rleasing the game piece)
        ejectCommand = new Eject(endEffector, EndEffectorConstants.MotorPower.L1);
        // elevator to home phase (moving the elevator to the home position)
        Command elevatorToHome = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom);

        // the main command
        coralCommand = new SequentialCommandGroup(
                pathCommand,
                ballDroppingCommand,
                adjustmentPhase,
                ejectCommand,
                elevatorToHome
        );
    }

    private Command createBallDroppingCommand(BallDropping ballDropping) {
        return new SequentialCommandGroup(
                new BallDropOnForHigh(ballDropping).onlyIf(() -> targetReef.isBallInUpPosition()),
                new BallDropOnForLow(ballDropping).onlyIf(() -> !targetReef.isBallInUpPosition()),
                new WaitCommand(2),
                new BallDropOff(ballDropping)
        );
    }

    public Command getPathCommand() {
        return pathCommand;
    }

    @Override
    public void initialize() {
        targetReef = targetReefSupplier.get(); // getting the target reef
        shouldDropBall = shouldDropBallSupplier.get(); // getting the ball drop status

        Pose2d pathFinderTarget = targetReef.getPose();

        if(shouldDropBall && !targetReef.isBallDropInSamePose()){
            pathFinderTarget = Constants.ReefLocation.values()[(targetReef.ordinal() + 1)].getPose();
        }

        // setting the target pose for the path command
        pathCommand.setTargetPose(pathFinderTarget);

        //setting the target pose for the adjustment phase
        homeToReef.setTargetPose(targetReef.getPose());

        ElevatorConstants.ElevatorLevel level = levelSupplier.get();

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
