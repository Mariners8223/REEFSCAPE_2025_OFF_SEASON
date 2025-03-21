package frc.robot.commands.MasterCommand;


import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.ReefLocation;
import frc.robot.Robot;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.EndEffector.EjectSequance;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.RobotAuto.RobotAutoConstants;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;


public class MasterCommand extends Command {
    private final Supplier<ElevatorConstants.ElevatorLevel> levelSupplier;
    private final Supplier<Constants.ReefLocation> targetReefSupplier;

    private final Command coralCommand;

    private final ReefFinderWrapper pathCommand;
    private final MoveToLevel moveElevatorCommand;
    private final EjectSequance ejectCommand;
    private final HomeToReef homeToReef;

    public MasterCommand(DriveBase driveBase, Elevator elevator, EndEffector endEffector, EventTrigger moveElevatorMarker,
                         Supplier<ElevatorConstants.ElevatorLevel> levelSupplier, Supplier<Constants.ReefLocation> targetReefSupplier, LED led) {

        this.targetReefSupplier = targetReefSupplier;
        this.levelSupplier = levelSupplier;

        // pathfinder phase (finding the path to the selected reef)
        pathCommand = new ReefFinderWrapper(driveBase, Constants.ReefLocation.REEF_1); // setting the default target pose

        // adjustment phase (minor adjustment to the reef and elevator raising)
        homeToReef = new HomeToReef(driveBase, Constants.ReefLocation.REEF_1);
        moveElevatorCommand = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom);

        // eject phase (releasing the game piece)
        ejectCommand = new EjectSequance(endEffector, elevator);

        // elevator to home phase (moving the elevator to the home position)
        Command elevatorToHome = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom);

        WaitUntilCommand waitUntilMarker = new WaitUntilCommand(moveElevatorMarker);

        BooleanSupplier isRobotFarFromTarget = () -> isRobotFarFromTarget(driveBase.getPose(), targetReefSupplier.get().getPose());

        // the main command
        coralCommand = new SequentialCommandGroup(
                led.blinkWithRSLCommand(Robot.isRedAlliance),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                pathCommand.onlyIf(isRobotFarFromTarget),
                                homeToReef
                        ),
                        new SequentialCommandGroup(
                                waitUntilMarker.onlyIf(isRobotFarFromTarget),
                                moveElevatorCommand
                        )
                ),
                ejectCommand,
                led.putDefaultPatternCommand(),
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
        ejectCommand.setLevel(getMotorPower(level, targetReef));

        coralCommand.initialize();
    }

    public static EndEffectorConstants.MotorPower getMotorPower(ElevatorConstants.ElevatorLevel level, ReefLocation targetReef) {
        if(targetReef == null) targetReef = ReefLocation.REEF_1;

        return switch (level) {
            case L1 -> targetReef.ordinal()%2 == 0 ?
                    EndEffectorConstants.MotorPower.L1_LEFT : EndEffectorConstants.MotorPower.L1_RIGHT;
            case L2, L3 -> EndEffectorConstants.MotorPower.L2_3;
            case L4 -> EndEffectorConstants.MotorPower.L4;
            default -> EndEffectorConstants.MotorPower.L1_LEFT;
        };
    }

    public static boolean isRobotFarFromTarget(Pose2d currentPose, Pose2d targetPose) {
        Translation2d currentTranslation = currentPose.getTranslation();
        Translation2d targetTranslation = targetPose.getTranslation();

        return currentTranslation.getDistance(targetTranslation) > RobotAutoConstants.FAR_FROM_TARGET_DISTANCE;
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
