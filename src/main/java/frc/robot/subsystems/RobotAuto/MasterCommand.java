package frc.robot.subsystems.RobotAuto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.EndEffector.Eject;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.EndEffector.EndEffector;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import frc.robot.subsystems.EndEffector.EndEffectorConstants;
import frc.robot.subsystems.RobotAuto.RobotAutoConstants.FeederLocations;

public class MasterCommand extends SequentialCommandGroup {
    public MasterCommand(DriveBase driveBase, EndEffector endEffector, Elevator elevator,
                         Supplier<ElevatorConstants.ElevatorLevel> chosenLevel) {

        BooleanSupplier closerToTopFeeder = () -> driveBase.getPose().getTranslation().getDistance(FeederLocations.TOP.robotPose.getTranslation()) <
                driveBase.getPose().getTranslation().getDistance(FeederLocations.BOTTOM.robotPose.getTranslation());

        BooleanSupplier closerToBottomFeeder = () -> !closerToTopFeeder.getAsBoolean();

        Supplier<EndEffectorConstants.MotorPower> motorPowerSupplier = () -> switch (chosenLevel.get()) {
            case L2, L3 -> EndEffectorConstants.MotorPower.L2_3;
            case L4 -> EndEffectorConstants.MotorPower.L4;
            default -> EndEffectorConstants.MotorPower.L1;
        };

        addCommands(
                // Move to the feeder if no gamepieces are loaded
                new SequentialCommandGroup(
                        driveBase.findPath(FeederLocations.TOP.robotPose).onlyIf(closerToTopFeeder),
                        driveBase.findPath(FeederLocations.BOTTOM.robotPose).onlyIf(closerToBottomFeeder)
                ).onlyIf(() -> !endEffector.gpLoaded()),

                //Move to the reef if a gamepiece is loaded
                new SequentialCommandGroup(
                        new PathFindToReef(driveBase),
                        new ParallelCommandGroup(
                                new HomeToReef(driveBase),
                                new MoveToSelectedLevel(elevator, chosenLevel)
                        ),
                        new Eject(endEffector, motorPowerSupplier),
                        new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Intake)

                ).onlyIf(endEffector::gpLoaded)
        );
    }

    private static class PathFindToReef extends Command {
        private final DriveBase driveBase;
        private Command pathCommand;

        public PathFindToReef(DriveBase driveBase) {
            this.driveBase = driveBase;
            addRequirements(this.driveBase);
        }

        @Override
        public void initialize() {
            Pose2d currentPose = driveBase.getPose();
            Pose2d reefPose = currentPose.nearest(RobotAutoConstants.reefPoses);

            pathCommand = driveBase.findPath(reefPose);
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) pathCommand.cancel();
        }

        @Override
        public boolean isFinished() {
            return pathCommand.isFinished();
        }
    }

    private static class MoveToSelectedLevel extends Command {
        private final Elevator elevator;
        private final Supplier<ElevatorConstants.ElevatorLevel> location;
        private Command moveToLevel;

        public MoveToSelectedLevel(Elevator elevator, Supplier<ElevatorConstants.ElevatorLevel> location) {
            this.elevator = elevator;
            this.location = location;
            addRequirements(this.elevator);
        }

        @Override
        public void initialize() {
            ElevatorConstants.ElevatorLevel location = this.location.get();

            moveToLevel = new MoveToLevel(elevator, location);

            moveToLevel.schedule();
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) moveToLevel.cancel();
        }

        @Override
        public boolean isFinished() {
            return moveToLevel.isFinished();
        }
    }
}