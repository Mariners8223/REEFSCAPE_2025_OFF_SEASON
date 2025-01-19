package frc.robot.subsystems.RobotAuto.MasterCommand;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.EndEffector.EndEffector;

import java.util.function.Supplier;

public class MasterCommand extends SequentialCommandGroup {
    public MasterCommand(DriveBase driveBase, Elevator elevator, EndEffector endEffector,
                         Supplier<ElevatorConstants.ElevatorLevel> levelSupplier, Supplier<Pose2d> targetPoseSupplier,
                         Supplier<Pose2d> chosenFeederSupplier) {
        addCommands(
                new CoralCommand(driveBase, elevator, endEffector, levelSupplier, targetPoseSupplier).onlyIf(endEffector::gpLoaded),
                new FeederCommand(driveBase, chosenFeederSupplier).onlyIf(() -> !endEffector.gpLoaded())
        );
    }
}