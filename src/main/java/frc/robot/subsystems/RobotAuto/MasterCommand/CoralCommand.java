package frc.robot.subsystems.RobotAuto.MasterCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.EndEffector.Eject;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;

import java.util.function.Supplier;


public class CoralCommand extends Command {
    private Command coralCommand;

    private final DriveBase driveBase;
    private final Supplier<ElevatorConstants.ElevatorLevel> levelSupplier;
    private final Supplier<Pose2d> targetPoseSupplier;

    private final MoveToLevel moveElevatorCommand;
    private final Eject ejectCommand;
    private final Command elevatorToHome;
    private final HomeToReef homeToReef;
    private final Command adjustmentPhase;


    public CoralCommand(DriveBase driveBase, Elevator elevator, EndEffector endEffector,
                        Supplier<ElevatorConstants.ElevatorLevel> levelSupplier, Supplier<Pose2d> targetPoseSupplier) {
        this.driveBase = driveBase;
        this.targetPoseSupplier = targetPoseSupplier;

        moveElevatorCommand = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Intake);
        ejectCommand = new Eject(endEffector, EndEffectorConstants.MotorPower.L1);

        elevatorToHome = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom);

        homeToReef = new HomeToReef(driveBase, Constants.ReefLocation.REEF_1.getPose());

        this.adjustmentPhase = new ParallelCommandGroup(
                homeToReef,
                moveElevatorCommand
        );

        this.levelSupplier = levelSupplier;
    }

    @Override
    public void initialize() {
        Pose2d targetPose = targetPoseSupplier.get();
        Command pathCommand = driveBase.findPath(targetPose);
        homeToReef.setTargetPose(targetPose);

        ElevatorConstants.ElevatorLevel level = levelSupplier.get();

        moveElevatorCommand.setDesiredLevel(level);
        ejectCommand.setMotorPower(getMotorPower(level));

        coralCommand = new SequentialCommandGroup(
                pathCommand,
                adjustmentPhase,
                ejectCommand,
                elevatorToHome
        );
    }

    @Override
    public boolean isFinished() {
        return !coralCommand.isScheduled();
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) coralCommand.cancel();
    }

    private EndEffectorConstants.MotorPower getMotorPower(ElevatorConstants.ElevatorLevel level){
        return switch (level){
            case L2,L3 -> EndEffectorConstants.MotorPower.L2_3;
            case L4 -> EndEffectorConstants.MotorPower.L4;
            default -> EndEffectorConstants.MotorPower.L1;
        };
    }
}
