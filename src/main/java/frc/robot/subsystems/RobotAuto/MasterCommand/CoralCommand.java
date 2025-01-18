package frc.robot.subsystems.RobotAuto.MasterCommand;

import com.fasterxml.jackson.core.PrettyPrinter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.EndEffector.Eject;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;
import frc.robot.subsystems.RobotAuto.RobotAutoConstants;

import java.util.function.Supplier;


public class CoralCommand extends Command {
    private Command coralCommand;

    private final Command moveElevatorCommand;
    private final Command ejectCommand;
    private final Command elevatorToHome;

    private final DriveBase driveBase;

    public CoralCommand(DriveBase driveBase, Elevator elevator, EndEffector endEffector, Supplier<ElevatorConstants.ElevatorLevel> levelSupplier) {
        this.driveBase = driveBase;

        moveElevatorCommand = new MoveToLevel(elevator, levelSupplier);
        ejectCommand = new Eject(endEffector, () -> getMotorPower(levelSupplier.get()));

        elevatorToHome = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom);
    }

    @Override
    public void initialize() {
        Pose2d targetPose = driveBase.getPose().nearest(RobotAutoConstants.reefPoses);

        Command pathCommand = driveBase.findPath(targetPose);

        Command homeToReef = new HomeToReef(driveBase, targetPose);

        coralCommand = new SequentialCommandGroup(
                pathCommand,
                new ParallelCommandGroup(
                        moveElevatorCommand,
                        homeToReef
                ),
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
