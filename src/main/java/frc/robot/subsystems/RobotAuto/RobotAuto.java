package frc.robot.subsystems.RobotAuto;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederLocation;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.EndEffector.Intake.Intake;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.EndEffector.EndEffector;

public class RobotAuto extends SubsystemBase {
    private final EndEffector endEffector;
    private final DriveBase driveBase;
    private final Elevator elevator;

    private final Command moveElevatorToBottom;

    private final Command sequnceCommand;

    public RobotAuto(DriveBase driveBase, Elevator elevator, EndEffector endEffector) {
        this.endEffector = endEffector;
        this.driveBase = driveBase;
        this.elevator = elevator;

        moveElevatorToBottom = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom);

        sequnceCommand = new SequentialCommandGroup(
            new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Intake),
            new Intake(endEffector),
            new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom)
        );
    }

    @Override
    public void periodic() {
        Pose2d robotPose = driveBase.getPose();

        boolean withinLeftFeeder = FeederLocation.LEFT.withinFeeder(robotPose);
        boolean withinRightFeeder = FeederLocation.RIGHT.withinFeeder(robotPose);

        Logger.recordOutput("AutoIntake/Within left feeder", withinLeftFeeder);
        Logger.recordOutput("AutoIntake/Within right feeder", withinRightFeeder);

        ElevatorConstants.ElevatorLevel level = elevator.getCurrentLevel();

        if(endEffector.gpLoaded()) return;

        if(withinLeftFeeder || withinRightFeeder) {
            if(!sequnceCommand.isScheduled()) sequnceCommand.schedule();
        }
        else{
            if(sequnceCommand.isScheduled()) sequnceCommand.cancel();
            if(level != ElevatorConstants.ElevatorLevel.Bottom && !moveElevatorToBottom.isScheduled()) moveElevatorToBottom.schedule();
        }
    }


}

