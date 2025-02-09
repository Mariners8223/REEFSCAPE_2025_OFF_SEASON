package frc.robot.subsystems.RobotAuto;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
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
            new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom),
            new Intake(endEffector),
            new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom)
        );

        SmartDashboard.putBoolean("Level 1", false);
        SmartDashboard.putBoolean("Level 2", false);
        SmartDashboard.putBoolean("Level 3", false);
        SmartDashboard.putBoolean("Level 4", false);

        SmartDashboard.putBoolean("drop ball", false);
    }

    @Override
    public void periodic() {
        Pose2d robotPose = driveBase.getPose();

        boolean withinLeftFeeder = FeederLocation.LEFT.withinFeeder(robotPose);
        boolean withinRightFeeder = FeederLocation.RIGHT.withinFeeder(robotPose);

        Logger.recordOutput("AutoIntake/Within left feeder", withinLeftFeeder);
        Logger.recordOutput("AutoIntake/Within right feeder", withinRightFeeder);

        ElevatorConstants.ElevatorLevel level = elevator.getCurrentLevel();

        if(endEffector.isGpLoaded()) return;

        if(withinLeftFeeder || withinRightFeeder) {
            if(!sequnceCommand.isScheduled()) sequnceCommand.schedule();
        }
        else{
            if(sequnceCommand.isScheduled()) sequnceCommand.cancel();
            if(level != ElevatorConstants.ElevatorLevel.Bottom && !moveElevatorToBottom.isScheduled()) moveElevatorToBottom.schedule();
        }
    }


    private Constants.ReefLocation selectedReef = Constants.ReefLocation.REEF_1;
    private ElevatorConstants.ElevatorLevel selectedLevel = ElevatorConstants.ElevatorLevel.L1;
    private boolean dropBallInCycle = false;

    public Constants.ReefLocation getSelectedReef() {
        return selectedReef;
    }

    public ElevatorConstants.ElevatorLevel getSelectedLevel() {
        return selectedLevel;
    }

    public boolean shouldDropBallInCycle() {
        return dropBallInCycle;
    }

    public void setSelectedReef(Constants.ReefLocation reef) {
        String name;

        if(reef != null){
            Robot.setObjectPoseFiled("selected reef", reef.getPose());
            name = reef.name();
        }
        else{
            Robot.clearObjectPoseField("selected reef");
            name = "None";
        }

        Logger.recordOutput("Selection/Reef", name);

        selectedReef = reef;
    }

    public void setSelectedLevel(ElevatorConstants.ElevatorLevel level) {
        String name;

        SmartDashboard.putBoolean("Level " + (selectedLevel.ordinal() + 1), false);
        if(level != null){
            SmartDashboard.putBoolean("Level " + (level.ordinal() + 1), true);
            name = level.name();
        }
        else name = "None";

        Logger.recordOutput("Selection/Level", name);

        selectedLevel = level;
    }

    public void setDropBallInCycle(boolean dropBall) {
        Logger.recordOutput("Selection/Should Drop Ball", dropBall);

        SmartDashboard.putBoolean("drop ball", dropBall);
        dropBallInCycle = dropBall;
    }

}

