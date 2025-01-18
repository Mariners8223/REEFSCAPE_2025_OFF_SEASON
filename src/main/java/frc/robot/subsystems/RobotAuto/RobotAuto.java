package frc.robot.subsystems.RobotAuto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.EndEffector.Intake.Intake;
import frc.robot.subsystems.BallDropping.BallDropping;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.EndEffector.EndEffector;

public class RobotAuto extends SubsystemBase {
    private final double triangleSize;

    private final EndEffector endEffector;
    private final DriveBase driveBase;
    private final Elevator elevator;

    private final Command moveElevatorToIntake;
    private final Command intakeCommand;

    public RobotAuto(DriveBase driveBase, Elevator elevator, EndEffector endEffector) {
        RobotAutoConstants.FeederLocations location = RobotAutoConstants.FeederLocations.TOP;

        triangleSize = calculateTriangleArea(location.bottomLeft, location.bottomRight, location.topLeft);

        this.endEffector = endEffector;
        this.driveBase = driveBase;
        this.elevator = elevator;

        moveElevatorToIntake = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Intake);
        intakeCommand = new Intake(endEffector);
    }

    @Override
    public void periodic() {
        if(!endEffector.gpLoaded()) return;

        Pose2d robotPose = driveBase.getPose();

        if(!withinFeeder(robotPose, RobotAutoConstants.FeederLocations.TOP)
                && !withinFeeder(robotPose, RobotAutoConstants.FeederLocations.BOTTOM)){
            if(intakeCommand.isScheduled()) intakeCommand.cancel();
            return;
        }

        ElevatorConstants.ElevatorLevel level = elevator.getLevel();

        if(!intakeCommand.isScheduled() && level == ElevatorConstants.ElevatorLevel.Intake){
            intakeCommand.schedule();
            return;
        }

        if(!moveElevatorToIntake.isScheduled() && level != ElevatorConstants.ElevatorLevel.Intake){
            moveElevatorToIntake.schedule();
        }
    }

    private boolean withinFeeder(Pose2d robotPose, RobotAutoConstants.FeederLocations feeder){
        return withinTriangle(robotPose, feeder.bottomLeft, feeder.bottomRight, feeder.topLeft)
                || withinTriangle(robotPose, feeder.bottomRight, feeder.topLeft, feeder.topRight);
    }

    private boolean withinTriangle(Pose2d robotPose, Translation2d point1, Translation2d point2, Translation2d point3){
        double s1 = calculateTriangleArea(robotPose.getTranslation(), point1, point2);
        double s2 = calculateTriangleArea(robotPose.getTranslation(), point2, point3);
        double s3 = calculateTriangleArea(robotPose.getTranslation(), point1, point3);

        return s1 + s2 + s3 == triangleSize;
    }

    private double calculateTriangleArea(Translation2d point1, Translation2d point2, Translation2d point3){
        return calculateTriangleArea(point1.getX(), point1.getY(), point2.getX(), point2.getY(), point3.getX(), point3.getY());
    }

    private double calculateTriangleArea(double x1, double y1, double x2, double y2, double x3, double y3){
        return 0.5 * Math.abs(x1 * (y2 - y3) + x2 * (y3-y1) + x3 *(y1-y2));
    }


}

