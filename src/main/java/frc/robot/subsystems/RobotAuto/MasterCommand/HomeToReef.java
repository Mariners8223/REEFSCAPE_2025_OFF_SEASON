package frc.robot.subsystems.RobotAuto.MasterCommand;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.RobotAuto.RobotAutoConstants;


public class HomeToReef extends Command {
    private final DriveBase driveBase;
    private Pose2d targetPose;

    private final PIDController XController;
    private final PIDController YController;
    private final PIDController ThetaController;

    public HomeToReef(DriveBase driveBase, Pose2d targetPose) {
        this.driveBase = driveBase;
        this.targetPose = targetPose;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(driveBase);


        XController = RobotAutoConstants.HomingConstants.XY_PID.createPIDController();
        YController = RobotAutoConstants.HomingConstants.XY_PID.createPIDController();
        ThetaController = RobotAutoConstants.HomingConstants.THETA_PID.createPIDController();

        XController.setIZone(Double.POSITIVE_INFINITY);
        YController.setIZone(Double.POSITIVE_INFINITY);
        ThetaController.setIZone(Double.POSITIVE_INFINITY);

        ThetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;

        Logger.recordOutput("home to reef/target Pose", targetPose);
    }

    @Override
    public void initialize() {
        XController.setSetpoint(targetPose.getX());
        YController.setSetpoint(targetPose.getY());
        ThetaController.setSetpoint(targetPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d robotPose = driveBase.getPose();

        double xOutput = XController.calculate(robotPose.getX(), targetPose.getX());
        double yOutput = YController.calculate(robotPose.getY(), targetPose.getY());

        double thetaOutput =
            ThetaController.calculate(robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        double maxOutput = RobotAutoConstants.HomingConstants.MAX_HOME_SPEED_METERS_PER_SECOND;
        xOutput = MathUtil.clamp(xOutput, -maxOutput, maxOutput);
        yOutput = MathUtil.clamp(yOutput, -maxOutput, maxOutput);

        maxOutput = RobotAutoConstants.HomingConstants.MAX_HOME_SPEED_RADIANS_PER_SECOND;
        thetaOutput = MathUtil.clamp(thetaOutput, -maxOutput, maxOutput);


        Logger.recordOutput("home to reef/xOutput", xOutput);
        Logger.recordOutput("home to reef/yOutput", yOutput);
        Logger.recordOutput("home to reef/thetaOutput", thetaOutput);

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(xOutput, yOutput, thetaOutput);

        ChassisSpeeds robotRelativeSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, driveBase.getRotation2d());

        driveBase.drive(robotRelativeSpeeds);
    }

    @Override
    public boolean isFinished() {
        double xError = Math.abs(XController.getError());
        double yError = Math.abs(YController.getError());
        double thetaError = Math.abs(ThetaController.getError());

        Logger.recordOutput("home to reef/x error", xError);
        Logger.recordOutput("home to reef/y error", yError);
        Logger.recordOutput("home to reef/theta error", thetaError);

        double xyTolerance = RobotAutoConstants.HomingConstants.XY_TOLERANCE;
        double thetaTolerance = RobotAutoConstants.HomingConstants.THETA_TOLERANCE;

        return xError <= xyTolerance && yError <= xyTolerance && thetaError <= thetaTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(new ChassisSpeeds());
        System.out.println("HomeToReef ended, interrupted: " + interrupted);
    }
}
