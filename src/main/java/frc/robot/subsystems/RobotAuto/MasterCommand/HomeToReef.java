package frc.robot.subsystems.RobotAuto.MasterCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.DriveTrain.DriveBaseConstants;
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
        addRequirements(this.driveBase);


        XController = DriveBaseConstants.PathPlanner.XY_PID.createPIDController();
        YController = DriveBaseConstants.PathPlanner.XY_PID.createPIDController();
        ThetaController = DriveBaseConstants.PathPlanner.THETA_PID.createPIDController();
    }

    public void setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
    }

    @Override
    public void execute() {
        Pose2d robotPose = driveBase.getPose();

        double xOutput = XController.calculate(robotPose.getX(), targetPose.getX());
        double yOutput = YController.calculate(robotPose.getY(), targetPose.getY());
        double thetaOutput =
                ThetaController.calculate(robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(xOutput, yOutput, thetaOutput);

        ChassisSpeeds robotRelativeSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, driveBase.getRotation2d());

        driveBase.drive(clampChassisSpeeds(robotRelativeSpeeds));
    }

    private static ChassisSpeeds clampChassisSpeeds(ChassisSpeeds robotRelativeSpeeds) {
        double maxXYSpeed = RobotAutoConstants.HomingConstants.MAX_HOME_SPEED_METERS_PER_SECOND;
        double maxThetaSpeed = RobotAutoConstants.HomingConstants.MAX_HOME_SPEED_RADIANS_PER_SECOND;

        return new ChassisSpeeds(
                MathUtil.clamp(robotRelativeSpeeds.vxMetersPerSecond, -maxXYSpeed, maxXYSpeed),
                MathUtil.clamp(robotRelativeSpeeds.vyMetersPerSecond, -maxXYSpeed, maxXYSpeed),
                MathUtil.clamp(robotRelativeSpeeds.omegaRadiansPerSecond, -maxThetaSpeed, maxThetaSpeed)
        );
    }

    @Override
    public boolean isFinished() {
        double xError = XController.getError();
        double yError = YController.getError();
        double thetaError = ThetaController.getError();

        double xyTolerance = RobotAutoConstants.HomingConstants.XY_TOLERANCE;
        double thetaTolerance = RobotAutoConstants.HomingConstants.THETA_TOLERANCE;

        return xError <= xyTolerance && yError <= xyTolerance && thetaError <= thetaTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(new ChassisSpeeds());
    }
}
