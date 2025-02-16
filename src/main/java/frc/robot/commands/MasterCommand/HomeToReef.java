package frc.robot.commands.MasterCommand;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ReefLocation;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.RobotAuto.RobotAutoConstants;


public class HomeToReef extends Command {
    private final DriveBase driveBase;
    private ReefLocation targetReef;

    private final PIDController XController;
    private final PIDController YController;
    private final PIDController ThetaController;

    public HomeToReef(DriveBase driveBase, ReefLocation targetReef) {
        this.driveBase = driveBase;
        this.targetReef = targetReef;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(driveBase);


        XController = RobotAutoConstants.HomingConstants.XY_PID.createPIDController();
        YController = RobotAutoConstants.HomingConstants.XY_PID.createPIDController();
        ThetaController = RobotAutoConstants.HomingConstants.THETA_PID.createPIDController();

        XController.setIZone(1);
        YController.setIZone(1);
        ThetaController.setIZone(0.5);

        ThetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setTargetPose(ReefLocation targetPose) {
        this.targetReef = targetPose;

        Logger.recordOutput("home to reef/target Pose", targetPose);
    }

    @Override
    public void initialize() {
        XController.setSetpoint(targetReef.getPose().getX());
        YController.setSetpoint(targetReef.getPose().getY());
        ThetaController.setSetpoint(targetReef.getPose().getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d robotPose = driveBase.getPose();

        double xOutput = XController.calculate(robotPose.getX(), targetReef.getPose().getX());
        double yOutput = YController.calculate(robotPose.getY(), targetReef.getPose().getY());

        double thetaOutput =
            ThetaController.calculate(robotPose.getRotation().getRadians(), targetReef.getPose().getRotation().getRadians());

        double maxOutput = RobotAutoConstants.HomingConstants.MAX_HOME_SPEED_METERS_PER_SECOND;
        xOutput = MathUtil.clamp(xOutput, -maxOutput, maxOutput);
        yOutput = MathUtil.clamp(yOutput, -maxOutput, maxOutput);

        maxOutput = RobotAutoConstants.HomingConstants.MAX_HOME_SPEED_RADIANS_PER_SECOND;
        thetaOutput = MathUtil.clamp(thetaOutput, -maxOutput, maxOutput);

        double XY_DEADBAND = RobotAutoConstants.HomingConstants.XY_DEADBAND;
        double THETA_DEADBAND = RobotAutoConstants.HomingConstants.THETA_DEADBAND;

        Logger.recordOutput("home to reef/x output", xOutput);
        Logger.recordOutput("home to reef/y output", yOutput);
        Logger.recordOutput("home to reef/theta output", thetaOutput);

        if(Math.abs(xOutput) <= XY_DEADBAND) xOutput = 0;
        if(Math.abs(yOutput) <= XY_DEADBAND) yOutput = 0;
        if(Math.abs(thetaOutput) <= THETA_DEADBAND) thetaOutput = 0;

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(
                xOutput,
                yOutput,
                thetaOutput);

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
    }
}
