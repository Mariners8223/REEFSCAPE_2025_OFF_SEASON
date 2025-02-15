package frc.robot.commands.MasterCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ReefLocation;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.RobotAuto.RobotAutoConstants;
import org.littletonrobotics.junction.Logger;


public class HomeToReefEndless extends Command {
    private final DriveBase driveBase;
    private ReefLocation targetReef;

    private final PIDController XController;
    private final PIDController YController;
    private final PIDController ThetaController;

    public HomeToReefEndless(DriveBase driveBase, ReefLocation targetReef) {
        this.driveBase = driveBase;
        this.targetReef = targetReef;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(driveBase);


        XController = RobotAutoConstants.HomingConstants.XY_PID.createPIDController();
        YController = RobotAutoConstants.HomingConstants.XY_PID.createPIDController();
        ThetaController = RobotAutoConstants.HomingConstants.THETA_PID.createPIDController();

        XController.setIZone(0.5);
        YController.setIZone(0.5);
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

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(
                xOutput >= 0.1 ? XY_DEADBAND : 0,
                yOutput >= 0.1 ? XY_DEADBAND : 0,
                thetaOutput >= 0.1 ? THETA_DEADBAND : 0);

        ChassisSpeeds robotRelativeSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, driveBase.getRotation2d());

        driveBase.drive(robotRelativeSpeeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(new ChassisSpeeds());
    }
}
