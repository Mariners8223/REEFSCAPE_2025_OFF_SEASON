package frc.robot.commands.MasterCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    // private final Consumer<Double> distanceConsumer;

    private final PIDController XController = RobotAutoConstants.X_PID;
    private final PIDController YController = RobotAutoConstants.Y_PID;
    private final PIDController ThetaController = RobotAutoConstants.THETA_PID;

    private int timer = 0;

    public HomeToReef(DriveBase driveBase, ReefLocation targetReef) {
        this.driveBase = driveBase;
        this.targetReef = targetReef;
        // this.distanceConsumer = distanceConsumer;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(driveBase);
    }

    public void setTargetPose(ReefLocation targetPose){
        this.targetReef = targetPose;

        Logger.recordOutput("home to reef/target Pose", targetPose);
        Logger.recordOutput("home to reef/target Pose 2d", targetPose.getPose());
    }

    public static void pidTune(){
        SmartDashboard.putData("X PID", RobotAutoConstants.X_PID);
        SmartDashboard.putData("Y PID", RobotAutoConstants.Y_PID);
        SmartDashboard.putData("Theta PID", RobotAutoConstants.THETA_PID);
    }

    @Override
    public void initialize() {
        XController.reset();
        YController.reset();
        ThetaController.reset();

        ThetaController.setSetpoint(targetReef.getPose().getRotation().getRadians());
        XController.setSetpoint(targetReef.getPose().getX());
        YController.setSetpoint(targetReef.getPose().getY());

        Pose2d currentPose = driveBase.getPose();

        XController.calculate(currentPose.getX(), targetReef.getPose().getX());
        YController.calculate(currentPose.getY(), targetReef.getPose().getY());
        ThetaController.calculate(currentPose.getRotation().getRadians(), targetReef.getPose().getRotation().getRadians());

        Logger.recordOutput("home to reef/target Theta", ThetaController.getSetpoint());
        Logger.recordOutput("home to reef/target X", XController.getSetpoint());
        Logger.recordOutput("home to reef/target Y", YController.getSetpoint());

        timer = 0;

        driveBase.drive(new ChassisSpeeds());
    }

    public boolean isOutOfTolarance(){
        Pose2d currentPose = driveBase.getPose();

        return Math.abs(targetReef.getPose().getX() - currentPose.getX()) > XController.getErrorTolerance() ||
            Math.abs(targetReef.getPose().getY() - currentPose.getY()) > YController.getErrorTolerance() ||
            Math.abs(targetReef.getPose().getRotation().getRadians() - currentPose.getRotation().getRadians())
            > ThetaController.getErrorTolerance();
    }

    @Override
    public void execute() {
        Pose2d robotPose = driveBase.getPose();

        double distance = robotPose.getTranslation().getDistance(targetReef.getPose().getTranslation());
        // distanceConsumer.accept(distance);

        double xOutput = XController.calculate(robotPose.getX());
        double yOutput = YController.calculate(robotPose.getY());

        Logger.recordOutput("home to reef/ distance", distance);

        Logger.recordOutput("home to reef/x Error", XController.getError());
        Logger.recordOutput("home to reef/y Error", YController.getError());

        double thetaOutput =
            ThetaController.calculate(robotPose.getRotation().getRadians(), targetReef.getPose().getRotation().getRadians());

        double XY_DEADBAND = RobotAutoConstants.XY_DEADBAND;
        double THETA_DEADBAND = RobotAutoConstants.THETA_DEADBAND;

        Logger.recordOutput("home to reef/x output", xOutput);
        Logger.recordOutput("home to reef/y output", yOutput);
        Logger.recordOutput("home to reef/theta output", thetaOutput);

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(
                xOutput,
                yOutput,
                thetaOutput);

        ChassisSpeeds robotRelativeSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, driveBase.getPose().getRotation());

        double upperLimitXY = RobotAutoConstants.UPPER_SPEED_LIMIT_XY;
        double lowerLimitXY = RobotAutoConstants.LOWER_SPEED_LIMIT_XY;

        double upperLimitTheta = RobotAutoConstants.UPPER_SPEED_LIMIT_THETA;
        double lowerLimitTheta = RobotAutoConstants.LOWER_SPEED_LIMIT_THETA;

        double maxXOutput = getClampValue(XController.getError(), upperLimitXY, lowerLimitXY);
        double maxYOutput = getClampValue(YController.getError(), upperLimitXY, lowerLimitXY);

        double maxThetaOutput = getClampValue(ThetaController.getError(), upperLimitTheta, lowerLimitTheta);

        robotRelativeSpeeds.vxMetersPerSecond = MathUtil.clamp(robotRelativeSpeeds.vxMetersPerSecond, -maxXOutput, maxXOutput);
        robotRelativeSpeeds.vyMetersPerSecond = MathUtil.clamp(robotRelativeSpeeds.vyMetersPerSecond, -maxYOutput, maxYOutput);
        robotRelativeSpeeds.omegaRadiansPerSecond = MathUtil.clamp(robotRelativeSpeeds.omegaRadiansPerSecond, -maxThetaOutput, maxThetaOutput);

        if(Math.abs(robotRelativeSpeeds.vxMetersPerSecond) <= XY_DEADBAND) robotRelativeSpeeds.vxMetersPerSecond = 0;
        if(Math.abs(robotRelativeSpeeds.vyMetersPerSecond) <= XY_DEADBAND) robotRelativeSpeeds.vyMetersPerSecond = 0;
        if(Math.abs(robotRelativeSpeeds.omegaRadiansPerSecond) <= THETA_DEADBAND) robotRelativeSpeeds.omegaRadiansPerSecond = 0;

        driveBase.drive(robotRelativeSpeeds);
    }

    private double getClampValue(double error, double upperLimit, double lowerLimit){
        double value = Math.abs(error) * upperLimit;

        return Math.max(value, lowerLimit);
    }

    @Override
    public boolean isFinished() {
        double thetaError = ThetaController.getError();

        Logger.recordOutput("home to reef/theta error", thetaError);

        if(XController.atSetpoint() && YController.atSetpoint() && ThetaController.atSetpoint()){
            timer ++;
        }
        else{
            timer = 0;
        }

        return timer >= 20;
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(new ChassisSpeeds());
    }
}
