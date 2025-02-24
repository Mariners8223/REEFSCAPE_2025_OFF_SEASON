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

    private static final PIDController XController = RobotAutoConstants.HomingConstants.XY_PID.createPIDController();
    private static final PIDController YController = RobotAutoConstants.HomingConstants.XY_PID.createPIDController();
    private static final PIDController ThetaController = RobotAutoConstants.HomingConstants.THETA_PID.createPIDController();

    private int timer = 0;

    public HomeToReef(DriveBase driveBase, ReefLocation targetReef) {
        this.driveBase = driveBase;
        this.targetReef = targetReef;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(driveBase);

        ThetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setTargetPose(ReefLocation targetPose) {
        this.targetReef = targetPose;

        Logger.recordOutput("home to reef/target Pose", targetPose);
    }

    public static void pidTune(){
        SmartDashboard.putData(XController);
        SmartDashboard.putData(YController);
        SmartDashboard.putData(ThetaController);
    }

    @Override
    public void initialize() {
        XController.setSetpoint(targetReef.getPose().getX());
        YController.setSetpoint(targetReef.getPose().getY());
        ThetaController.setSetpoint(targetReef.getPose().getRotation().getRadians());

        Logger.recordOutput("home to reef/target X", XController.getSetpoint());
        Logger.recordOutput("home to reef/target Y", YController.getSetpoint());
        Logger.recordOutput("home to reef/target Theta", ThetaController.getSetpoint());

        timer = 0;
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
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, driveBase.getPose().getRotation());

        driveBase.drive(robotRelativeSpeeds);
    }

    @Override
    public boolean isFinished() {
        double xError = XController.getError();
        double yError = YController.getError();
        double thetaError = ThetaController.getError();

        Logger.recordOutput("home to reef/x error", xError);
        Logger.recordOutput("home to reef/y error", yError);
        Logger.recordOutput("home to reef/theta error", thetaError);

        if(XController.atSetpoint() && YController.atSetpoint() && ThetaController.atSetpoint()){
            timer ++;
        }
        else{
            timer = 0;
        }

        return timer >= 10;
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(new ChassisSpeeds());
    }
}
