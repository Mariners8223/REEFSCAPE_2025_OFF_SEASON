package frc.robot.commands.MasterCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator.ElevatorConstants;
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
    private ElevatorConstants.ElevatorLevel desiredLevel;

    private PIDController XController;
    private PIDController YController;
    private PIDController ThetaController;

    private int timer = 0;

    public HomeToReef(DriveBase driveBase, ReefLocation targetReef, ElevatorConstants.ElevatorLevel desiredLevel) {
        this.driveBase = driveBase;
        this.targetReef = targetReef;
        this.desiredLevel = desiredLevel;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(driveBase);
    }

    public void setTargetPose(ReefLocation targetPose){
        this.targetReef = targetPose;
        this.desiredLevel = ElevatorConstants.ElevatorLevel.Bottom;

        Logger.recordOutput("home to reef/target Pose", targetPose);
        Logger.recordOutput("home to reef/target Level", desiredLevel);
    }

    public void setTargetPose(ReefLocation targetPose, ElevatorConstants.ElevatorLevel targetLevel){
        this.targetReef = targetPose;
        this.desiredLevel = targetLevel;

        Logger.recordOutput("home to reef/target Pose", targetPose);
        Logger.recordOutput("home to reef/target Level", targetLevel);
    }

    public static void pidTune(){
        for(ElevatorConstants.ElevatorLevel level : ElevatorConstants.ElevatorLevel.values()){
            PIDController XController = RobotAutoConstants.XY_PID_CONSTANTS.get(level).getXController();
            PIDController YController = RobotAutoConstants.XY_PID_CONSTANTS.get(level).getYController();
            PIDController ThetaController = RobotAutoConstants.THETA_PID_CONSTANTS.get(level).getThetaController();

            SmartDashboard.putData("X PID " + level, XController);
            SmartDashboard.putData("Y PID " + level, YController);
            SmartDashboard.putData("Theta PID " + level, ThetaController);
        }
    }

    @Override
    public void initialize() {
        XController = RobotAutoConstants.XY_PID_CONSTANTS.get(desiredLevel).getXController();
        YController = RobotAutoConstants.XY_PID_CONSTANTS.get(desiredLevel).getYController();
        ThetaController = RobotAutoConstants.THETA_PID_CONSTANTS.get(desiredLevel).getThetaController();

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

        double upperLimitXY = RobotAutoConstants.UPPER_SPEED_LIMIT_XY;
        double lowerLimitXY = RobotAutoConstants.LOWER_SPEED_LIMIT_XY;

        double upperLimitTheta = RobotAutoConstants.UPPER_SPEED_LIMIT_THETA;
        double lowerLimitTheta = RobotAutoConstants.LOWER_SPEED_LIMIT_THETA;

        double maxXOutput = getClampValue(XController.getError(), upperLimitXY, lowerLimitXY);
        double maxYOutput = getClampValue(YController.getError(), upperLimitXY, lowerLimitXY);

        double maxThetaOutput = getClampValue(ThetaController.getError(), upperLimitTheta, lowerLimitTheta);

        xOutput = MathUtil.clamp(xOutput, -maxXOutput, maxXOutput);
        yOutput = MathUtil.clamp(yOutput, -maxYOutput, maxYOutput);

        thetaOutput = MathUtil.clamp(thetaOutput, -maxThetaOutput, maxThetaOutput);

        double XY_DEADBAND = RobotAutoConstants.XY_DEADBAND;
        double THETA_DEADBAND = RobotAutoConstants.THETA_DEADBAND;

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

    private double getClampValue(double error, double upperLimit, double lowerLimit){
        double value = Math.abs(error) * upperLimit;

        return Math.max(value, lowerLimit);
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
