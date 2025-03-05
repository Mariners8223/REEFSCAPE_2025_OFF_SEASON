package frc.robot.commands.MasterCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

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

    private PIDController TranslationController;
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
        Logger.recordOutput("home to reef/target Pose 2d", targetPose.getPose());
    }

    public void setTargetPose(ReefLocation targetPose, ElevatorConstants.ElevatorLevel targetLevel){
        this.targetReef = targetPose;
        this.desiredLevel = targetLevel;

        Logger.recordOutput("home to reef/target Pose", targetPose);
        Logger.recordOutput("home to reef/target Level", targetLevel);
        Logger.recordOutput("home to reef/target Pose 2d", targetPose.getPose());
    }

    public static void pidTune(){
        for(ElevatorConstants.ElevatorLevel level : ElevatorConstants.ElevatorLevel.values()){
            PIDController XController = RobotAutoConstants.XY_PID_CONSTANTS.get(level).getTranslationController();
            PIDController ThetaController = RobotAutoConstants.THETA_PID_CONSTANTS.get(level).getThetaController();

            SmartDashboard.putData("X PID " + level, XController);
            SmartDashboard.putData("Theta PID " + level, ThetaController);
        }
    }

    @Override
    public void initialize() {
        TranslationController = RobotAutoConstants.XY_PID_CONSTANTS.get(desiredLevel).getTranslationController();
        ThetaController = RobotAutoConstants.THETA_PID_CONSTANTS.get(desiredLevel).getThetaController();

        TranslationController.reset();
        ThetaController.reset();

        ThetaController.setSetpoint(targetReef.getPose().getRotation().getRadians());

        Logger.recordOutput("home to reef/target X", TranslationController.getSetpoint());
        Logger.recordOutput("home to reef/target Theta", ThetaController.getSetpoint());

        timer = 0;
    }

    @Override
    public void execute() {
        Pose2d robotPose = driveBase.getPose();

        double distance = robotPose.getTranslation().getDistance(targetReef.getPose().getTranslation());

        double scalar = TranslationController.calculate(distance);

        double xOutput = scalar * (robotPose.getX() - targetReef.getPose().getX());

        double yOutput = scalar * (robotPose.getY() - targetReef.getPose().getY());

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

        if(Math.abs(robotRelativeSpeeds.vxMetersPerSecond) <= XY_DEADBAND) robotRelativeSpeeds.vxMetersPerSecond = 0;
        if(Math.abs(robotRelativeSpeeds.vyMetersPerSecond) <= XY_DEADBAND) robotRelativeSpeeds.vyMetersPerSecond = 0;
        if(Math.abs(robotRelativeSpeeds.omegaRadiansPerSecond) <= THETA_DEADBAND) robotRelativeSpeeds.omegaRadiansPerSecond = 0;

        driveBase.drive(robotRelativeSpeeds);
    }

    @Override
    public boolean isFinished() {
        double xError = TranslationController.getError();
        double thetaError = ThetaController.getError();

        Logger.recordOutput("home to reef/x error", xError);
        Logger.recordOutput("home to reef/theta error", thetaError);

        if(TranslationController.atSetpoint() && ThetaController.atSetpoint()){
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
