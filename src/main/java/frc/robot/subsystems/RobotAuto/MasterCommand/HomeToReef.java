package frc.robot.subsystems.RobotAuto.MasterCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.DriveTrain.DriveBaseConstants;


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
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
