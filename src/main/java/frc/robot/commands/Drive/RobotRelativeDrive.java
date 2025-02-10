package frc.robot.commands.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain.DriveBase;

import static frc.robot.commands.Drive.DriveCommand.deadBand;

public class RobotRelativeDrive extends Command {
    private final DriveBase driveBase;
    private final CommandXboxController controller;

    private static final double MAX_FREE_WHEEL_SPEED = 2;

    public RobotRelativeDrive(DriveBase driveBase, CommandXboxController controller) {
        this.driveBase = driveBase;
        this.controller = controller;
        addRequirements(driveBase);
    }

    @Override
    public void execute() {
        double R2Axis  = 1 - (0.5 + controller.getRightTriggerAxis() / 2);

        if(R2Axis <= 0.1) {
            R2Axis = 0.1;
        }

        //sets the value of the 3 vectors we need
        double leftX = -deadBand(controller.getLeftY());
        double leftY = -deadBand(controller.getLeftX());
        double rightX = -deadBand(controller.getRightX());

        leftX *= R2Axis * MAX_FREE_WHEEL_SPEED;
        leftY *= R2Axis * MAX_FREE_WHEEL_SPEED;
        rightX *= R2Axis * MAX_FREE_WHEEL_SPEED;

        driveBase.drive(new ChassisSpeeds(leftX, leftY, rightX));
    }
}
