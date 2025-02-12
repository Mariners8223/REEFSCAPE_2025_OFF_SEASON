package frc.robot.commands.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain.DriveBase;

import static frc.robot.commands.Drive.DriveCommand.deadBand;

public class RobotRelativeDrive extends Command {
    private final DriveBase driveBase;
    private final CommandXboxController controller;

    private static final double MAX_FREE_WHEEL_SPEED = 1.25;

    public RobotRelativeDrive(DriveBase driveBase, CommandXboxController controller) {
        this.driveBase = driveBase;
        this.controller = controller;
        addRequirements(driveBase);
    }

    @Override
    public void execute() {
        //sets the value of the 3 vectors we need
        double leftX = -deadBand(controller.getLeftY());
        double leftY = -deadBand(controller.getLeftX());
        double rightX = -deadBand(controller.getRightX());

        leftX *= MAX_FREE_WHEEL_SPEED;
        leftY *= MAX_FREE_WHEEL_SPEED;
        rightX *= MAX_FREE_WHEEL_SPEED;

        driveBase.drive(new ChassisSpeeds(leftX, leftY, rightX));
    }
}
