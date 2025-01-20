package frc.robot.commands.EndEffector;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;

public class Eject extends Command {
    private final EndEffector endEffector;
    private double startTime;

    private EndEffectorConstants.MotorPower motorPower;

    public Eject(EndEffector endEffector, EndEffectorConstants.MotorPower motorPower) {
        this.endEffector = endEffector;

        this.motorPower = motorPower;

        addRequirements(endEffector);
    }

    public void setMotorPower(EndEffectorConstants.MotorPower motorPower) {
        this.motorPower = motorPower;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        endEffector.setLeftMotorPower(motorPower.leftMotorPower);
        endEffector.setRightMotorPower(motorPower.rightMotorPower);

        startTime = RobotController.getMeasureTime().in(Units.Seconds);
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.stopMotors();
        endEffector.setGpLoaded(false);    }

    @Override
    public boolean isFinished() {
        return RobotController.getMeasureTime().in(Units.Seconds) - startTime >= motorPower.ejectTime;
    }
}
