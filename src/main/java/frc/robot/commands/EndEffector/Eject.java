package frc.robot.commands.EndEffector;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;

import java.util.function.Supplier;

public class Eject extends Command {
    private final EndEffector endEffector;
    private double startTime;

    private final Supplier<EndEffectorConstants.MotorPower> motorPowerSupplier;
    private EndEffectorConstants.MotorPower motorPower;

    public Eject(EndEffector endEffector, Supplier<EndEffectorConstants.MotorPower> motorPower) {
        this.endEffector = endEffector;

        this.motorPowerSupplier = motorPower;

        addRequirements(endEffector);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        motorPower = this.motorPowerSupplier.get();

        endEffector.setLeftMotorPower(motorPower.leftMotorPower);
        endEffector.setRightMotorPower(motorPower.rightMotorPower);

        startTime = RobotController.getMeasureTime().in(Units.Seconds);
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return RobotController.getMeasureTime().in(Units.Seconds) - startTime >= motorPower.ejectTime;
    }
}
