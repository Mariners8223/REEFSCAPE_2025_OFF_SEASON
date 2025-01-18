
package frc.robot.commands.EndEffector;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;
import frc.robot.subsystems.EndEffector.EndEffectorConstants.MotorPower;


public class EjectL4 extends Command {
  private final EndEffector endEffector;
  private double startTime;

  public EjectL4(EndEffector endEffector) {
    this.endEffector = endEffector;

    addRequirements(endEffector);
  }

  
  @Override
  public void initialize() {
    endEffector.setLeftMotorPower(EndEffectorConstants.MotorPower.L4.leftMotorPower);
    endEffector.setRightMotorPower(EndEffectorConstants.MotorPower.L4.rightMotorPower);

    startTime = RobotController.getMeasureTime().in(Units.Seconds);
  }

  @Override
  public void end(boolean interrupted) {
    endEffector.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return RobotController.getMeasureTime().in(Units.Seconds) - startTime >= MotorPower.L4.ejectTime;
  }
}