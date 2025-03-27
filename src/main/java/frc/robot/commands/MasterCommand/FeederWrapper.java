// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MasterCommand;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.FeederLocation;
import frc.robot.Constants.FeederSide;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.RobotAuto.RobotAuto;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeederWrapper extends Command {
  private Command pathCommand;

  private final DriveBase driveBase;
  private final LED led;
  private final Supplier<FeederSide> feederSideSupplier;
  private final FeederLocation feederLocation;
  /** Creates a new FeederWrapper. */
  public FeederWrapper(DriveBase driveBase, RobotAuto robotAuto, LED led, FeederLocation feederLocation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveBase = driveBase;
    this.led = led;
    this.feederSideSupplier = robotAuto::getFeederSide;
    this.feederLocation = feederLocation;

    pathCommand = new InstantCommand();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    FeederSide side = feederSideSupplier.get();

    pathCommand = driveBase.pathFindToPathAndFollow(feederLocation.getPath(side));
    led.blinkWithRSLCommand(side.color);

    pathCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.end(interrupted);

    led.putDefaultPattern();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }

  public Set<Subsystem> getRequirements() {
    return pathCommand.getRequirements();
  }

  public boolean runsWhenDisabled() {
    return pathCommand.runsWhenDisabled();
  }

  public Command.InterruptionBehavior getInterruptionBehavior() {
    return pathCommand.getInterruptionBehavior();
  }
}
