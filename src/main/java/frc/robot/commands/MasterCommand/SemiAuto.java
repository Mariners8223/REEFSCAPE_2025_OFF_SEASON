package frc.robot.commands.MasterCommand;

import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.Drive.RobotRelativeDrive;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.Elevator.MoveToLevelActive;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.LED.LED;

import java.util.Set;
import java.util.function.Supplier;


public class SemiAuto extends Command {
   private final Command sequence;

   private final MoveToLevel moveElevatorCommand;
   private final LED led;

   private final Supplier<ElevatorConstants.ElevatorLevel> elevatorLevelSupplier;

    public SemiAuto(DriveBase driveBase, Elevator elevator, Supplier<Constants.ReefLocation> targetReefSupplier,
                    Supplier<ElevatorConstants.ElevatorLevel> elevatorLevelSupplier, EventTrigger moveElevatorMarker,
                    CommandXboxController controller, LED led) {
       RobotToReef robotToReef = new RobotToReef(driveBase, targetReefSupplier, led);

       RobotRelativeDrive robotRelativeDrive = new RobotRelativeDrive(driveBase, controller);


       this.elevatorLevelSupplier = elevatorLevelSupplier;

       WaitUntilCommand waitUntilMarker = new WaitUntilCommand(moveElevatorMarker);

       moveElevatorCommand = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom);

       MoveToLevelActive moveElevatorCommandActive = new MoveToLevelActive(elevator, elevatorLevelSupplier);

       this.led = led;

       sequence = new SequentialCommandGroup(
               new ParallelCommandGroup(
                       robotToReef,
                          new SequentialCommandGroup(
                                 waitUntilMarker,
                                 moveElevatorCommand
                          )
               ),
               new ParallelCommandGroup(
                       robotRelativeDrive,
                       moveElevatorCommandActive
               )
       );
    }

    @Override
    public void initialize() {
        ElevatorConstants.ElevatorLevel level = elevatorLevelSupplier.get();

        moveElevatorCommand.changeDesiredlevel(level);
        led.blinkWithRSL(Color.kOrangeRed);

        sequence.initialize();
    }

    @Override
    public void execute() {
        sequence.execute();
    }

    @Override
    public boolean isFinished() {
        return sequence.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        sequence.end(interrupted);
        led.putDefaultPattern();
    }

    public Set<Subsystem> getRequirements() {
        return this.sequence.getRequirements();
    }

    public boolean runsWhenDisabled() {
        return this.sequence.runsWhenDisabled();
    }

    public Command.InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }


}
