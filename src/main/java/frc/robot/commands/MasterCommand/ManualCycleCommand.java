package frc.robot.commands.MasterCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.EndEffector.Eject;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;

import java.util.Set;
import java.util.function.Supplier;


public class ManualCycleCommand extends Command {
    private final MoveToLevel moveToSelectedLevel;
    private final Eject eject;

    private final SequentialCommandGroup command;

    private final Supplier<ElevatorConstants.ElevatorLevel> levelSupplier;

    public ManualCycleCommand(EndEffector endEffector, Elevator elevator,
                              Supplier<ElevatorConstants.ElevatorLevel> levelSupplier) {


        moveToSelectedLevel = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.L1);
        eject = new Eject(endEffector, EndEffectorConstants.MotorPower.L1);

        MoveToLevel moveToBottom = new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.Bottom);

        this.levelSupplier = levelSupplier;

        command = new SequentialCommandGroup(
                moveToSelectedLevel,
                eject,
                moveToBottom
        );
    }

    @Override
    public void initialize() {
        moveToSelectedLevel.changeDesiredlevel(levelSupplier.get());
        eject.setLevel(MasterCommand.getMotorPower(levelSupplier.get()));
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    public Set<Subsystem> getRequirements() {
        return this.command.getRequirements();
    }

    public boolean runsWhenDisabled() {
        return this.command.runsWhenDisabled();
    }

    public Command.InterruptionBehavior getInterruptionBehavior() {
        return this.command.getInterruptionBehavior();
    }
}
