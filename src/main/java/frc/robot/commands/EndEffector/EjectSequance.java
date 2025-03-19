package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.EndEffector.Intake.Intake;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;

import java.util.Set;

public class EjectSequance extends Command {
    private final Eject ejectCommand;
    private final ParallelDeadlineGroup group;

    public EjectSequance(EndEffector endEffector, Elevator elevator) {
        ejectCommand = new Eject(endEffector, EndEffectorConstants.MotorPower.L2_3);

        group = new ParallelDeadlineGroup(
                ejectCommand,
                new SequentialCommandGroup(
                        // new WaitCommand(EndEffectorConstants.MotorPower.L4.ejectTime / 4),
                        new MoveToLevel(elevator, ElevatorConstants.ElevatorLevel.L4_EXTRA)
                ).onlyIf(() -> ejectCommand.getLevel() == EndEffectorConstants.MotorPower.L4)
        );

    }

    public void setLevel(EndEffectorConstants.MotorPower motorPower){
        ejectCommand.setLevel(motorPower);
    }

    @Override
    public void initialize() {
        group.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        group.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        group.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return group.isFinished();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return group.getRequirements();
    }

    @Override
    public boolean runsWhenDisabled() {
        return group.runsWhenDisabled();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return group.getInterruptionBehavior();
    }


}
