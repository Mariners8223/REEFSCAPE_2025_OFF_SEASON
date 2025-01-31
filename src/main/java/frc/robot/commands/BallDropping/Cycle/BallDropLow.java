package frc.robot.commands.BallDropping.Cycle;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BallDropping.BallDropOff;
import frc.robot.commands.BallDropping.BallDropOnForLow;
import frc.robot.subsystems.BallDropping.BallDropping;

public class BallDropLow extends SequentialCommandGroup {
    public BallDropLow(BallDropping ballDrop) {
        addCommands(
                new BallDropOnForLow(ballDrop),
                new WaitCommand(0.5),
                new BallDropOff(ballDrop)
        );
    }
}
