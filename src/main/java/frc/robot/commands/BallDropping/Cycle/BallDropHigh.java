package frc.robot.commands.BallDropping.Cycle;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BallDropping.BallDropOff;
import frc.robot.commands.BallDropping.BallDropOnForHigh;
import frc.robot.subsystems.BallDropping.BallDropping;

public class BallDropHigh extends SequentialCommandGroup {
    public BallDropHigh(BallDropping ballDrop) {
         addCommands(
                 new BallDropOnForHigh(ballDrop),
                 new WaitCommand(0.5),
                 new BallDropOff(ballDrop)
         );
    }
}
