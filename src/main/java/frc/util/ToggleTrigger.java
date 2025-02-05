package frc.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ToggleTrigger {
    private boolean lastState = false;

    public ToggleTrigger(Trigger trigger, Command command) {

        Command fullCommand = command.andThen(new InstantCommand(() -> lastState = false));

        Runnable action = () -> {
            if (lastState) {
                fullCommand.cancel();
                lastState = false;
            } else {
                fullCommand.schedule();
                lastState = true;
            }
        };

        trigger.onTrue(new InstantCommand(action));
    }
}
