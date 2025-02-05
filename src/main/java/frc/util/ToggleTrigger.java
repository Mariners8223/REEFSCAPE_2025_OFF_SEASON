package frc.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ToggleTrigger {
    private boolean lastState = false;

    public ToggleTrigger(Trigger trigger, Command command) {
        Runnable action = () -> {
            if (lastState) {
                command.cancel();
                lastState = false;
            } else {
                command.schedule();
                lastState = true;
            }
        };

        trigger.onTrue(new InstantCommand(action));
    }
}
