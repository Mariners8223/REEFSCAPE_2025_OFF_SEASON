package frc.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ToggleTrigger {
    public ToggleTrigger(Trigger trigger, Command command) {

        Runnable action = () -> {
            if(command.isScheduled()){
                command.cancel();
            } else {
                command.schedule();
            }
        };

        trigger.onTrue(new InstantCommand(action));
    }
}
