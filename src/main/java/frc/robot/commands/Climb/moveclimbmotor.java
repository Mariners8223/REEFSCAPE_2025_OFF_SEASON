package frc.robot.commands.Climb;

public class DriveCommand extends Command {

    public DriveCommand() {
    }

    @Override
    public void initialize() {
        setMotorDutyCycle(CLIMB_MOTOR_HIGH)
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean isInterrupted){
    }

    @Override
    public boolean isFinished(){
    }
}
