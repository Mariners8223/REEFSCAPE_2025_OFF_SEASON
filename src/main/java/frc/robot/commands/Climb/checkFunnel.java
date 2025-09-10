package main.java.frc.robot.commands.Climb;

public class checkFunnel extends Command {

    public DriveCommand() {
        private final Climb climb;

        /**
         * Creates a new Climb.
         */
        public ClimbCommand(Climb climb) {
            this.climb = climb;
    }
    

    @Override
    public void initialize() {
        climb.readMotor();
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
