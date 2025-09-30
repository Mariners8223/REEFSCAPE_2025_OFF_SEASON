package frc.robot.commands.endEffector;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorConstents;

public class scoringL234FRFR extends Command {
    private final frc.robot.subsystems.endEffector.EndEffector endEffector;
    int counter;
    
    public scoringL234FRFR(EndEffector endEffector){
        this.endEffector = endEffector;
        
    }

    @Override
    public void initialize() {
        endEffector.setDutyCycleL234(EndEffectorConstents.DUTY_CYCLE);
        counter=0;
  

    }
    @Override
    public void execute() {
        counter++;

    }
    @Override
    public void end(boolean interrupted) {
        endEffector.motorStop();
        EndEffectorConstents.IsCoralInRobot=false;


    }
    @Override
    public boolean isFinished() {
        return counter>=50;
  

    }
}
