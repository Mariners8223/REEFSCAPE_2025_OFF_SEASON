package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.endEffector.EndEffectorConstents.*;

public class EndEffector extends SubsystemBase{
    EndEffectorIO io;
    EndEffectorInputsAutoLog inputs = new EndEffectorInputsAutoLog();

    public EndEffector(){
        io = new EndEffectorIOReal(); 
            
    };
    
    public void setDutyCycleL234(double EndEffectorConstents.EndEffector.dutyCycle){
        io.setMotorDutyCycleLeft(EndEffectorConstents.EndEffector.dutyCycle);
        io.setMotorDutyCycleRight(EndEffectorConstents.EndEffector.dutyCycle*-1);
    }

    public void setDutyCycleL1(double EndEffectorConstents.EndEffector.DUTY_CYCLE){
        io.setMotorDutyCycleLeft(EndEffectorConstents.EndEffector.DUTY_CYCLE*-1);
        io.setMotorDutyCycleRight(EndEffectorConstents.EndEffector.DUTY_CYCLE*-0.4);
    }

    public void motorStop(){
        io.setMotorDutyCycleLeft(0);
        io.setMotorDutyCycleRight(0);
    }

}
