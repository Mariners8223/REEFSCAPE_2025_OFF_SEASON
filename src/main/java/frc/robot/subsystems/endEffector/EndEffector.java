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
    
    public void setDutyCycleL234(double dutyCycle){
        io.setMotorDutyCycleLeft(dutyCycle);
        io.setMotorDutyCycleRight(dutyCycle*-1);
    }

    public void setDutyCycleL1(double dutyCycle){
        io.setMotorDutyCycleLeft(dutyCycle*-1);
        io.setMotorDutyCycleRight(dutyCycle*-0.4);
    }

    public void motorStop(){
        io.setMotorDutyCycleLeft(0);
        io.setMotorDutyCycleRight(0);
    }

}
