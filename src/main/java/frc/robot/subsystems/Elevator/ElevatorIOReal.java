package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersController.ControllerLocation;

public class ElevatorIOReal implements ElevatorIO{
  
    public final MarinersSparkBase LeadMotor;
    public final MarinersSparkBase FollowMotor;

    public ElevatorIOReal(){
        this.LeadMotor =  ConfigureLeadMotor();
        this.FollowMotor =  ConfigureFollowMotor();
    }

    public MarinersSparkBase ConfigureLeadMotor(){
        
    }
    public MarinersSparkBase ConfigureFollowMotor(){

    }
    public void GetToDesiredHeight(double Height){
        
    }

    public void GetToBottom(){
        
    }

    public void SetElevatorSpeed(double speed){}

    public void PauseElevatorMotor(boolean pause){}




}
