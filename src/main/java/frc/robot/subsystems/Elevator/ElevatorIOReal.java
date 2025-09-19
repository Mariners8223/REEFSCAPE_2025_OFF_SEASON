package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase.MotorType;
x

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
        MarinersSparkBase motor = new MarinersSparkBase("follow motor", ControllerLocation.RIO, 16, true, MotorType.SPARK_FLEX, );
    }
    public void GetToDesiredHeight(double Height){
        
    }

    public void GetToBottom(){
        
    }

    public void SetElevatorSpeed(double speed){}

    public void PauseElevatorMotor(boolean pause){}




}
