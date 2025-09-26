package frc.robot.subsystems.Elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
        FollowMotor.setMotorAsFollower(LeadMotor, true);
    }

    public MarinersSparkBase ConfigureLeadMotor(){
        MarinersSparkBase motor = new MarinersSparkBase("LeadMotor", ControllerLocation.RIO, ElevatorConstants.LeadMotor.MOTOR_ID,ElevatorConstants.LeadMotor.IS_BRUSHLESS , ElevatorConstants.LeadMotor.MOTOR_TYPE);
    }
    public MarinersSparkBase ConfigureFollowMotor(){
        MarinersSparkBase motor = new MarinersSparkBase("follow motor", ControllerLocation.RIO, ElevatorConstants.FollowMotor.MOTOR_ID, ElevatorConstants.FollowMotor.IS_BRUSHLESS,ElevatorConstants.FollowMotor.MOTOR_TYPE );
    }
    public void SetToDesiredHeight(double Height){
        LeadMotor.setReference(Height, controlMode);

        
    }



    public void SetElevatorSpeed(double speed){}

    public void PauseElevatorMotor(boolean pause){}




}
