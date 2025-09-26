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
        
    }

    public MarinersSparkBase ConfigureLeadMotor(){
        MarinersSparkBase motor = new MarinersSparkBase("LeadMotor", ControllerLocation.RIO, ElevatorConstants.LeadMotor.MOTOR_ID,ElevatorConstants.LeadMotor.IS_BRUSHLESS , ElevatorConstants.LeadMotor.MOTOR_TYPE);
        motor.setProfile(ElevatorConstants.LeadMotor.TRAPEZOID_PROFILE);
    }
    public MarinersSparkBase ConfigureFollowMotor(){
        MarinersSparkBase motor = new MarinersSparkBase("follow motor", ControllerLocation.RIO, ElevatorConstants.FollowMotor.MOTOR_ID, ElevatorConstants.FollowMotor.IS_BRUSHLESS,ElevatorConstants.FollowMotor.MOTOR_TYPE );
        motor.setMotorAsFollower(LeadMotor, true);
        
    }
    public void SetToDesiredHeight(double Height){
        LeadMotor.setReference(Height, frc.util.MarinersController.MarinersController.ControlMode.ProfiledPosition);  
    }

    public void PauseElevatorMotor(boolean pause){
        LeadMotor.setMotorIdleMode(pause);
    }

    public void update(ElevatorInputsAutoLogged inputs){
        inputs.height = LeadMotor.
    }


}