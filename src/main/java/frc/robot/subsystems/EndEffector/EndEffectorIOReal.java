package frc.robot.subsystems.EndEffector;
import edu.wpi.first.wpilibj.SensorUtil;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;

public class EndEffectorIOReal implements EndEffectorIO{
    private final VictorSPX rightMotor;
    private final VictorSPX leftMotor;
    private final DigitalInput beamBreak;

    public EndEffectorIOReal() {
        this.rightMotor = configerRightMotor();
        this.leftMotor = configerLeftMotor();
        this.beamBreak = new DigitalInput(EndEffectorConstants.beamBreakPort);
    

    
        
    }
private VictorSPX configerRightMotor(){
    VictorSPX motor;
    motor = new VictorSPX(EndEffectorConstants.motorRightID);
    motor.setNeutralMode(NeutralMode.Brake);
    return motor;
}
private VictorSPX configerLeftMotor(){
    VictorSPX motor;
    motor = new VictorSPX(EndEffectorConstants.motorLeftID);
    motor.setNeutralMode(NeutralMode.Brake);
    return motor;
}
@Override
public void setMotorDutyCycleRight(double RightMotorDutyCycle){
    rightMotor.set(VictorSPXControlMode.PercentOutput, RightMotorDutyCycle);
}
@Override
public void setMotorDutyCycleLeft(double LeftMotorDutyCycle){
    leftMotor.set(VictorSPXControlMode.PercentOutput, LeftMotorDutyCycle);
}
@Override
public void stopMotor(){
    rightMotor.set(VictorSPXControlMode.PercentOutput,0);
    leftMotor.set(VictorSPXControlMode.PercentOutput,0);
}
@Override
public void brakeMotor(){
    rightMotor.setNeutralMode(NeutralMode.Brake);
    leftMotor.setNeutralMode(NeutralMode.Brake);
}
@Override
public  boolean getBeamBreak(){
    return !beamBreak.get();
}
public void update(EndEffectorInputs inputs){
    inputs.MotorPowerLeft = leftMotor.getMotorOutputPercent();
    inputs.MotorPowerRight = rightMotor.getMotorOutputPercent();
    inputs.BeamBreak = !beamBreak.get();
}
}
