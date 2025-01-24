// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

public class EndEffectorIOReal implements EndEffectorIO {
    private final VictorSPX RightMotor;
    private final VictorSPX LeftMotor;
    private final DigitalInput beamBreak;


    public EndEffectorIOReal(){
        RightMotor = configueMotor(EndEffectorConstants.rightID);
        LeftMotor = configueMotor(EndEffectorConstants.leftID);
        beamBreak = new DigitalInput(EndEffectorConstants.beamBreakPort);
    }
    
    private VictorSPX configueMotor(int ID){
        VictorSPX motor = new VictorSPX(ID);
        return motor;
    }


    public void setRightMotorPower(double PowerToSet){
        double realPower = MathUtil.clamp(PowerToSet,-EndEffectorConstants.MotorPower.maxMotorPower,
        EndEffectorConstants.MotorPower.maxMotorPower);

        RightMotor.set(VictorSPXControlMode.PercentOutput,realPower);
    }

    public void setLeftMotorPower(double PowerToSet){
        double realPower = MathUtil.clamp(PowerToSet,-EndEffectorConstants.MotorPower.maxMotorPower,
        EndEffectorConstants.MotorPower.maxMotorPower);

        LeftMotor.set(VictorSPXControlMode.PercentOutput,realPower);
    }

    public void Update(EndEffectorInputs inputs){
        inputs.rightPower = RightMotor.getMotorOutputPercent();
        inputs.leftPower = LeftMotor.getMotorOutputPercent();
        inputs.beamBreakValue = EndEffectorConstants.beamBreakInverted ? !beamBreak.get() : beamBreak.get();
    }

}
