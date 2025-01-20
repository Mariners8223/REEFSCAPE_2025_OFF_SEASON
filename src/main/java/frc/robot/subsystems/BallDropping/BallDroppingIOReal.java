// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BallDropping;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersController.ControlMode;
import frc.util.MarinersController.MarinersSparkBase.MotorType;


public class BallDroppingIOReal implements BallDroppingIO{
    private final MarinersController angleMotor;
    private final VictorSPX dropperMotor;

    public BallDroppingIOReal(){
       angleMotor = configureAngleMotor();
       dropperMotor = configureDropperMotor();
    }

    private MarinersController configureAngleMotor(){
        MarinersController motor = new MarinersSparkBase("angle motor", BallDroppingConstants.AngleMotor.location, 
        BallDroppingConstants.AngleMotor.id, true, MotorType.SPARK_FLEX);

        motor.setPIDF(BallDroppingConstants.AngleMotor.AnglePID);
        motor.getMeasurements().setGearRatio(BallDroppingConstants.AngleMotor.gearRatio);
        
        return motor;
    }
    private VictorSPX configureDropperMotor(){
        return new VictorSPX(BallDroppingConstants.DropperMotor.id);
    }


    //angle motor implement
    public void resetAngleEncoder(){
        angleMotor.resetMotorEncoder();
    }

    public void reachAngle(double angleToReach){
        angleMotor.setReference(angleToReach, ControlMode.Position);
    }

    //dropping motor implement
    public void setDropperMotorPower(double dropperPower){
        double realPower = MathUtil.clamp(dropperPower,-BallDroppingConstants.DropperMotor.maxDropperPower,
        BallDroppingConstants.DropperMotor.maxDropperPower);

        dropperMotor.set(VictorSPXControlMode.PercentOutput, realPower);
    }

    public void stopDropperMotor(){
        setDropperMotorPower(0);
    }

    public void Update(BallDroppingInputs inputs){
        inputs.angle = angleMotor.getPosition();
        inputs.dropperPower = dropperMotor.getMotorOutputPercent();        
    }


}
