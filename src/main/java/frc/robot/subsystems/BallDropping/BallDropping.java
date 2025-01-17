// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BallDropping;

//import frc.robot.subsystems.BallDropping.BallDroppingIO;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.BallDropping.BallDroppingIO.balldroppingInputs;

public class BallDropping extends SubsystemBase{

    BallDroppingIO io;
    balldroppingInputs inputs = new balldroppingInputs();

    public BallDropping(){
        // TODO: add IOReal 
        this.resetAngleEncoder();
    }

    //Angle subsystems
    public void resetAngleEncoder(){
        io.resetAngleEncoder();
    }

    public void reachAngle(double angleToReach){
        io.reachAngle(angleToReach);
    }

    public double getAngle(){
        return io.getAngle();
    }

    //dropper subsystems
    public void setDropperMotorPower(double dropperPower){
        io.setDropperMotorPower(dropperPower);
    }

    public void stopDropperMotor(){
        setDropperMotorPower(0);
    }

    public double getDropperMotorPower(){
        return io.getDropperMotorPower();
    }

    //update logs
    @Override
    public void periodic() {
        io.Update(inputs);
    }
}
