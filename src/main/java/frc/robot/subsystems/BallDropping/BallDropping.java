// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BallDropping;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallDropping extends SubsystemBase{
    BallDroppingIO io;
    balldroppingInputsAutoLogged inputs = new balldroppingInputsAutoLogged();

    public BallDropping(){
        io = new BallDroppinIOReal();

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
        return inputs.angle;
    }

    //dropper subsystems
    public void setDropperMotorPower(double dropperPower){
        io.setDropperMotorPower(dropperPower);
    }

    public void setVoltage(double voltage) { io.setVoltage(voltage);}

    public void stopDropperMotor(){
        setDropperMotorPower(0);
    }

    public double getDropperMotorPower(){
        return inputs.dropperPower;
    }

    //update logs
    @Override
    public void periodic() {
        io.Update(inputs);
        Logger.processInputs(getName(), inputs);
    }
}
