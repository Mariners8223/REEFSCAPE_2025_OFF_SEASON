// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BallDropping;

import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSparkBase;
import frc.robot.subsystems.BallDropping.BallDroppingConstants.AngleMotor.WhichMotor;


/** Add your docs here. */
public class BallDroppinIOReal implements BallDroppingIO{
    WhichMotor witchMotor;
    MarinersController angleMotor;


    public BallDroppinIOReal(Boolean withRev){
        //this.angleMotor = (this.withRev) ? configureAngleMotorAsRev() : configureAngleMotorAsCTRE();
        switch(witchMotor){
            
        }
    }

    public MarinersSparkBase configureAngleMotorAsRev(){
        MarinersSparkBase motor;
        motor = new MarinersSparkBase("angle drop motor",BallDroppingConstants.AngleMotor.location, 
        BallDroppingConstants.AngleMotor.id, BallDroppingConstants.AngleMotor.isBrushless,  
        BallDroppingConstants.AngleMotor.type_rev, BallDroppingConstants.AngleMotor.PID_gains, BallDroppingConstants.AngleMotor.gearRatio);
    
    
        return motor;
    }


    //angle motor implement
    public void resetAngleEncoder(){

    }

    public void reachAngle(double angleToReach){

    }

    public double getAngle(){

    }

    //dropping motor implement
    /*public void setDropperMotorPower(double dropperPower){

    }

    public void stopDropperMotor(){

    }
    
    public double getDropperMotorPower(){
    
    }
    */


    public void Update(balldroppingInputs inputs){
        
    }


}
