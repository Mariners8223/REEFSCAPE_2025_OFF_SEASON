// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;
import frc.util.PIDFGains;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO{
    ElevatorSim elevator;
    PIDController PID;

    public ElevatorIOSim(){
        elevator = new ElevatorSim(
            DCMotor.getNeoVortex(2),
             ElevatorConstants.LeadMotor.GEAR_RATIO,
             ElevatorConstants.ELEVATOR_WEIGHT, 
             ElevatorConstants.PULLEY_RADIUS, 
             ElevatorLevel.Bottom.getHeight(),
             ElevatorLevel.L4.getHeight(),
             true,
             ElevatorLevel.Bottom.getHeight(),
             null);
        
        PID = new PIDController(ElevatorConstants.LeadMotor.PID_GAINS.getP(), ElevatorConstants.LeadMotor.PID_GAINS.getI(), ElevatorConstants.LeadMotor.PID_GAINS.getD());
    }

    public void resetMotorEncoder(){
        elevator.setState(0, 0);
    }

    public void moveMotorByPosition(double position){
        PID.
    }
}
