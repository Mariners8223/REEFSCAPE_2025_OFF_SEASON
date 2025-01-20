// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO{
    private final ElevatorSim elevator;
    private final PIDController PID;
    
    public ElevatorIOSim(){
        elevator = new ElevatorSim(
            DCMotor.getNeoVortex(2),
            ElevatorConstants.LeadMotor.GEAR_RATIO,
            ElevatorConstants.ELEVATOR_WEIGHT, 
            ElevatorConstants.PULLEY_RADIUS, 
            ElevatorLevel.Bottom.getHeight(),
            ElevatorLevel.L4.getHeight(),
            false,
            ElevatorLevel.Bottom.getHeight());
        
        PID = new PIDController(ElevatorConstants.LeadMotor.PID_GAINS.getP(), ElevatorConstants.LeadMotor.PID_GAINS.getI(), ElevatorConstants.LeadMotor.PID_GAINS.getD());
    }

    public void resetMotorEncoder(){
        elevator.setState(0, 1);
    }

    public void moveMotorByPosition(double position){
        PID.setSetpoint(position);
        // elevator.setInput(PID.calculate(elevator.getPositionMeters()));
        // elevator.setInputVoltage(PID.calculate(elevator.getPositionMeters()));
        // elevator.setInputVoltage(1000);
    }

    public void Update(ElevatorInputs inputs){
        elevator.setInputVoltage(1);
        elevator.update(1/50);

        inputs.elevatorHeight = elevator.getPositionMeters();
        inputs.elevator3DPose = new Pose3d(ElevatorConstants.X_ON_ROBOT, ElevatorConstants.Y_ON_ROBOT, inputs.elevatorHeight, new Rotation3d());

        Logger.recordOutput("Elevator/PID Setpoint", PID.getSetpoint());
        Logger.recordOutput("Elevator/PID Calculate", PID.calculate(elevator.getPositionMeters(), PID.getSetpoint()));
        Logger.recordOutput("Elevator/Elevator Draw Amps", elevator.getCurrentDrawAmps());
        Logger.recordOutput("Elavtor/Output", elevator.getOutput(0));
    }
}