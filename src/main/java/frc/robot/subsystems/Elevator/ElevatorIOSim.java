// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO{
    private final ElevatorSim elevator;
    // private final PIDController PID;

      private final PIDController pidController;
    
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
            0.05, 0.05);

        pidController = new PIDController(ElevatorConstants.LeadMotor.PID_GAINS.getP(), ElevatorConstants.LeadMotor.PID_GAINS.getI(), ElevatorConstants.LeadMotor.PID_GAINS.getD());
    }

    public void resetMotorEncoder(){
        elevator.setState(0, 0);
    }

    public void moveMotorByPosition(double position){
        pidController.setSetpoint(position);
    }


    public void setVoltage(double voltage){ elevator.setInputVoltage(voltage); }
    public double getVoltage() { return elevator.getInput().get(0, 0); }
    public double getVelocity() { return elevator.getVelocityMetersPerSecond(); }

    public void Update(ElevatorInputs inputs){
        // elevator.setInput(pidController.calculate(elevator.getPositionMeters()));
        elevator.update(0.02);

        inputs.elevatorHeight = elevator.getPositionMeters();
        inputs.elevator3DPose = new Pose3d(ElevatorConstants.X_ON_ROBOT, ElevatorConstants.Y_ON_ROBOT, inputs.elevatorHeight, new Rotation3d());
    }
}