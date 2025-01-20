// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO{
    private final ElevatorSim elevator;
    // private final PIDController PID;

      private final ProfiledPIDController pidController;
    
    public ElevatorIOSim(){
        // elevator = new ElevatorSim(
        //     DCMotor.getNeoVortex(2),
        //     ElevatorConstants.LeadMotor.GEAR_RATIO,
        //     ElevatorConstants.ELEVATOR_WEIGHT, 
        //     ElevatorConstants.PULLEY_RADIUS, 
        //     ElevatorLevel.Bottom.getHeight(),
        //     ElevatorLevel.L4.getHeight(),
        //     false,
        //     ElevatorLevel.Bottom.getHeight());

        elevator =
            new ElevatorSim(
                DCMotor.getVex775Pro(4),
                5,
                4,
                0.1,
                0,
                6,
                true,
                0,
                0.0, 0.0);
        
        // PID = new PIDController(ElevatorConstants.LeadMotor.PID_GAINS.getP(), ElevatorConstants.LeadMotor.PID_GAINS.getI(), ElevatorConstants.LeadMotor.PID_GAINS.getD());
        pidController = new ProfiledPIDController(
            ElevatorConstants.LeadMotor.PID_GAINS.getP(),
            ElevatorConstants.LeadMotor.PID_GAINS.getI(),
            ElevatorConstants.LeadMotor.PID_GAINS.getD(),
            new TrapezoidProfile.Constraints(2.45, 2.45));
    }

    public void resetMotorEncoder(){
        elevator.setState(0, 0.1);
    }

    public void moveMotorByPosition(double position){
        pidController.setGoal(position);
        // elevator.setInput(PID.calculate(elevator.getPositionMeters()));
        // elevator.setInputVoltage(PID.calculate(elevator.getPositionMeters()));
        // elevator.setInputVoltage(1000);
        // elevator.setInputVoltage(PID.calculate(elevator.getPositionMeters(), position));
        // reachGoal(position);

        // elevator.setState(0, 10);
        // elevator.setInputVoltage(10);
    }

    public void reachGoal(double goal) {
        pidController.setGoal(goal);
    
        // With the setpoint value we run PID control like normal
        double pidOutput = pidController.calculate(elevator.getPositionMeters());
        elevator.setInputVoltage(pidOutput);
    }

    public void Update(ElevatorInputs inputs){
        // elevator.setInput(10);
        // elevator.setState(elevator.getPositionMeters(), 100);
        elevator.setInput(pidController.calculate(elevator.getPositionMeters()));
        elevator.update(1);

        inputs.elevatorHeight = elevator.getPositionMeters();
        inputs.elevator3DPose = new Pose3d(ElevatorConstants.X_ON_ROBOT, ElevatorConstants.Y_ON_ROBOT, inputs.elevatorHeight, new Rotation3d());

        Logger.recordOutput("Elevator/PID Setpoint", pidController.getSetpoint().position);
        Logger.recordOutput("Elevator/PID Calculate", pidController.calculate(elevator.getPositionMeters(), pidController.getSetpoint()));
        Logger.recordOutput("Elevator/Elevator Draw Amps", elevator.getCurrentDrawAmps());

        Logger.recordOutput("Elevator/Elevator Input 0", elevator.getInput(0));
        Logger.recordOutput("Elevator/Elevator Output 0", elevator.getOutput(0));

        // Logger.recordOutput("Elevator/Elevator Input 1", elevator.getInput(1));
        Logger.recordOutput("Elevator/Elevator Output 1", elevator.getOutput(1));

        Logger.recordOutput("Elevator/Velocity", elevator.getVelocityMetersPerSecond());
    }
}