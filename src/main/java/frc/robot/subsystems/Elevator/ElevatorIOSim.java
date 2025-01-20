// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO{
    private final ElevatorSim elevator;
    private final PIDController pidController;
    
    public ElevatorIOSim(){
        elevator =
            new ElevatorSim(
                DCMotor.getNeoVortex(2),
                ElevatorConstants.LeadMotor.GEAR_RATIO,
                ElevatorConstants.ELEVATOR_WEIGHT,
                ElevatorConstants.PULLEY_RADIUS,
                ElevatorConstants.LeadMotor.SOFT_MINIMUM,
                ElevatorConstants.LeadMotor.SOFT_MAXIMUM,
                true,
                ElevatorConstants.ElevatorLevel.Bottom.getHeight(),
                0.0, 0.0);
        
        // PID = new PIDController(ElevatorConstants.LeadMotor.PID_GAINS.getP(), ElevatorConstants.LeadMotor.PID_GAINS.getI(), ElevatorConstants.LeadMotor.PID_GAINS.getD());
        pidController = ElevatorConstants.LeadMotor.PID_GAINS.createPIDController();
    }

    public void resetMotorEncoder(){
        elevator.setState(0, 0.1);
    }

    public void moveMotorByPosition(double position){
        pidController.setSetpoint(position);
    }

    public void Update(ElevatorInputs inputs){
        elevator.setInput(pidController.calculate(elevator.getPositionMeters()));
        elevator.update(0.02);

        inputs.elevatorHeight = elevator.getPositionMeters();
        inputs.elevator3DPose = new Pose3d(ElevatorConstants.X_ON_ROBOT, ElevatorConstants.Y_ON_ROBOT, inputs.elevatorHeight, new Rotation3d());
    }
}