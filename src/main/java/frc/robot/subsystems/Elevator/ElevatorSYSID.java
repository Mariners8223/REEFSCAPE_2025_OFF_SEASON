// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

/** Add your docs here. */
public class ElevatorSYSID {
    private final SysIdRoutine routine;
    
    public ElevatorSYSID(Elevator elevator){
        routine = new SysIdRoutine(
            new Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Elevator/SYSID/State", state.toString())
            ),
            
            new Mechanism(
                (voltage) -> elevator.setVoltage(voltage.baseUnitMagnitude()),
                // (log) -> {
                //     log.motor("SYSID Elevator")
                //     .voltage(Voltage.ofBaseUnits(elevator.getVoltage(), Volts))
                //     .linearPosition(Distance.ofBaseUnits(elevator.getCurrentHeight(), Meters))
                //     .linearVelocity(LinearVelocity.ofBaseUnits(elevator.getVelocity(), MetersPerSecond)); },
                null,
                elevator
            ));
    }

    public Command getElevatorQuasistatic(Direction direction){
        return routine.quasistatic(direction);
    }

    public Command getElevatorDynamic(Direction direction){
        return routine.dynamic(direction);
    }
}
