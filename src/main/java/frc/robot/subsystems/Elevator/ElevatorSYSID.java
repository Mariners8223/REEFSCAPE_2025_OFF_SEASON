// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Time;
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
                Voltage.ofBaseUnits(0.5, Volts).per(Seconds),
                Voltage.ofBaseUnits(2, Volts),
                Time.ofBaseUnits(100, Seconds),
                (state) -> Logger.recordOutput("Elevator/SYSID/State", state.toString())
            ),
            
            new Mechanism(
                (voltage) -> elevator.setVoltage(voltage.baseUnitMagnitude()),
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
