// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import frc.robot.subsystems.BallDropping.BallDropping;
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
//public class  Abortion{}
public class BallDroppingSYSID {
    private final SysIdRoutine routine;

    public BallDroppingSYSID(BallDropping ballDropping){
        routine = new SysIdRoutine(
                new Config(
                        Voltage.ofBaseUnits(0.5, Volts).per(Seconds),
                        Voltage.ofBaseUnits(2, Volts),
                        Time.ofBaseUnits(100, Seconds),
                        (state) -> Logger.recordOutput("BallDropping/SYSID/State", state.toString())
                ),

                new Mechanism(
                        (voltage) -> ballDropping.setVoltage(voltage.baseUnitMagnitude()),
                        null,
                        ballDropping
                ));
    }

    public Command getBallDroppingQuasistatic(Direction direction){
        return routine.quasistatic(direction);
    }

    public Command getBallDroppingDynamic(Direction direction){
        return routine.dynamic(direction);
    }
}
