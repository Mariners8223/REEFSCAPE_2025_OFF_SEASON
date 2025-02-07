// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController.ControllerLocation;

/** Add your docs here. */
public class EndEffectorConstants {
    public static final int RIGHT_ID = 22;
    public static final boolean RIGHT_INVERTED = true;
    public static final int LEFT_ID = 21;
    public static final boolean LEFT_INVERTED = false;
    public static final int BEAM_BREAK_PORT = 9;

    public static final boolean BEAM_BREAK_INVERTED = true;

    public static final double MAX_MOTOR_POWER = 0.8;

    public static final boolean STARTS_WITH_GP = false;

    public enum MotorPower{
        INTAKE(0.3, 0.3, 1),
        L1(-0.1, 0.7, 1),
        L2_3(0.1, 0.1, 1),
        L4(0.1, 0.1, 1);

        public final double leftMotorPower;
        public final double rightMotorPower;
        public final double ejectTime;

        MotorPower(double leftMotorPower, double rightMotorPower, double ejectTime){
            this.leftMotorPower = leftMotorPower;
            this.rightMotorPower = rightMotorPower;
            this.ejectTime = ejectTime;
        }
    }

    public static class FunnelMotor{
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.RIO;
        public static final int MOTOR_ID = 23;

        public static final boolean IS_INVERTED = true;

        public static final PIDFGains PID_GAINS = new PIDFGains(
            3,
            0,
            1,
            0,
            0.1,
            0);
        public static final double GEAR_RATIO = 9;


        public static final double COLLECT_POSITION = 0;
        public static final double CLIMB_POSITION = 0.75;
        public static final double TOLERANCE = 0;
    }
}
