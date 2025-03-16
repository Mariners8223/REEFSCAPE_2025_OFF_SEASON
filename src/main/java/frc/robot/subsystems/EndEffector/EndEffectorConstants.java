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

    public static final double X_ON_ROBOT = -0.34;
    public static final double Y_ON_ROBOT = -0.33;
    public static final double Z_OFFSET = 0.735;

    public enum MotorPower{
        INTAKE(0.5, 0.5, 0.25),
        L1_RIGHT(0.65, -0.3, 1),
        L1_LEFT(-0.3, 0.65, 1),
        L2_3(0.4, 0.4, 0.5),
        L4(0.5, 0.5, 0.5);

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
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 23;

        public static final boolean IS_INVERTED = true;

        public static final PIDFGains PID_GAINS = new PIDFGains(
            200,
            2,
            0,
            0,
            0.0,
            0.05);
        public static final double GEAR_RATIO = 45;


        public static final double COLLECT_POSITION = 0.05;
        public static final double CLIMB_POSITION = -0.3;
        public static final double TOLERANCE = 0.05;
    }
}
