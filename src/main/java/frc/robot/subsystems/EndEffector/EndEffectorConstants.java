// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase.MotorType;

/** Add your docs here. */
public class EndEffectorConstants {
    public static final int rightID = 2;
    public static final int leftID = 3;
    public static final int beamBreakPort = 1;

    public static final boolean beamBreakInverted = true;

    public static enum MotorPower{
        INTAKE(0.1, 0.1, 1),
        L1(0.1, 0.1, 1),
        L2_3(0.1, 0.1, 1),
        L4(0.1, 0.1, 1);

        public final double leftMotorPower;
        public final double rightMotorPower;
        public final double ejectTime;

        public static final double maxMotorPower = 0.8;

        MotorPower(double leftMotorPower, double rightMotorPower, double ejectTime){
            this.leftMotorPower = leftMotorPower;
            this.rightMotorPower = rightMotorPower;
            this.ejectTime = ejectTime;
        }
    }

    public static class FunnelMotor{
        public static final ControllerLocation CONTROLLER_LOCATION = null;
        public static final int MOTOR_ID = 4;
        public static final boolean IS_BRUSHLESS = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_MAX;

        public static final boolean IS_INVERTED = false;

        public static final PIDFGains PID_GAINS = new PIDFGains(
            20,
            0,
            0);
        public static final double GEAR_RATIO = 1;


        public static final double target = 0;
        public static final double tolerance = 0;
    }
}
