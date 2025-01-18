// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

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
}
