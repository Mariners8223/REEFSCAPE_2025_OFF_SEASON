// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

/** Add your docs here. */
public class ElevatorConstants {
    public enum ElevatorLevel{
        Bottom(0),
        L1(1),
        L2(2),
        L3(3),
        L4(4),
        Intake(5),
        Moving(-1),
        NULL(-1);

        private double height;
        private ElevatorLevel(double height){
            this.height = height;
        }

        public double getHeight(){
            return this.height;
        }
    }

    public static final double GEAR_RATIO = 1;
    public static final double ELEVATOR_TOLERANCE = 0.1;
}
