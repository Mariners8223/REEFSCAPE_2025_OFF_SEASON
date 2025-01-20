// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * Add your docs here.
 */
public class Constants {
    public enum RobotType {
        COMPETITION,
        DEVELOPMENT,
        REPLAY
    }
    public enum reef{
    //HEY
        REEF1 (5.864,3.8789,-180),
        REEF2 (5.85,4.21,-180),
        REEF3 (5.343,5.146,-120),
        REEF4 (5.021,5.293,-120),
        REEF5 (3.959,5.312,-60),
        REEF6 (3.646,5.068,-60),
        REEF7 (3.140,4.21,0),
        REEF8 (3.13,3.849,0),
        REEF9 (3.627,2.933,60),
        REEF10 (3.91,2.796,60),
        REEF11 (5.08,2.757,120),
        REEF12 (5.304,2.913,120);
reef( double x, double y, double deg){
    
    }

    public static final RobotType ROBOT_TYPE = RobotType.DEVELOPMENT; //the type of robot the code is running on
}
}