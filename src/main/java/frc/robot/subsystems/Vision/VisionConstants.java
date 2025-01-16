package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public enum CameraConstants{
        FRONT_LEFT("front left",
            new Transform3d(
                -0.11, 0.31, -0.17,
                new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-90))));

        // BACK_MIDDLE("back middle",
        //     new Transform3d(
        //         0, 0, 0,
        //         new Rotation3d(0, 0, 0)));

        public final String cameraName;

        public final Transform3d robotToCamera;
        

        CameraConstants(String name, Transform3d robotToCamera){
            this.cameraName = name;
            this.robotToCamera = robotToCamera;
        }

        @Override
        public String toString(){
            return cameraName;
        }
    }
    //TODO test to find better numbers
    public static final double maxHightDeveation = 0.05;
    public static final double maxMultiAmbiguity = 0.5;
    public static final double maxSingleAmbiguity = 0.3;

    public static final double xStdDev = 2;
    public static final double yStdDev = 2;
    public static final double ThetaStdDev = 4;



}
