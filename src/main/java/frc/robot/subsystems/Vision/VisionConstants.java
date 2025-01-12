package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    public enum CameraConstants{
        FRONT_LEFT("front left",
            new Transform3d(
                0, 0, 0,
                new Rotation3d(0, 0, 0))),


        BACK_MIDDLE("back middle",
            new Transform3d(
                0, 0, 0,
                new Rotation3d(0, 0, 0)));

        public final String cameraName;

        public final Transform3d robotToCamera;
        

        private CameraConstants(String name, Transform3d robotToCamera){
            this.cameraName = name;
            this.robotToCamera = robotToCamera;
        }

        @Override
        public String toString(){
            return cameraName;
        }
    }
 
}
