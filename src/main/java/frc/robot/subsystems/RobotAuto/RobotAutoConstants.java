package frc.robot.subsystems.RobotAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Elevator.ElevatorConstants;

import java.util.Map;

public class RobotAutoConstants {
        public static double XY_DEADBAND = 0;
        public static double THETA_DEADBAND = 0.4;

        public static double XY_TOLERANCE = 0.0105;
        public static double THETA_TOLERANCE = Units.degreesToRadians(2);

        public static double FAR_FROM_TARGET_DISTANCE = 1;

        public static double THETA_IZONE = Units.degreesToRadians(10);

        public static final Map<ElevatorConstants.ElevatorLevel, HomeToReefPIDXY> XY_PID_CONSTANTS =
                Map.of(
                        ElevatorConstants.ElevatorLevel.Bottom, new HomeToReefPIDXY(15, 5, 0, XY_TOLERANCE, 0.1),
                        ElevatorConstants.ElevatorLevel.L1, new HomeToReefPIDXY(12, 18, 0.4, XY_TOLERANCE * 1.5, 0.1),
                        ElevatorConstants.ElevatorLevel.L2, new HomeToReefPIDXY(9, 12, 0.7, XY_TOLERANCE, 0.1),
                        ElevatorConstants.ElevatorLevel.L3, new HomeToReefPIDXY(10, 15, 0.8, XY_TOLERANCE, 0.1),
                        ElevatorConstants.ElevatorLevel.L4, new HomeToReefPIDXY(6, 20, 0.5, XY_TOLERANCE, 0.1)
                );

        public static final Map<ElevatorConstants.ElevatorLevel, HomeToReefPIDTheta> THETA_PID_CONSTANTS =
                Map.of(
                        ElevatorConstants.ElevatorLevel.Bottom, new HomeToReefPIDTheta(6, 15, 0, THETA_TOLERANCE, THETA_IZONE),
                        ElevatorConstants.ElevatorLevel.L1, new HomeToReefPIDTheta(6, 15, 0, THETA_TOLERANCE, THETA_IZONE),
                        ElevatorConstants.ElevatorLevel.L2, new HomeToReefPIDTheta(6, 15, 0, THETA_TOLERANCE, THETA_IZONE),
                        ElevatorConstants.ElevatorLevel.L3, new HomeToReefPIDTheta(6, 15, 0, THETA_TOLERANCE, THETA_IZONE),
                        ElevatorConstants.ElevatorLevel.L4, new HomeToReefPIDTheta(6, 11, 0, THETA_TOLERANCE, THETA_IZONE)
                );

        public static class HomeToReefPIDXY{
            private final PIDController translationController;


            public HomeToReefPIDXY(double kp, double ki, double kd, double tolerance, double iZone){
                translationController = new PIDController(kp, ki, kd);

                translationController.setTolerance(tolerance);
                translationController.setSetpoint(0);

                translationController.setIntegratorRange(-100, 100);

                // translationController.setIZone(iZone);
            }

            public PIDController getTranslationController(){
                return translationController;
            }
        }

        public static class HomeToReefPIDTheta{
            private final PIDController ThetaController;

            public HomeToReefPIDTheta(double kp, double ki, double kd, double tolerance, double iZone){
                ThetaController = new PIDController(kp, ki, kd);

                ThetaController.setTolerance(tolerance);
                ThetaController.setIZone(iZone);

                ThetaController.enableContinuousInput(-Math.PI, Math.PI);
            }

            public PIDController getThetaController(){
                return ThetaController;
            }
        }
}
