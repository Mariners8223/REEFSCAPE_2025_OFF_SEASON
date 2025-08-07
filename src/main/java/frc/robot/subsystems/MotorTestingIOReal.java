package frc.robot.subsystems;
import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSparkBase;


public class MotorTestingIOReal implements MotorTestingIO {
    private final MarinersController motorTest;


    public MotorTestingIOReal(){
        motorTest = configureTestMotor();
    }
    private MarinersController configureTestMotor(){
        MarinersController motor = new MarinersSparkBase("test motor", MarinersController.ControllerLocation.MOTOR,
                5, true, MarinersSparkBase.MotorType.SPARK_MAX);

        motor.setMotorInverted(true);
        motor.setPIDF(new PIDFGains(0.6660000085830688, 0, 0.005799999926239252));
        motor.setMotorIdleMode(false);

        return motor;
    }

    @Override
    public void setTestMotorDutyCyle(double dutyCyle) {
        motorTest.setDutyCycle(dutyCyle);
    }
    public  void stopTestMotor(){
        motorTest.stopMotor();
;
    }
//    public void Update(MotorTestInputs inputs) {
//        inputs.motorCurrent = motorTest.getMotorOutputPercent();
//        inputs.leftPower = LeftMotor.getMotorOutputPercent();
//
//    }
}
