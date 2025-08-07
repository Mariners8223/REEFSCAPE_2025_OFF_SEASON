package frc.robot.subsystems;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSparkBase;


public class MotorTestingIOReal implements MotorTestingIO {
    private final MarinersController motorTest;


    public MotorTestingIOReal(){
        motorTest = configureTestMotor();
    }
    private MarinersController configureTestMotor(){
        MarinersController motor = new MarinersSparkBase("test motor", MarinersController.ControllerLocation.MOTOR,
                2, true, MarinersSparkBase.MotorType.SPARK_MAX);




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
