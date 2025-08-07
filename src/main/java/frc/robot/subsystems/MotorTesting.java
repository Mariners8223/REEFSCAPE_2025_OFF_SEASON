package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorTesting extends SubsystemBase {
    MotorTestingIOReal io;
    public MotorTesting(){
        io = new MotorTestingIOReal();
    }

    public void setMotorDutyCycle(double dutyCycle){io.setTestMotorDutyCyle(dutyCycle);}

    public void stopMotor(){io.stopTestMotor();}
}
