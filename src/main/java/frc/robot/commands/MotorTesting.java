package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorTesting extends SubsystemBase {
    MotorTestingIO io;

    public void setMotorDutyCycle(double dutyCycle){io.setTestMotorDutyCyle(dutyCycle);}

    public void stopMotor(){io.stopTestMotor();}
}
