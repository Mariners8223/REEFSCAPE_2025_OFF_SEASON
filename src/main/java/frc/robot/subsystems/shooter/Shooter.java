package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private final ShooterIO io;
    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();
    
    public Shooter(){
        io = new ShooterIOReal();
    }

    public void spinBackMotor(double RPM){
        io.spinBackWheel(RPM);
    }

    public void spinFrontMotor(double RPM){
        io.spinFrontWheel(RPM);
    }

    public void stopMotors(){
        io.spinFrontWheel(0);
        io.spinBackWheel(0);
    }
}
