package main.java.frc.robot.subsystems.shooter;

public class shooter extends SubsystemBase {
    private final shooterIO io;
    private final shooterInputesAutoLog inputes = new ShooterInputesAutoLog();
    
    public shooter()
    {
        io = new shooterIOReal();
    }

    public void frontSpin(double power)
    {
        io.frontSpin(power);
    }

    public void backSpin(double power)
    {
        backSpin(power);
    }

    public void setAngleTo(double angle)
    {
        setAngleTo(angle);
    }

  
    public boolean isGpIn()
    {
        return input.beamBreakValue;
    }


    public void periodic()
    {
        io.updateInputs(inputs);
    }
}
