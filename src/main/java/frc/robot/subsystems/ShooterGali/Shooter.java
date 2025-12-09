package frc.robot.subsystems.ShooterGali;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.BallDrop.BallDroppingInputsAutoLogged;
import frc.robot.subsystems.shooterGali.ShooterIO;

public class Shooter extends  SubsystemBase
{
    private ShooterIO io;
    ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    public Shooter()
    {
        io = new ShooterIOReal();
        this.resetAngleEncoder();
    }

    public void resetAngleEncoder(){
        io.resetAngleEncoder();
    }

    public void setBackMotorPower(double power){
        io.setBackMotorPower(power);
    }

    public void setFrontMotorPower(double power){
        io.setFrontMotorPower(power);
    }

    public void stopFrontMotor()
    {
        io.setFrontMotorPower(0);
    }

    public void stopBackMotor()
    {
        io.setBackMotorPower(0);
    }

    public boolean isGPInside()
    {
        return inputs.beamBreak;
    }

    public void setAngle(double angleToReach){
        io.setAngle(angleToReach);
    }

    public double getAngle(){
        return inputs.angle;
    }

    @Override
    public void periodic() {
        io.Update(inputs);
        Logger.processInputs(getName(), inputs);

        String currentCommandName = getCurrentCommand() == null ? "Null" : getCurrentCommand().toString();
        Logger.recordOutput("Shooter/Current Command", currentCommandName);
    }
}
