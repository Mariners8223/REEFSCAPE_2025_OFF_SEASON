package main.java.frc.robot.subsystems.shooter;

public class shooterIOReal implements shooterIO
{
    private final MarinersTalonFX BackMotor;
    private final MarinersTalonFX FrontMotor;
    private final MarinersTalonFX AngleMotor;
    private final DigitalInput beamBreak;

    public shooterIOReal()
    {
        
        FrontMotor = configureShooterMotor(ShooterConstants.FrontMotor.FRONT_ID, ShooterConstants.FRONT_INVERTED);
        BackMotor = configureShooterMotor(ShooterConstants.BackMotor.BACK_ID, ShooterConstants.FRONT_INVERTED);
        AngleMotor = configureShooterMotor(ShooterConstants.AngleMotor.ANGLE_ID, ShooterConstants.ANGLE_INVERTED);

        beamBreak = new DigitalInput(ShooterConstants.BEAM_BREAK_PORT);
    }
    private MarinersTalonFX configureFrontMotor()
    {
        MarinersTalonFX motor = new MarinersTalonFX("FrontMotor", ShooterConstants.FrontMotor.ControllerLocation.CONTROLLER_LOCATION,
                                                    ShooterConstants.FrontMotor.MOTOR_ID, ShooterConstants.FrontMotor.PID_GAINS,
                                                    ShooterConstants.FrontMotor.GEAR_RATIO);
        motor.setMotorInverted(ShooterConstants.FrontMotor.IS_INVERTED);
        motor.setMotorIdleMode(true);
        motor.setMaxMinOutput(8, 7);

        return motor; 
    }
    private MarinersTalonFX configureBackMotor()
    {
        MarinersTalonFX motor = new MarinersTalonFX ("BackMotor", ShooterConstants.BackMotor.ControllerLocation.CONTROLLER_LOCATION,
                                                    ShooterConstants.BackMotor.MOTOR_ID, ShooterConstants.BackMotor.PID_GAINS,
                                                    ShooterConstants.FrontMotor.GEAR_RATIO);
        motor.setMotorInverted(ShooterConstants.BackMotor.IS_INVERTED);
        motor.setMotorIdleMode(true);
        motor.setMaxMinOutput(8, 7);

        return motor;
    }
    private MarinersTalonFX configureAngleMotor()
    {
        MarinersTalonFX motor = new MarinersTanlonFX("AngleMotor", ShooterConstants.AngleMotor.ControllerLocation.CONTROLLER_LOCATION,
                                                    ShooterConstants.AngleMotor.MOTOR_ID, ShooterConstants.AngleMotor.PID_GAINS,
                                                    ShooterConstants.AngleMotor.GEAR_RATIO);
        motor.setMotorInverted(ShooterConstants.AngleMotor.IS_INVERTED);
        motor.setMotorIdleMode(true);
        motor.setMaxMinOutput(8, 7);

        return motor;
    }

    
    public void frontSpin(double power)
    {
        double realPower = MathUtil.clamp(power, -ShooterConstants.MAX_MOTOR_POWER,
                                            ShooterConstants.MAX_MOTOR_POWER);
        FrontMotor.set();//TODO: i dont know wtf do i need to put here
    }

    public void backSpin(double power)
    {
        double realPower = MathUtil.clamp(power, -ShooterConstants.MAX_MOTOR_POWER,
                                            ShooterConstants.MAX_MOTOR_POWER);
        BackMotor.set();//TODO: fix this shit
    }

    public void setAngleTo(double angle)
    {
        AngleMotor.setReference(angle, ControlMode.Position, calculateFeedForward(angleMotor.getPosition()));
    }

    public void update(shooterInputes inputes)
    {
        inputs.frontPower = FrontMotor.getMotorOutputPercent();
        inputs.BackMotor = BackMotor.getMotorOutputPercent();
        inputs.AngleMotor = AngleMotor.getMotorOutputPercent();//TODO: find how to get the angle instead of motorOutputPercent
        inputs.beamBreakValue = ShooterConstants.BEAM_BREAK_INVERTED == beamBreak.get();
    }
}
