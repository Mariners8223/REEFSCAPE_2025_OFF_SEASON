package frc.robot.subsystems.Climb;

import frc.robot.subsystems.Climb.ClimbConstants.climbConstants;
import frc.robot.subsystems.Climb.ClimbConstants.climbMotor;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControlMode;

public class ClimbIOReal implements ClimbIO{
    private final MarinersTalonFX climbMotor;
    @SuppressWarnings ("unused")

    public funnelIOReal(){
        this.climbMotor = configureClimbMotor();
    }

    private MarinersTalonFX configureClimbMotor(){
        MarinersTalonFX motor = new MarinersTalonFX("Climb Motor", climbConstants.climbMotor.ControllerLocation,
            climbConstants.climbMotor.MOTOR_ID, climbConstants.climbMotor.ROTATION_TO_ANGLE);
        return motor;
    }
    @Override
    public void setMotorDutyCycle(double MotorDutyCycle) {
        climbMotor.setDutyCycle(MotorDutyCycle);
    }
    @Override
    public void setMotorPosition(double MotorPosition) {
        climbMotor.setReference(MotorPosition, ControlMode.Position);
    }
    
    @Override
    public void stopMotor(){
        climbMotor.stopMotor();
    }


    public void Update(climbInputs inputs){
        inputs.somthingAngle = climbMotor.getPosition();
    }
}
