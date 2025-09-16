package frc.robot.subsystems.funnel;

import frc.robot.subsystems.funnel.funnelConstants.FunnelConstants;
import frc.robot.subsystems.funnel.funnelConstants.funnelMotor;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControlMode;

public class funnelIOReal implements FunnelIO {
    private final MarinersTalonFX funnelMotor;
    @SuppressWarnings ("unused")

    public funnelIOReal(){
        this.funnelMotor = configureLeadMotor();   
    }

    private MarinersTalonFX configureLeadMotor(){
        MarinersTalonFX motor = new MarinersTalonFX("Funnel Motor", funnelConstants.funnelMotor.CONTROLLER_LOCATION,
            funnelConstants.funnelMotor.MOTOR_ID, funnelConstants.funnelMotor.ROTATION_TO_ANGLE);
        return motor;
    }
    @Override
    public void setMotorDutyCycle(double MotorDutyCycle) {
        funnelMotor.setDutyCycle(MotorDutyCycle);
    }
    public void setMotorPosition(double MotorPosition) {
        funnelMotor.setReference(MotorPosition, ControlMode.Position);
    }
    public void stopMotor(){
        funnelMotor.stopMotor();
    }
} 
















































































































































































































































//yaga baga duuuduu//