package frc.robot.subsystems.BallDrop;

import java.lang.ModuleLayer.Controller;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControlMode;
import frc.util.MarinersController.MarinersController.ControllerLocation;

public class BallDropIOReal implements BallDropIO{

 private final VictorSPX wheelMotor;
 private final MarinersController armMotor;

 public BallDropIOReal()
 {
    this.wheelMotor = configureWheelMotor();
    this.armMotor = configureArmMotor();
 }
private MarinersController configureArmMotor(){
        MarinersSparkBase motor;
        motor = new MarinersSparkBase("Arm Motor", ControllerLocation.MOTOR, BallDropConstants.ArmMotor.MOTOR_ID, BallDropConstants.ArmMotor.IS_BRUSHLESS, BallDropConstants.ArmMotor.MOTOR_TYPE, BallDropConstants.ArmMotor.ANGLE_PID);

        motor.enableSoftLimits(BallDropConstants.ArmMotor.SOFT_MINIMUM, BallDropConstants.ArmMotor.SOFT_MAXIMUM);

        motor.setMotorInverted(BallDropConstants.ArmMotor.IS_INVERTED);
        motor.setMotorIdleMode(true);

        return motor;
    }
private VictorSPX configureWheelMotor()
{
    VictorSPX motor;
    motor = new VictorSPX(BallDropConstants.DropperMotor.ID);
    motor.setInverted(BallDropConstants.DropperMotor.IS_INVERTED);
    motor.setNeutralMode(NeutralMode.Brake);
    return motor;
}

    @Override
    public void setVoltageWheel(double voltage) {
        armMotor.setVoltage(voltage);
    }

    public void resetMotorEncoder() {
        armMotor.resetMotorEncoder();
    }

    public double getAngle()
    {
        return armMotor.getPosition();
    }
    private double calculateFeedForward(double motorRotation){
        return Math.sin(motorRotation * 2 * Math.PI) * BallDropConstants.ArmMotor.MOTOR_FEED_FORWARD;
    }
    public void setAngle(double angle) {
        armMotor.setReference(angle,ControlMode.Position, calculateFeedForward(armMotor.getPosition()));
    }
    
    public void Update(BallDroppingInputs inputs) {
        inputs.armAnglePose = armMotor.getPosition();
        inputs.ballDropping3DPose = new Pose3d (BallDropConstants.X_ON_ROBOT,BallDropConstants.Y_ON_ROBOT,BallDropConstants.Z_OFFSET,new Rotation3d(0,inputs.armAnglePose,0));
    }

}
