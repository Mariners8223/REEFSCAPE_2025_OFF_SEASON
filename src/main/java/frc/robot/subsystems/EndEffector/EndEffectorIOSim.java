// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class EndEffectorIOSim implements EndEffectorIO{
    private double rightMotorPower;
    private double leftMotorPower;

    private final SingleJointedArmSim funnelSim;
    private final PIDController Pid;

    public EndEffectorIOSim(){
        funnelSim = new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            EndEffectorConstants.FunnelMotor.GEAR_RATIO,
            1,
            0.5,
            -Math.PI/6,
            (double) 3 /2 * Math.PI,
            true,
            -Math.PI/6);
        
        Pid = EndEffectorConstants.FunnelMotor.PID_GAINS.createPIDController();

        SmartDashboard.putBoolean("beam break value", false);
    }

    @Override
    public void setRightMotorPower(double powerToSet) {
        this.rightMotorPower = powerToSet;
    }

    @Override
    public void setLeftMotorPower(double powerToSet) {
        this.leftMotorPower = powerToSet;
    }

    @Override
    public void resetFunnelEncoder(){
        funnelSim.setState(-Math.PI/6, 0);
    }

    @Override
    public void moveFunnel(double target){
        Pid.setSetpoint(target);
    }

    @Override
    public void setFunnelVoltage(double voltage){
        funnelSim.setInput(voltage);
    }

    @Override
    public void stopFunnel(){
        funnelSim.setInput(0);
    }

    public void startFunnelPIDCalibration(){
        return;
    }

    public void endFunnelPIDCalibration(){
        return;
    }

    @Override
    public void Update(EndEffectorInputs inputs) {
        funnelSim.setInput(Pid.calculate(funnelSim.getAngleRads()));
        funnelSim.update(0.02);

        inputs.funnelPosition = funnelSim.getAngleRads();
        inputs.leftPower = leftMotorPower;
        inputs.rightPower = rightMotorPower;
        inputs.beamBreakValue = SmartDashboard.getBoolean("beam break value", false);
    }

}
