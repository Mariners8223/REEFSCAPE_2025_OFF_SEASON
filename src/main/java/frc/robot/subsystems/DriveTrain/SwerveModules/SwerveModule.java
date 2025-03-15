package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.ROBOT_TYPE;

public class SwerveModule {

    /**
     * the name of the swerve modules by order
     */
    public enum ModuleName {
        Front_Left,
        Front_Right,
        Back_Left,
        Back_Right
    }

    private final String moduleName;
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    public SwerveModule(ModuleName name) {
        this.moduleName = name.toString();

        if (RobotBase.isReal()) {
            io = new SwerveModuleIODevBot(name);
        } else {
            io = ROBOT_TYPE == Constants.RobotType.REPLAY ? new SwerveModuleIOSIM.SwerveModuleIOReplay() : new SwerveModuleIOSIM(name);

        }
    }

    public SwerveModulePosition modulePeriodic() {
        io.updateInputs(inputs);

        Logger.processInputs("SwerveModule/" + moduleName, inputs);

        return new SwerveModulePosition(inputs.drivePositionMeters, inputs.currentState.angle);
    }

    public SwerveModuleState run(SwerveModuleState targetState) {
        targetState.optimize(inputs.currentState.angle);

        targetState.cosineScale(inputs.currentState.angle);

        io.setDriveMotorReference(targetState.speedMetersPerSecond);
        io.setSteerMotorReference(targetState.angle.getRotations());

        return targetState;
    }

    public SwerveModuleState run(SwerveModuleState targetState, double acceleration) {
        double inputtedVelocity = targetState.speedMetersPerSecond;

        targetState.optimize(inputs.currentState.angle);

        //checks if the speed was flipped
        if(Math.signum(inputtedVelocity) != Math.signum(targetState.speedMetersPerSecond)){
            acceleration = -acceleration;
        }

        double cosineValue = targetState.angle.minus(inputs.currentState.angle).getCos();

        targetState.speedMetersPerSecond *= cosineValue;
        acceleration *= cosineValue;

        io.setDriveMotorReference(targetState.speedMetersPerSecond, acceleration);
        io.setSteerMotorReference(targetState.angle.getRotations());

        return targetState;
    }

    public void runSysID(double voltage, Rotation2d angle) {
        io.setDriveMotorVoltage(voltage);
        io.setSteerMotorReference(angle.getRotations());
    }

    public void runSysIDSteer(Voltage voltage){
        io.setDriveMotorVoltage(0);
        io.setSteerMotorVoltage(voltage.baseUnitMagnitude());
    }

    public SwerveModuleState getCurrentState() {
        return inputs.currentState;
    }

    public void runDriveCalibration() {
        io.startDriveCalibration();
    }

    public void stopDriveCalibration() {
        io.endDriveCalibration();
    }

    public void runSteerCalibration() {
        io.startSteerCalibration();
    }

    public void stopSteerCalibration() {
        io.endSteerCalibration();
    }

    public void setIdleMode(boolean isBrakeMode) {
        io.setIdleMode(isBrakeMode);
    }

    public void resetDriveEncoder() {
        io.resetDriveEncoder();
    }
}
