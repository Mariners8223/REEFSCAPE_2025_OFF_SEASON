package frc.robot.subsystems.endEffector;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.subsystems.endEffector.EndEffectorConstents;
import edu.wpi.first.wpilibj.DigitalInput;

public class EndEffectorIOReal implements EndEffectorIO{
    private final VictorSPX RightMotor;
    private final VictorSPX LeftMotor;
    private final DigitalInput beambreak;

    public EndEffectorIOReal(){
      RightMotor = configureEndEffectorMotor(EndEffectorConstents.R_ID);
      LeftMotor = configureEndEffectorMotor(EndEffectorConstents.L_ID);
      beambreak = new DigitalInput(EndEffectorConstents.BEAM_BREAK_PORT);
    }

    private VictorSPX configureEndEffectorMotor(int ID) {
        VictorSPX motor = new VictorSPX(ID);
        motor.setNeutralMode(NeutralMode.Brake);
        return motor;
    }


}
