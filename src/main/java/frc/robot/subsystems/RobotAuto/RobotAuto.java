package frc.robot.subsystems.RobotAuto;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorConstants;

public class RobotAuto extends SubsystemBase {
    private Constants.ReefLocation selectedReef = null;
    private ElevatorConstants.ElevatorLevel selectedLevel = null;

    public RobotAuto() {
        SmartDashboard.putBoolean("Level 1", false);
        SmartDashboard.putBoolean("Level 2", false);
        SmartDashboard.putBoolean("Level 3", false);
        SmartDashboard.putBoolean("Level 4", false);
    }

    public Constants.ReefLocation getSelectedReef() {
        return selectedReef;
    }

    public ElevatorConstants.ElevatorLevel getSelectedLevel() {
        return selectedLevel;
    }

    public void setSelectedReef(Constants.ReefLocation reef) {
        String name;

        if(reef != null){
            Robot.setObjectPoseFiled("selected reef", reef.getPose());
            name = reef.name();
        }
        else{
            Robot.clearObjectPoseField("selected reef");
            name = "None";
        }

        SmartDashboard.putString("selected Reef", name);
        Logger.recordOutput("Selection/Reef", name);

        selectedReef = reef;
    }

    public void setSelectedLevel(ElevatorConstants.ElevatorLevel level) {
        String name;

        if(selectedLevel != null){
            SmartDashboard.putBoolean("Level " + selectedLevel.ordinal(), false);
        }

        if(level != null){
            SmartDashboard.putBoolean("Level " + level.ordinal(), true);
            name = level.name();
        }
        else name = "None";

        selectedLevel = level;

        Logger.recordOutput("Selection/Level", name);
    }

}

