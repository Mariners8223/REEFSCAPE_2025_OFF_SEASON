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
    private boolean dropBallInCycle = false;
    
    public RobotAuto() {
        SmartDashboard.putBoolean("Level 1", false);
        SmartDashboard.putBoolean("Level 2", false);
        SmartDashboard.putBoolean("Level 3", false);
        SmartDashboard.putBoolean("Level 4", false);

        SmartDashboard.putBoolean("drop ball", false);
    }

    public Constants.ReefLocation getSelectedReef() {
        return selectedReef;
    }

    public ElevatorConstants.ElevatorLevel getSelectedLevel() {
        return selectedLevel;
    }

    public boolean shouldDropBallInCycle() {
        return dropBallInCycle;
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
        if(level != null && level != selectedLevel){
            SmartDashboard.putBoolean("Level " + level.ordinal(), true);
            name = level.name();
            selectedLevel = level;

        }
        else{
            name = "None";
            selectedLevel = null;
        }

        Logger.recordOutput("Selection/Level", name);
    }

    public void setDropBallInCycle(boolean dropBall) {
        Logger.recordOutput("Selection/Should Drop Ball", dropBall);

        SmartDashboard.putBoolean("drop ball", dropBall);
        dropBallInCycle = dropBall;
    }

}

