
import java.security.PrivateKey;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveBase;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonTrackedTarget;;


public class vision_i_like_ice_cream extends Command {
    private PhotonCamera camera = new PhotonCamera("EndEffectorCamera");
    private final DriveBase driveBase;
    double tolerance = 10; //what fucking units
    public vision_i_like_ice_cream(DriveBase driveBase)
    {
        this.driveBase = driveBase;
        addRequirements(this.driveBase);
    }
    
    @Override
    public void initialize() {

    }   

    @Override
    public void execute() {
        var visionResult = camera.getLatestResult();

        PhotonTrackedTarget visionTarget = visionResult.getBestTarget();

        if(visionResult.hasTargets())
        {
            if0(visionTarget.getFiducialId() == 7) 
            {
                driveBase.drive(new ChassisSpeeds(0,0,0));
                if (visionTarget.getArea() < 75) // change units
                {
                    if (Math.abs(visionTarget.getYaw()) <= tolerance) //i need units
                    {
                        driveBase.drive(new ChassisSpeeds(1.5,0,0));
                    }
                    else 
                    {
                        driveBase.drive(new ChassisSpeeds(0,0,1));
                        
                    }
                }
            }
            else return;
        }

        else driveBase.drive(new ChassisSpeeds(0,0,1));  
        


        //בודק אם יש תג. אם לא אז מסתובב
        //אם זה תג מספר 7 אז עוצר את התזוזה אחרת חוזר 
        //אחרי זה בודק אם השטח שהתג טופס במצלמה הוא פחות מ75%
        //אם כן, הוא מסתובב עד שנכנס לטווח של הטולרנס
        //

    }

    public void end() {
        driveBase.drive(new ChassisSpeeds(0,0,0));
    }

    public boolean isFinished() {

        return false;
    }
}