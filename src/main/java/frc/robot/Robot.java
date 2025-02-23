// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.util.Elastic;
import frc.util.LocalADStarAK;
import frc.util.MarinersController.ControllerMaster;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.wpilog.WPILOGWriter.AdvantageScopeOpenBehavior;

import java.util.List;

public class Robot extends LoggedRobot
{
    private Command autonomousCommand;
    private static final Field2d field = new Field2d();
    public static boolean isRedAlliance = false;
    private static AprilTagFieldLayout apriltagField;

    private final Command moveToBottom;
    
    @SuppressWarnings("resource")
    public Robot() {
        Logger.recordMetadata("Robot Type", Constants.ROBOT_TYPE.name());

        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            //noinspection DataFlowIssue
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        if(isReal()){
            new PowerDistribution(1, ModuleType.kRev);
            if(Constants.ROBOT_TYPE == Constants.RobotType.DEVELOPMENT){
                Logger.addDataReceiver(new NT4Publisher());
                Logger.addDataReceiver(new WPILOGWriter("/media/logs"));
            } 
            else{
                Logger.addDataReceiver(new WPILOGWriter("/U"));
            }
        }
        else{
            if(Constants.ROBOT_TYPE == Constants.RobotType.REPLAY){
                String logPath = LogFileUtil.findReplayLog();
                ControllerMaster.getInstance().stopLoop();

                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"), AdvantageScopeOpenBehavior.ALWAYS));
                Logger.setReplaySource(new WPILOGReader(logPath));

                setUseTiming(false);
            }

            Logger.addDataReceiver(new NT4Publisher());
        }


        SignalLogger.enableAutoLogging(false);
        DataLogManager.stop();

        Logger.start();
        Logger.recordOutput("Zero 3D", new Pose3d());

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((path) ->
                Logger.recordOutput("PathPlanner/ActivePath", path.toArray(new Pose2d[0])));

        PathPlannerLogging.setLogTargetPoseCallback((targetPose) ->
                Logger.recordOutput("PathPlanner/TargetPose", targetPose));

        PathfindingCommand.warmupCommand().schedule();


        ControllerMaster.getInstance();

        if(Constants.ROBOT_TYPE != Constants.RobotType.COMPETITION){
            checkFlip();
            isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        }

        SmartDashboard.putData("Field", field);
        apriltagField = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        Logger.recordOutput("Zero 3D", new Pose3d());

        new RobotContainer();

        moveToBottom = new MoveToLevel(RobotContainer.elevator, ElevatorConstants.ElevatorLevel.Bottom);
    }

    private static void checkFlip() {
        boolean isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        // Constants.FeederLocation.checkAlliance(!isRedAlliance);
        Constants.ReefLocation.checkAlliance(!isRedAlliance);
    }

    public static void setRobotPoseField(Pose2d pose) {
        if(isRedAlliance){
            pose = new Pose2d(apriltagField.getFieldLength() - pose.getX(),
                    apriltagField.getFieldWidth() - pose.getY(),
                    pose.getRotation().plus(Rotation2d.k180deg));
        }
        field.setRobotPose(pose);
    }

    public static void setObjectPoseFiled(String name, Pose2d pose) {
        if(isRedAlliance){
            pose = new Pose2d(apriltagField.getFieldLength() - pose.getX(),
                    apriltagField.getFieldWidth() - pose.getY(),
                    pose.getRotation().plus(Rotation2d.k180deg));
        }
        field.getObject(name).setPose(pose);
    }

    public static void clearObjectPoseField(String name) {
        field.getObject(name).setPoses();
    }

    public static void setTrajectoryField(String name, List<Pose2d> poses) {
        field.getObject(name).setPoses(poses);
    }
    
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Robot Velocity", RobotContainer.driveBase.getVelocity());
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
    }
    
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {
    }
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void disabledExit() {}

    private void resetEncoders(){
        if(RobotContainer.elevator != null) RobotContainer.elevator.resetMotorEncoder();
        if(RobotContainer.endEffector != null) RobotContainer.endEffector.resetFunnelEncoder();
        if(RobotContainer.ballDropping != null) RobotContainer.ballDropping.resetAngleEncoder();
    }
    
    @Override
    public void autonomousInit()
    {
        if(Constants.ROBOT_TYPE == Constants.RobotType.COMPETITION){
            checkFlip();
            resetEncoders();
            if(RobotContainer.endEffector != null) RobotContainer.endEffector.setLoadedValue(true);
        }

        isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        autonomousCommand = RobotContainer.getAutoCommand();

        Elastic.selectTab(1);
        
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
    }
    
    
    @Override
    public void autonomousPeriodic() {}
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void autonomousExit() {}
    
    
    @Override
    public void teleopInit()
    {
        Elastic.selectTab(0);

        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
        moveToBottom.schedule();
    }
    
    
    @Override
    public void teleopPeriodic() {}
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void teleopExit() {}
    
    
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
        // new InstantCommand(() -> RobotContainer.elevator.resetMotorEncoder()).schedule();
    }
    
    
    @Override
    public void testPeriodic() {}
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void testExit() {}
}
