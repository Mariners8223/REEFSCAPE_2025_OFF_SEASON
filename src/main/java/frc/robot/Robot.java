// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.Vision.VisionConstants;
import frc.util.LocalADStarAK;
import frc.util.MarinersController.ControllerMaster;

import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.wpilog.WPILOGWriter.AdvantageScopeOpenBehavior;
import frc.robot.subsystems.Vision.VisionConstants;

import java.util.List;

public class Robot extends LoggedRobot
{
    private Command autonomousCommand;
    public static boolean isRedAlliance = false;
    private int driverStationCheckTimer = 0;

    private static final Field2d field = new Field2d();
    private static AprilTagFieldLayout apriltagField;

    public Robot(){
        if(Constants.ROBOT_TYPE != Constants.RobotType.COMPETITION){
            checkFlip();
            isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        }

        configureLogging();
        configurePathplanning();


        SmartDashboard.putData("Field", field);
        apriltagField = VisionConstants.FIELD_LAYOUT;

        new RobotContainer();
        ControllerMaster.getInstance();
    }

    private void configureLogging(){
        Logger.recordMetadata("Robot Type", Constants.ROBOT_TYPE.name());

        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        Logger.addDataReceiver(new WPILOGWriter("/media/logs"));

        if(isReal()){
            switch (Constants.ROBOT_TYPE){
                case DEVELOPMENT -> {
                    Logger.addDataReceiver(new NT4Publisher());
                    break;
                }

                case REPLAY -> System.out.println("Achievement Unlocked: How did we get here?");
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

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((path) ->
                Logger.recordOutput("PathPlanner/ActivePath", path.toArray(new Pose2d[0])));

        PathPlannerLogging.setLogTargetPoseCallback((targetPose) ->
                Logger.recordOutput("PathPlanner/TargetPose", targetPose));

        PathfindingCommand.warmupCommand().schedule();

    }

    private static void configurePathplanning(){
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((path) ->
                Logger.recordOutput("PathPlanner/ActivePath", path.toArray(new Pose2d[0])));

        PathPlannerLogging.setLogTargetPoseCallback((targetPose) ->
                Logger.recordOutput("PathPlanner/TargetPose", targetPose));

        PathfindingCommand.warmupCommand().schedule();
    }

    
    private static void checkFlip() {
        isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Robot Velocity", RobotContainer.driveBase.getVelocity());
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
        SmartDashboard.putNumber("PDH Voltage", ConduitApi.getInstance().getPDPVoltage());
    }
    
    
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {
        if(Constants.ROBOT_TYPE == Constants.RobotType.COMPETITION ){
            driverStationCheckTimer++;
            
            if(driverStationCheckTimer >= 50){
                driverStationCheckTimer = 0;

                isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

            }
        }

        
        }
    
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void disabledExit() {}

    private void resetEncoders(){
    }
    
    @Override
    public void autonomousInit()
    {
        if(Constants.ROBOT_TYPE == Constants.RobotType.COMPETITION){
            checkFlip();
            resetEncoders();
        }

        isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        // Pose2d resetPose = new Pose2d(RobotContainer.driveBase.getPose().getTranslation(), isRedAlliance ? new Rotation2d() : new Rotation2d(Math.PI));
        // RobotContainer.driveBase.reset(resetPose);




        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
<<<<<<< HEAD


        
        //ledState = true;



=======
>>>>>>> origin/NewChassis
    }
    
    
    @Override
    public void autonomousPeriodic() {}
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void autonomousExit() {

    }
    
    
    @Override
    public void teleopInit()
    {


        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }
    
    
    @Override
    public void teleopPeriodic() {}
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void teleopExit() {


    }
    
    
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
