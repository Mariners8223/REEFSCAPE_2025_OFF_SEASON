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
import frc.util.Elastic;
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

import java.util.List;

public class Robot extends LoggedRobot
{
    private Command autonomousCommand;
    private static final Field2d field = new Field2d();
    public static boolean isRedAlliance = false;
    private static AprilTagFieldLayout apriltagField;

    private int driverStationCheckTimer = 0;
    private boolean ledState = true;
    
    @SuppressWarnings({ "resource", "incomplete-switch" })
    public Robot() {
        Logger.recordMetadata("Robot Type", Constants.ROBOT_TYPE.name());

        Logger.addDataReceiver(new WPILOGWriter("/media/logs"));

        if(isReal()){
            switch (Constants.ROBOT_TYPE){
                case DEVELOPMENT -> {
                    Logger.addDataReceiver(new NT4Publisher());
                    break;
                }

                // case COMPETITION -> {
                //     Logger.addDataReceiver(new WPILOGWriter("/U"));
                // }

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
        Logger.recordOutput("Bumper Pose", new Pose3d());

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


        Logger.recordOutput("Zero 3D", new Pose3d());

        new RobotContainer();

        SmartDashboard.putBoolean("LED on", true);
    }

    private static void checkFlip() {
        boolean isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;


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
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
        SmartDashboard.putNumber("PDH Voltage", ConduitApi.getInstance().getPDPVoltage());
        // Logger.recordOutput("LED power draw", pdh.getCurrent(23) * pdh.getVoltage());
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

        
        if (SmartDashboard.getBoolean("LED on", true) != ledState) {
            ledState = !ledState;

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



        Elastic.selectTab(0);
        Logger.recordOutput("Elastic Tab", "auto");

        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }


        
        ledState = true;

        SmartDashboard.putBoolean("LED on", true);


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
        Elastic.selectTab(1);
        Logger.recordOutput("Elastic Tab", "Telop");


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
