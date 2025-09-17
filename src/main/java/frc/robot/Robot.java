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
import frc.util.LocalADStarAK;
import frc.util.MarinersController.ControllerMaster;

import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.SensorUtil;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.wpilog.WPILOGWriter.AdvantageScopeOpenBehavior;

import java.util.List;

public class Robot extends LoggedRobot
{
    private Command autonomousCommand;
    public static boolean isRedAlliance = false;

    private int driverStationCheckTimer = 0;
    private boolean ledState = true;
    public Robot(){
        new RobotContainer();
    }
    
    private static void checkFlip() {
        boolean isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;


    }

    
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
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


        
        ledState = true;



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
