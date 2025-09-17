// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.RobotType;
import frc.robot.commands.Drive.DriveCommand;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.SensorUtil;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.EndEffector.*;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.EndEffector.EndEffector;

public class RobotContainer {
    public static DriveBase driveBase;
    public static DigitalInput beambreak;
    public static EndEffector endEffector;
    public static CommandPS5Controller driveController;
    public static CommandXboxController driveXboxController;


    public RobotContainer() {
        driveController = new CommandPS5Controller(0);
        driveXboxController = new CommandXboxController(1);
        beambreak = new DigitalInput(8);
        driveBase = new DriveBase();
        endEffector = new EndEffector();
        configureDriveBindings();
        
    }


    public void configureDriveBindings(){
        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(new StartEndCommand(() ->
            driveBase.setDefaultCommand(new DriveCommand(driveBase, driveXboxController)),
            driveBase::removeDefaultCommand).ignoringDisable(true));
        driveController.L1().onTrue(new CoralScoring(endEffector));
        driveController.R1().onTrue(new scoring(endEffector));
        driveController.touchpad().whileFalse(new collecting(endEffector));
        
   }
   


}
