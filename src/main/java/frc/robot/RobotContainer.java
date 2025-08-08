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
import frc.robot.commands.TestMotorSpinCommand;
import frc.robot.subsystems.MotorTesting;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class RobotContainer {
    public static MotorTesting motorTesting;

    public static CommandXboxController driveController;

    public RobotContainer() {
        driveController = new CommandXboxController(0);

        motorTesting = new MotorTesting();
        driveController.a().whileTrue(new TestMotorSpinCommand(motorTesting, -1));
        driveController.y().whileTrue(new TestMotorSpinCommand(motorTesting, 1));

    }

//    configureDriveBindings();
//
//
//    public void configureDriveBindings(){
//        driveController.a().whileTrue(new TestMotorSpinCommand(motorTesting));
//    }


}
