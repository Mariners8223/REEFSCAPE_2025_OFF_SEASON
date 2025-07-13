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
import frc.robot.Constants.FeederLocation;
import frc.robot.Constants.FeederSide;
import frc.robot.Constants.ReefLocation;
import frc.robot.Constants.RobotType;
import frc.robot.commands.BallDropping.BallDropOff;
import frc.robot.commands.BallDropping.BallDropOnForHigh;
import frc.robot.commands.BallDropping.BallDropOnForLow;
import frc.robot.commands.BallDropping.Sequence.BallDropHigh;
import frc.robot.commands.BallDropping.Sequence.BallDropLow;
import frc.robot.commands.Climb.ClimbCommand;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Drive.MinorAdjust;
import frc.robot.commands.Drive.MinorAdjust.AdjustmentDirection;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.Elevator.MoveToLevelActive;
import frc.robot.commands.EndEffector.EjectSequance;
import frc.robot.commands.EndEffector.MiniEject;
import frc.robot.commands.EndEffector.Funnel.ToggleFunnel;
import frc.robot.commands.EndEffector.Intake.Intake;
import frc.robot.commands.MasterCommand.*;
import frc.robot.subsystems.BallDropping.BallDropping;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants.MotorPower;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LED.StripControl;
import frc.robot.subsystems.RobotAuto.RobotAuto;

import frc.robot.subsystems.Vision.Vision;
import frc.util.Elastic;

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
import frc.robot.subsystems.DriveTrain.DriveBase;


public class RobotContainer {

    public RobotContainer() {

}

}
