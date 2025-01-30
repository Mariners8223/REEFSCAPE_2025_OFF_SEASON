// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.ReefLocation;
import frc.robot.commands.MasterCommand.MasterCommand;
import frc.robot.commands.MasterCommand.PathPlannerWrapper;
import frc.robot.subsystems.BallDropping.BallDropping;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.RobotAuto.RobotAuto;

import frc.robot.subsystems.Vision.Vision;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.DriveTrain.DriveBaseSYSID;


public class RobotContainer {
    public static DriveBase driveBase;
    public static Elevator elevator;
    public static EndEffector endEffector;
    public static BallDropping ballDropping;
    public static RobotAuto robotAuto;
    public static Vision vision;

    public static Field2d field;
    public static LoggedDashboardChooser<Command> autoChooser;
    public static DriveBaseSYSID driveBaseSYSID;

    public static CommandXboxController driveController;
    public static CommandGenericHID operatorController;

    public RobotContainer() {
        driveController = new CommandXboxController(0);
        operatorController = new CommandPS4Controller(1);

        driveBase = new DriveBase();
        elevator = new Elevator();
        endEffector = new EndEffector();
        ballDropping = new BallDropping();
        robotAuto = new RobotAuto(driveBase, elevator, endEffector);
        vision = new Vision(driveBase::addVisionMeasurement);

        driveBaseSYSID = new DriveBaseSYSID(driveBase, driveController);

        field = new Field2d();
        SmartDashboard.putData(field);

        configChooser();
        configNamedCommands();
        configureDriveBindings();
        configureOperatorBinding();
    }

    public static void configureOperatorBinding() {
        //port 2-12 + A0 are reefs in order (2 to A0) (2 is 1, 3 is 2 and so on) (A0 is 12)
        //port A1 is for ball dropping
        //ports A2-A5 are elevator levels (A2 is L1, A3 is L2, A4 is L3, A5 is L4)

        for (int i = 0; i < 12; i++) {
            ReefLocation location = ReefLocation.values()[i];

            operatorController.button(i + 1).onTrue(new InstantCommand(() -> robotAuto.setSelectedReef(location)));
        }

        for (int i = 0; i < 4; i++) {
            ElevatorLevel level = ElevatorLevel.values()[i + 1];

            operatorController.pov(i * 90).onTrue(new InstantCommand(() -> robotAuto.setSelectedLevel(level)));
        }

        operatorController.button(12).onTrue(new InstantCommand(() ->
                robotAuto.setDropBallInCycle(!robotAuto.shouldDropBallInCycle())));
    }

    public static ReefLocation configureTargetReefSupplier() {
        int reef = (int) SmartDashboard.getNumber("target Reef", 1);

        reef = MathUtil.clamp(reef, 1, 12);

        return ReefLocation.values()[reef - 1];
    }

    public static ElevatorLevel configureLevelSupplier() {
        int level = (int) SmartDashboard.getNumber("target Level", 1);

        level = MathUtil.clamp(level, 1, 4);

        return ElevatorLevel.values()[level];
    }


    public static Boolean configureBallDropSupplier() {
        return SmartDashboard.getBoolean("should drop ball", false);
    }

    public static void configureDriveBindings() {
        driveController.start().onTrue(driveBase.resetOnlyDirection());

        Command masterCommand = new MasterCommand(
                driveBase, elevator, endEffector, ballDropping,
                robotAuto::getSelectedLevel, robotAuto::getSelectedReef, robotAuto::shouldDropBallInCycle);

        Command resetSelection = new InstantCommand(() -> {
            robotAuto.setSelectedLevel(null);
            robotAuto.setSelectedReef(null);
            robotAuto.setDropBallInCycle(false);
        });

        BooleanSupplier isCycleReady = () ->
                robotAuto.getSelectedReef() != null && robotAuto.getSelectedLevel() != null && endEffector.isGpLoaded();

        new Trigger(isCycleReady).onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0.25)),
                new WaitCommand(0.5),
                new InstantCommand(() -> driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0))
        ));

        driveController.x().whileTrue(masterCommand.onlyIf(isCycleReady));
        driveController.x().onFalse(resetSelection.onlyIf(() -> !endEffector.isGpLoaded()));

        driveController.b().onTrue(new InstantCommand(() -> {
            RobotContainer.robotAuto.setSelectedReef(RobotContainer.configureTargetReefSupplier());
            RobotContainer.robotAuto.setSelectedLevel(RobotContainer.configureLevelSupplier());
            RobotContainer.robotAuto.setDropBallInCycle(RobotContainer.configureBallDropSupplier());
        }));

        Supplier<Pose2d> rightFeeder = Constants.FeederLocation.RIGHT::getRobotPose;
        Supplier<Pose2d> leftFeeder = Constants.FeederLocation.LEFT::getRobotPose;

        driveController.rightBumper().whileTrue(new PathPlannerWrapper(driveBase, rightFeeder));
        driveController.leftBumper().whileTrue(new PathPlannerWrapper(driveBase, leftFeeder));
    }

    public static void configNamedCommands() {
        NamedCommands.registerCommand("intake", new InstantCommand());
        NamedCommands.registerCommand("eject l1", new InstantCommand());
        NamedCommands.registerCommand("eject l2", new InstantCommand());
        NamedCommands.registerCommand("eject l3", new InstantCommand());
        NamedCommands.registerCommand("eject l4", new InstantCommand());
        NamedCommands.registerCommand("ball drop l2", new InstantCommand());
        NamedCommands.registerCommand("ball drop l3", new InstantCommand());
        NamedCommands.registerCommand("ball drop off", new InstantCommand());
    }


    public static Command getAutoCommand() {
        return autoChooser.get();
    }

    private static final BooleanSupplier checkForPathChoiceUpdate = new BooleanSupplier() {
        private String lastAutoName = "InstantCommand";

        @Override
        public boolean getAsBoolean() {
            if (autoChooser.get() == null) return false;

            String currentAutoName = autoChooser.get().getName();

            try {
                return !Objects.equals(lastAutoName, currentAutoName);
            } finally {
                lastAutoName = currentAutoName;
            }

        }
    };

    private void configChooser() {
        List<String> namesOfAutos = AutoBuilder.getAllAutoNames();
        List<PathPlannerAuto> autosOfAutos = new ArrayList<>();

        autoChooser = new LoggedDashboardChooser<>("chooser");
        for (String autoName : namesOfAutos) {
            PathPlannerAuto auto = new PathPlannerAuto(autoName);
            autosOfAutos.add(auto);
        }

        autosOfAutos.forEach(auto -> autoChooser.addOption(auto.getName(), auto));

        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
        SmartDashboard.putData("chooser", autoChooser.getSendableChooser());

        new Trigger(RobotState::isEnabled).and(RobotState::isTeleop).onTrue(new InstantCommand(() -> field.getObject("AutoPath").setPoses()).ignoringDisable(true));
        new Trigger(RobotState::isDisabled).and(checkForPathChoiceUpdate).onTrue(new InstantCommand(() -> updateFieldFromAuto(autoChooser.get().getName())).ignoringDisable(true));
    }

    private static void updateFieldFromAuto(String autoName) {
        List<Pose2d> poses = new ArrayList<>();

        try {
            boolean invert =
                    DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

            PathPlannerAuto.getPathGroupFromAutoFile(autoName).forEach(path -> {
                path = invert ? path.flipPath() : path;

                poses.addAll(path.getPathPoses());
            });
        } catch (IOException | ParseException e) {
            DriverStation.reportError("Error loading auto path", e.getStackTrace());
        }

        field.getObject("AutoPath").setPoses(poses);
    }
}
