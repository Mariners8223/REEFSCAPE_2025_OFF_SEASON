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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.ReefLocation;
import frc.robot.commands.BallDropping.BallDropOff;
import frc.robot.commands.BallDropping.BallDropOnForHigh;
import frc.robot.commands.BallDropping.BallDropOnForLow;
import frc.robot.commands.BallDropping.Sequence.BallDropHigh;
import frc.robot.commands.BallDropping.Sequence.BallDropLow;
import frc.robot.commands.Climb.ClimbCommand;
import frc.robot.commands.Drive.RobotRelativeDrive;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.EndEffector.Eject;
import frc.robot.commands.EndEffector.MiniEject;
import frc.robot.commands.EndEffector.Funnel.ToggleFunnel;
import frc.robot.commands.EndEffector.Intake.Intake;
import frc.robot.commands.MasterCommand.HomeToReef;
import frc.robot.commands.MasterCommand.MasterCommand;
import frc.robot.commands.MasterCommand.PathPlannerWrapper;
import frc.robot.subsystems.BallDropping.BallDropping;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants.MotorPower;
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
    public static Climb climb;

    public static LoggedDashboardChooser<Command> autoChooser;
    public static DriveBaseSYSID driveBaseSYSID;

    public static CommandXboxController driveController;
    public static CommandGenericHID operatorController;

    public RobotContainer() {
        driveController = new CommandXboxController(0);
        operatorController = new CommandGenericHID(1);

        driveBase = new DriveBase();
        elevator = new Elevator();
        endEffector = new EndEffector();
        ballDropping = new BallDropping();
        climb = new Climb();
        robotAuto = new RobotAuto();
        vision = new Vision(driveBase::addVisionMeasurement, driveBase::getPose);

        endEffector.setDefaultCommand(new Intake(endEffector));

        configNamedCommands();
        configChooser();

        configureDriveBindings();
        configureOperatorBinding();

        //until we have real driver station
        // SmartDashboard.putNumber("target Reef", 1);
        // SmartDashboard.putNumber("target Level", 2);
        // SmartDashboard.putBoolean("should drop ball", false);


        // driveController.b().onTrue(new InstantCommand(() -> {
        //     RobotContainer.robotAuto.setSelectedReef(RobotContainer.configureTargetReefSupplier());
        //     RobotContainer.robotAuto.setSelectedLevel(RobotContainer.configureLevelSupplier());
        //     RobotContainer.robotAuto.setDropBallInCycle(RobotContainer.configureBallDropSupplier());
        //     System.out.println("set new targets");
        // }));
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

        operatorController.button(13).onTrue(new InstantCommand(() ->
                robotAuto.setDropBallInCycle(!robotAuto.shouldDropBallInCycle())));

        //funnel flipping
        operatorController.povUpRight().onTrue(new ToggleFunnel(endEffector));

        //climb
        operatorController.povDownLeft().whileTrue(new ClimbCommand(climb).onlyIf(() ->
                Timer.getMatchTime() <= 30 && endEffector.getFunnelPosition() < -0.4));

        //manual intake
        operatorController.povDownLeft().whileTrue(new MiniEject(endEffector).onlyIf(() ->
                endEffector.getFunnelPosition() > -0.4));
    }

    public static ReefLocation configureTargetReefSupplier() {
        int reef = (int) SmartDashboard.getNumber("target Reef", 1);

        reef = MathUtil.clamp(reef, 1, 12);

        return ReefLocation.values()[reef - 1];
    }

    public static ElevatorLevel configureLevelSupplier() {
        double shit = SmartDashboard.getNumber("target Level", 1);
        System.out.println("woop woop " + shit);
        int level = (int) shit;

        level = MathUtil.clamp(level, 1, 4);

        System.out.println("new selcted level: " + level);

        return ElevatorLevel.values()[level];
    }


    public static Boolean configureBallDropSupplier() {
        return SmartDashboard.getBoolean("should drop ball", false);
    }

    public static void configureDriveBindings() {
        BooleanSupplier isCycleReady = () ->
                robotAuto.getSelectedReef() != null && robotAuto.getSelectedLevel() != null && endEffector.isGpLoaded();

        Command masterCommand = new MasterCommand(
                driveBase, elevator, endEffector, ballDropping,
                robotAuto::getSelectedLevel, robotAuto::getSelectedReef, robotAuto::shouldDropBallInCycle);

        Command resetSelection = new InstantCommand(() -> {
            robotAuto.setSelectedLevel(null);
            robotAuto.setSelectedReef(null);
            robotAuto.setDropBallInCycle(false);
        });
        
      Supplier<Pose2d> rightFeeder = Constants.FeederLocation.RIGHT::getRobotPose;
        Supplier<Pose2d> leftFeeder = Constants.FeederLocation.LEFT::getRobotPose;


        // main cycle
        driveController.leftTrigger().whileTrue(masterCommand.onlyIf(isCycleReady));
        driveController.leftTrigger().onFalse(resetSelection.onlyIf(() -> !endEffector.isGpLoaded()));

        // feeder path finder
        driveController.rightBumper().whileTrue(new PathPlannerWrapper(driveBase, rightFeeder));
        driveController.leftBumper().whileTrue(new PathPlannerWrapper(driveBase, leftFeeder));

        //manual cycle
        // driveController.x().onTrue(new ManualCycleCommand(endEffector, elevator, robotAuto::getSelectedLevel).onlyIf(isCycleReady));
        MoveToLevel moveToLevel = new MoveToLevel(elevator, ElevatorLevel.L1);
        Eject ejectCommand = new Eject(endEffector, MotorPower.L1);

        driveController.x().onTrue(
                new InstantCommand(() -> {
                    moveToLevel.changeDesiredlevel(robotAuto.getSelectedLevel());
                    ejectCommand.setLevel(MasterCommand.getMotorPower(robotAuto.getSelectedLevel()));
                }
                ).andThen(
                        moveToLevel
                ).onlyIf(isCycleReady)
        );

        driveController.x().whileTrue(new RobotRelativeDrive(driveBase, driveController));

        resetSelection = new InstantCommand(() -> {
            robotAuto.setSelectedLevel(null);
            robotAuto.setSelectedReef(null);
            robotAuto.setDropBallInCycle(false);
        });

        Command ejectSequence = new SequentialCommandGroup(
                ejectCommand,
                new MoveToLevel(elevator, ElevatorLevel.Bottom)
                //resetSelection
        ).onlyIf(isCycleReady);

        driveController.x().onFalse(ejectSequence);


        //ball dropping manual control
        driveController.povUp().whileTrue(new BallDropOnForHigh(ballDropping));
        driveController.povDown().whileTrue(new BallDropOnForLow(ballDropping));

        driveController.povUp().onFalse(new BallDropOff(ballDropping));
        driveController.povDown().onFalse(new BallDropOff(ballDropping));

        driveController.start().onTrue(driveBase.resetOnlyDirection());

        new Trigger(isCycleReady).onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0.25)),
                new WaitCommand(0.5),
                new InstantCommand(() -> driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0))
        ));


        //driveController.y().whileTrue(new HomeToReef(driveBase, ReefLocation.REEF_1));
    }

    public static void configNamedCommands() {
        NamedCommands.registerCommand("intake", new MoveToLevel(elevator, ElevatorLevel.Bottom)
                .andThen(new Intake(endEffector)));

        NamedCommands.registerCommand("eject l1", new MoveToLevel(elevator, ElevatorLevel.L1)
                .andThen(new Eject(endEffector, MotorPower.L1), (new MoveToLevel(elevator, ElevatorLevel.Bottom))));

        NamedCommands.registerCommand("eject l2", new MoveToLevel(elevator, ElevatorLevel.L2)
                .andThen(new Eject(endEffector, MotorPower.L2_3), (new MoveToLevel(elevator, ElevatorLevel.Bottom))));

        NamedCommands.registerCommand("eject l3", new MoveToLevel(elevator, ElevatorLevel.L3)
                .andThen(new Eject(endEffector, MotorPower.L2_3), (new MoveToLevel(elevator, ElevatorLevel.Bottom))));

        NamedCommands.registerCommand("eject l4", new MoveToLevel(elevator, ElevatorLevel.L4)
                .andThen(new Eject(endEffector, MotorPower.L4), (new MoveToLevel(elevator, ElevatorLevel.Bottom))));

        NamedCommands.registerCommand("ball drop l2", new BallDropLow(ballDropping));
        NamedCommands.registerCommand("ball drop l3", new BallDropHigh(ballDropping));
        NamedCommands.registerCommand("ball drop off", new BallDropOff(ballDropping));
        NamedCommands.registerCommand("reset elevator", new MoveToLevel(elevator, ElevatorLevel.Bottom));

        for (ReefLocation reef : ReefLocation.values()) {
            NamedCommands.registerCommand("home to reef " + (reef.ordinal() + 1), new HomeToReef(driveBase, reef));
        }

        NamedCommands.registerCommand("Wait until GP", new WaitUntilCommand(endEffector::isGpLoaded));
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

        new Trigger(RobotState::isEnabled).and(RobotState::isTeleop).onTrue(new InstantCommand(() -> Robot.clearObjectPoseField("AutoPath")).ignoringDisable(true));
        new Trigger(RobotState::isDisabled).and(checkForPathChoiceUpdate).onTrue(new InstantCommand(() -> updateFieldFromAuto(autoChooser.get().getName())).ignoringDisable(true));
    }

    private static void updateFieldFromAuto(String autoName) {
        List<Pose2d> poses = new ArrayList<>();

        try {
            // boolean invert =
            //         DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

            PathPlannerAuto.getPathGroupFromAutoFile(autoName).forEach(path -> {
//                path = invert ? path.flipPath() : path;
                //no need cause the field inverts the path

                poses.addAll(path.getPathPoses());
            });
        } catch (IOException | ParseException e) {
            DriverStation.reportError("Error loading auto path", e.getStackTrace());
        }

        Robot.setTrajectoryField("AutoPath", poses);
    }
}
