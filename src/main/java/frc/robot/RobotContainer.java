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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.FeederLocation;
import frc.robot.Constants.ReefLocation;
import frc.robot.commands.BallDropping.BallDropOff;
import frc.robot.commands.BallDropping.BallDropOnForHigh;
import frc.robot.commands.BallDropping.BallDropOnForLow;
import frc.robot.commands.BallDropping.Sequence.BallDropHigh;
import frc.robot.commands.BallDropping.Sequence.BallDropLow;
import frc.robot.commands.Climb.ClimbCommand;
import frc.robot.commands.Drive.MinorAdjust;
import frc.robot.commands.Drive.MinorAdjust.Direcation;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.Elevator.MoveToLevelActive;
import frc.robot.commands.EndEffector.Eject;
import frc.robot.commands.EndEffector.Funnel.MoveFunnel;
import frc.robot.commands.EndEffector.MiniEject;
import frc.robot.commands.EndEffector.Funnel.ToggleFunnel;
import frc.robot.commands.EndEffector.Intake.Intake;
import frc.robot.commands.MasterCommand.*;
import frc.robot.subsystems.BallDropping.BallDropping;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;
import frc.robot.subsystems.EndEffector.EndEffectorConstants.MotorPower;
import frc.robot.subsystems.RobotAuto.RobotAuto;

import frc.robot.subsystems.Vision.Vision;
import frc.util.Elastic;

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

        if(Constants.ROBOT_TYPE == Constants.RobotType.COMPETITION){
            new Trigger(DriverStation::isDSAttached).onTrue(
                new InstantCommand(() -> Elastic.selectTab(1)).ignoringDisable(true)
            );
        }

        configNamedCommands();
        configChooser();

        configureDriveBindings();
        configureOperatorBinding();

        //  configureCamera();
        if (RobotBase.isReal()) {
            CameraServer.startAutomaticCapture();
            CameraServer.getServer().getSource().setFPS(15);
        } else {
            //until we have real driver station
            SmartDashboard.putNumber("target Reef", 1);
            SmartDashboard.putNumber("target Level", 2);
            SmartDashboard.putBoolean("should drop ball", false);


            driveController.y().onTrue(new InstantCommand(() -> {
                RobotContainer.robotAuto.setSelectedReef(RobotContainer.configureTargetReefSupplier());
                RobotContainer.robotAuto.setSelectedLevel(RobotContainer.configureLevelSupplier());
                System.out.println("set new targets");
            }));
        }

        HomeToReef.pidTune();
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

        // operatorController.button(13).onTrue(new InstantCommand(() ->
        //         robotAuto.setDropBallInCycle(!robotAuto.shouldDropBallInCycle())));
        operatorController.button(13).whileTrue(new BallDropOnForLow(ballDropping)).and(() -> !robotAuto.getSelectedReef().isBallInUpPosition()).or(() -> robotAuto.getSelectedReef() == null);
        operatorController.button(13).whileTrue(new BallDropOnForHigh(ballDropping)).and(() -> robotAuto.getSelectedReef().isBallInUpPosition());
        operatorController.button(13).onFalse(new BallDropOff(ballDropping));

        //funnel flipping
        operatorController.axisLessThan(0, -0.5).and(() -> !endEffector.isGpLoaded()).onTrue(new ToggleFunnel(endEffector));

        //climb
        operatorController.axisLessThan(2, -0.5).and(() ->
                Timer.getMatchTime() <= 30 && endEffector.getFunnelPosition() < -0.4).whileTrue(new ClimbCommand(climb));
        // operatorController.povDownLeft().whileTrue(new ClimbCommand(climb));

        //manual intake
        operatorController.axisLessThan(2, -0.5).and(() ->
                endEffector.getFunnelPosition() > -0.4).whileTrue(new MiniEject(endEffector, elevator::getCurrentLevel));

        new Trigger(() -> Timer.getMatchTime() <= 30).and(() -> isRobotInClimbArea() && !endEffector.isGpLoaded())
                .onTrue(new MoveFunnel(endEffector, EndEffectorConstants.FunnelMotor.CLIMB_POSITION));
    }

    private static boolean isRobotInClimbArea(){
        Pose2d pose = driveBase.getPose();

        return pose.getX() > 7.5 && pose.getX() < 8.5;
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

    public static void configureDriveBindings() {
        BooleanSupplier isCycleReady = () ->
                robotAuto.getSelectedReef() != null && robotAuto.getSelectedLevel() != null && endEffector.isGpLoaded();

        EventTrigger moveElevatorMarker = new EventTrigger("move to selected level");

        Command masterCommand = new MasterCommand(
                driveBase, elevator, endEffector, moveElevatorMarker, robotAuto::getSelectedLevel, robotAuto::getSelectedReef);

        Command semiAutoCommand = new SemiAuto(driveBase, elevator, robotAuto::getSelectedReef,
                robotAuto::getSelectedLevel, moveElevatorMarker, driveController);

//        Command moveElevatorToBottom = new MoveToLevel(elevator, ElevatorLevel.Bottom);

        Command resetSelection = new InstantCommand(() -> {
            robotAuto.setSelectedLevel(null);
            robotAuto.setSelectedReef(null);
        });

        BooleanSupplier robotBelowCertainSpeed = () -> {
            // speed is below 1 m/s total and below 1 omega
            return driveBase.getVelocity() < 1;
        };

        Command resetSelectionAdvanced = resetSelection.onlyIf(() -> !endEffector.isGpLoaded());

        Trigger mainCycleTrigger = driveController.leftTrigger();
        Trigger leftFeeder = driveController.leftBumper();
        Trigger rightFeeder = driveController.rightBumper();

        Trigger moveElevator = driveController.x();
        Trigger onlyRobotToReef = driveController.b();

        Trigger semiAuto = driveController.a();

        // main cycle
        mainCycleTrigger.and(isCycleReady).whileTrue(masterCommand);
        mainCycleTrigger.onFalse(resetSelectionAdvanced.andThen(new MoveToLevel(elevator, ElevatorLevel.Bottom)));

        // feeder path finder
        rightFeeder.whileTrue(new PathPlannerWrapper(driveBase, FeederLocation.RIGHT));
        leftFeeder.whileTrue(new PathPlannerWrapper(driveBase, FeederLocation.LEFT));

        onlyRobotToReef.and(() -> robotAuto.getSelectedReef() != null)
                .whileTrue(new RobotToReef(driveBase, robotAuto::getSelectedReef));

        moveElevator.whileTrue(
                        new MoveToLevelActive(elevator, robotAuto::getSelectedLevel)
                .onlyIf(() -> robotAuto.getSelectedLevel() != null && endEffector.isGpLoaded() && robotBelowCertainSpeed.getAsBoolean())
        );


        semiAuto.and(isCycleReady).whileTrue(semiAutoCommand);
        semiAuto.onFalse(new MoveToLevel(elevator, ElevatorLevel.Bottom));

        driveController.start().onTrue(driveBase.resetOnlyDirection());

        new Trigger(isCycleReady).onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0.25)),
                new WaitCommand(0.5),
                new InstantCommand(() -> driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0))
        ));

        driveController.povRight().whileTrue(new MinorAdjust(driveBase, Direcation.RIGHT));
        driveController.povLeft().whileTrue(new MinorAdjust(driveBase, Direcation.LEFT));
        driveController.povUp().whileTrue(new MinorAdjust(driveBase, Direcation.FORWARD));
        driveController.povDown().whileTrue(new MinorAdjust(driveBase, Direcation.BACKWARDS));
    }

    public static void configNamedCommands() {
        NamedCommands.registerCommand("move l1", new MoveToLevel(elevator, ElevatorLevel.L1));

        NamedCommands.registerCommand("move l2", new MoveToLevel(elevator, ElevatorLevel.L2));

        NamedCommands.registerCommand("move l3", new MoveToLevel(elevator, ElevatorLevel.L3));

        NamedCommands.registerCommand("move l4", new MoveToLevel(elevator, ElevatorLevel.L4));

        Eject eject = new Eject(endEffector, MotorPower.L1);
        NamedCommands.registerCommand("eject", new SequentialCommandGroup(
                new InstantCommand(() -> eject.setLevel(MasterCommand.getMotorPower(elevator.getCurrentLevel()))),
                eject,
                new MoveToLevel(elevator, ElevatorLevel.Bottom)
        ));

        NamedCommands.registerCommand("ball drop l2", new BallDropLow(ballDropping));
        NamedCommands.registerCommand("ball drop l3", new BallDropHigh(ballDropping));
        NamedCommands.registerCommand("ball drop off", new BallDropOff(ballDropping));
        NamedCommands.registerCommand("reset elevator", new MoveToLevel(elevator, ElevatorLevel.Bottom));

        for (ReefLocation reef : ReefLocation.values()) {
            HomeToReef homeToReef = new HomeToReef(driveBase, reef, ElevatorLevel.Bottom);

            NamedCommands.registerCommand("home to reef " + (reef.ordinal() + 1),
                    homeToReef.beforeStarting(() -> homeToReef.setTargetPose(reef, elevator.getDesiredLevel())));
        }

        NamedCommands.registerCommand("Wait until GP", new Intake(endEffector));
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
