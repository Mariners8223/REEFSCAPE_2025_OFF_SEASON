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
    public static DriveBase driveBase;
    public static Elevator elevator;
    public static EndEffector endEffector;
    public static BallDropping ballDropping;
    public static RobotAuto robotAuto;
    public static Vision vision;
    public static Climb climb;
    public static LED led;

    public static LoggedDashboardChooser<Command> autoChooser;

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
        led = new LED();
        robotAuto = new RobotAuto();
        vision = new Vision(driveBase::addVisionMeasurement, driveBase::getPose, HomeToReef::isRunning);

        endEffector.setDefaultCommand(new Intake(endEffector));

        if(Constants.ROBOT_TYPE == Constants.RobotType.COMPETITION){
            new Trigger(DriverStation::isDSAttached).onTrue(
                new InstantCommand(() -> {
                    Elastic.selectTab(0);
                    Logger.recordOutput("Elastic Tab", "auto");
                }).ignoringDisable(true)
            );
        }

        configNamedCommands();
        configChooser();
        configLEDs();
        
        configureDriveBindings();
        configureOperatorBinding();

        //  configureCamera();
        if (RobotBase.isReal()) {
            CameraServer.startAutomaticCapture();
            CameraServer.getServer().getSource().setResolution(160, 90);
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

        if(Constants.ROBOT_TYPE == RobotType.DEVELOPMENT) HomeToReef.pidTune();

        Trigger robotReadyClimb = new Trigger(() -> Timer.getMatchTime() < 30 && endEffector.isFunnelInClimb());

        robotReadyClimb.onTrue(new InstantCommand(() -> {
            DriveCommand.halfSpeed();
            Logger.recordOutput("Elastic Tab", "EndGame");
            Elastic.selectTab(2);
        }));

        new Trigger(() -> !endEffector.isFunnelInClimb()).onTrue(new InstantCommand(() -> {
            DriveCommand.normalSpeed();
            Logger.recordOutput("Elastic Tab", "Telop");
            Elastic.selectTab(1);
        }));

        new Trigger(() -> Robot.pdh.getVoltage() > 7)
                .whileTrue(new InstantCommand(() -> Robot.pdh.setSwitchableChannel(true)).ignoringDisable(true))
                .whileFalse(new WaitCommand(10)
                        .andThen(new InstantCommand(() -> Robot.pdh.setSwitchableChannel(false)).ignoringDisable(true)));

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

        BooleanSupplier isBallDropUp = () -> {
            ReefLocation selectedReef = robotAuto.getSelectedReef();

            if(selectedReef == null) return true;

            return selectedReef.isBallInUpPosition();
        };

        operatorController.button(13).whileTrue(new SequentialCommandGroup(
            new BallDropOnForLow(ballDropping).onlyIf(() -> !isBallDropUp.getAsBoolean()),
            new BallDropOnForHigh(ballDropping).onlyIf(isBallDropUp)
        ));
        operatorController.button(13).onFalse(new BallDropOff(ballDropping));

        //funnel flipping
        operatorController.axisLessThan(0, -0.5).and(() -> !endEffector.isGpLoaded()).onTrue(new ToggleFunnel(endEffector));

        //climb
        operatorController.axisLessThan(2, -0.5).and(() ->
                Timer.getMatchTime() <= 30 && endEffector.isFunnelInClimb()).whileTrue(new ClimbCommand(climb));

        //manual intake
        operatorController.axisLessThan(2, -0.5).and(() ->
                !endEffector.isFunnelInClimb()).whileTrue(new MiniEject(endEffector, elevator::getCurrentLevel, robotAuto::getSelectedReef));
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
                driveBase, elevator, endEffector, moveElevatorMarker, robotAuto::getSelectedLevel, robotAuto::getSelectedReef, led);

        Command semiAutoCommand = new SemiAuto(driveBase, elevator, robotAuto::getSelectedReef,
                robotAuto::getSelectedLevel, moveElevatorMarker, driveController);

        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(new StartEndCommand(() ->
                driveBase.setDefaultCommand(new DriveCommand(driveBase, RobotContainer.driveController)),
                driveBase::removeDefaultCommand).ignoringDisable(true));

        BooleanSupplier robotBelowCertainSpeed = () -> {
            // speed is below 1 m/s total
            return driveBase.getVelocity() < 1;
        };

        Trigger mainCycleTrigger = driveController.leftTrigger();

        Trigger moveElevator = driveController.x();
        Trigger onlyRobotToReef = driveController.b();

        Trigger semiAuto = driveController.a();

        Trigger undoGP = driveController.y();

        setFeederBinding(true);

        // main cycle
        mainCycleTrigger.whileTrue(masterCommand.onlyIf(isCycleReady).withName("Master Command"));
        mainCycleTrigger.onFalse(new MoveToLevel(elevator, ElevatorLevel.Bottom));

        onlyRobotToReef.and(() -> robotAuto.getSelectedReef() != null)
                .whileTrue(new RobotToReef(driveBase, robotAuto::getSelectedReef));

        moveElevator.whileTrue(
                        new MoveToLevelActive(elevator, robotAuto::getSelectedLevel)
                .onlyIf(() -> robotAuto.getSelectedLevel() != null && endEffector.isGpLoaded() && robotBelowCertainSpeed.getAsBoolean())
        );

        semiAuto.and(isCycleReady).whileTrue(semiAutoCommand);
        semiAuto.onFalse(new MoveToLevel(elevator, ElevatorLevel.Bottom));

        driveController.start().onTrue(driveBase.resetOnlyDirection());

        undoGP.onTrue(new InstantCommand(() -> endEffector.setLoadedValue(false)));

        new Trigger(isCycleReady).onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0.25)),
                new WaitCommand(0.5),
                new InstantCommand(() -> driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0))
        ).ignoringDisable(true));

        driveController.povRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.RIGHT));
        driveController.povLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.LEFT));
        driveController.povUp().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FORWARD));
        driveController.povDown().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACKWARDS));
        driveController.povUpLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FRONT_LEFT));
        driveController.povUpRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FRONT_RIGHT));
        driveController.povDownLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACK_LEFT));
        driveController.povDownRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACK_RIGHT));
    }

    public static void setFeederBinding(boolean isBlueAlliance){
        final Trigger leftFeeder;
        final Trigger rightFeeder;

        if(isBlueAlliance){
            leftFeeder = driveController.leftBumper();
            rightFeeder = driveController.rightBumper();
        }
        else{
            rightFeeder = driveController.leftBumper();
            leftFeeder = driveController.rightBumper();
        }



        leftFeeder.whileTrue(new SequentialCommandGroup(
                led.blinkWithRSLCommand(!isBlueAlliance),
                driveBase.pathFindToPathAndFollow(Constants.FeederLocation.LEFT.getPath()),
                led.putDefaultPatternCommand()
        ));

        leftFeeder.onFalse(led.putDefaultPatternCommand());

        rightFeeder.whileTrue(new SequentialCommandGroup(
                led.blinkWithRSLCommand(!isBlueAlliance),
                driveBase.pathFindToPathAndFollow(Constants.FeederLocation.RIGHT.getPath()),
                led.putDefaultPatternCommand()
        ));

        rightFeeder.onFalse(led.putDefaultPatternCommand());
    }

    public static void configNamedCommands() {
        NamedCommands.registerCommand("move l1", new MoveToLevel(elevator, ElevatorLevel.L1));

        NamedCommands.registerCommand("move l2", new MoveToLevel(elevator, ElevatorLevel.L2));

        NamedCommands.registerCommand("move l3", new MoveToLevel(elevator, ElevatorLevel.L3));

        NamedCommands.registerCommand("move l4", new MoveToLevel(elevator, ElevatorLevel.L4));

        EjectSequance eject = new EjectSequance(endEffector, elevator);

        NamedCommands.registerCommand("eject", eject.beforeStarting(() ->
                eject.setLevel(MotorPower.L4)));

        NamedCommands.registerCommand("ball drop l2", new BallDropLow(ballDropping));
        NamedCommands.registerCommand("ball drop l3", new BallDropHigh(ballDropping));
        NamedCommands.registerCommand("ball drop off", new BallDropOff(ballDropping));
        NamedCommands.registerCommand("reset elevator", new MoveToLevel(elevator, ElevatorLevel.Bottom));

        for (ReefLocation reef : ReefLocation.values()) {
            HomeToReef homeToReef = new HomeToReef(driveBase, reef);

            // .onlyIf(homeToReef::isOutOfTolarance))

            NamedCommands.registerCommand("home to reef " + (reef.ordinal() + 1), homeToReef);

            NamedCommands.registerCommand("set target reef " + (reef.ordinal() + 1), 
                new InstantCommand(() -> robotAuto.setSelectedReef(reef)));
        }

        NamedCommands.registerCommand("Wait until GP", new Intake(endEffector));

        EventTrigger moveElevatorMarker = new EventTrigger("move to selected level");

        NamedCommands.registerCommand("wait to move elevator", new WaitUntilCommand(moveElevatorMarker));
    }

    private static void configLEDs(){
        led.setStripControl(StripControl.ALL);
        led.setDefaultPattern(Robot.isRedAlliance);
        led.putDefaultPattern();

        Command gpLoadedCommand = new SequentialCommandGroup(
                led.setStripControlCommand(StripControl.ALL),
                led.SetSolidColourCommand(Color.kGreen),
                led.BlinkCommand(0.3),
                new WaitCommand(1.2),
                led.putDefaultPatternCommand());

        gpLoadedCommand.addRequirements(led);

        Trigger gpLoadedTrigger = new Trigger(endEffector::isGpLoaded);

        gpLoadedTrigger.onTrue(gpLoadedCommand);
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
