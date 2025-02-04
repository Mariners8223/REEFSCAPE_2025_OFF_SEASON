// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.EndEffector.Eject;
import frc.robot.commands.EndEffector.moveFunnel;
import frc.robot.commands.EndEffector.Intake.Intake;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.DriveTrain.DriveBaseSYSID;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorSYSID;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants.FunnelMotor;
import frc.robot.subsystems.EndEffector.EndEffectorConstants.MotorPower;
import frc.robot.subsystems.EndEffector.EndEffectorSYSID;

public class RobotContainer {
    public static DriveBase driveBase;
    public static CommandPS5Controller driveController;
    public static DriveBaseSYSID driveBaseSYSID;
    public static Elevator elevator;
    public static ElevatorSYSID elevatorSYSID;

    public static Field2d field;
    public static LoggedDashboardChooser<Command> autoChooser;

    public static EndEffector endEffector;
    public static EndEffectorSYSID endEffectorSYSID;

    public RobotContainer() {
        driveController = new CommandPS5Controller(0);
        // driveBase = new DriveBase();
        elevator = new Elevator();
        // driveBaseSYSID = new DriveBaseSYSID(driveBase, driveController);
        elevatorSYSID = new ElevatorSYSID(elevator);

        endEffectorSYSID = new EndEffectorSYSID(endEffector);
        endEffector = new EndEffector();
        endEffector.setLoadedValue(false);

        configureBindings();

        field = new Field2d();

        SmartDashboard.putData(field);

        // configChooser();
    }


    private void configureBindings() {
        // driveController.options().onTrue(driveBase.resetOnlyDirection());

        driveController.povUp().onTrue(new MoveToLevel(elevator, ElevatorLevel.L4));
        driveController.povDown().onTrue(new MoveToLevel(elevator, ElevatorLevel.Bottom));
        driveController.povLeft().onTrue(new MoveToLevel(elevator, ElevatorLevel.L3));
        driveController.povRight().onTrue(new MoveToLevel(elevator, ElevatorLevel.L2));


        driveController.triangle().whileTrue(elevatorSYSID.getElevatorDynamic(Direction.kForward));
        driveController.cross().whileTrue(elevatorSYSID.getElevatorDynamic(Direction.kReverse));

        driveController.circle().whileTrue(elevatorSYSID.getElevatorQuasistatic(Direction.kForward));
        driveController.square().whileTrue(elevatorSYSID.getElevatorQuasistatic(Direction.kReverse));

        driveController.povUp().onTrue(new Intake(endEffector).onlyIf(() -> !endEffector.isGpLoaded()));
        driveController.povDown().onTrue(new Eject(MotorPower.L1, endEffector).onlyIf(endEffector::isGpLoaded));
        driveController.povRight().onTrue(new moveFunnel(endEffector, FunnelMotor.CLIMB_POSITION));
        driveController.povLeft().onTrue(new moveFunnel(endEffector, FunnelMotor.COLLECT_POSITION));
        

        driveController.triangle().whileTrue(endEffectorSYSID.getEndEffectorDynamic(Direction.kForward));
        driveController.cross().whileTrue(endEffectorSYSID.getEndEffectorDynamic(Direction.kReverse));

        driveController.circle().whileTrue(endEffectorSYSID.getEndEffectorQuasistatic(Direction.kForward));
        driveController.square().whileTrue(endEffectorSYSID.getEndEffectorQuasistatic(Direction.kReverse));
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
