package com.swrobotics.robot;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import com.swrobotics.robot.logging.RobotView;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;
import com.swrobotics.robot.subsystems.vision.VisionSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.swrobotics.robot.commands.AutonomousCommands;
import com.swrobotics.robot.control.AimCalc;
import com.swrobotics.robot.control.ControlBoard;
import com.swrobotics.robot.logging.FieldView;
//import com.swrobotics.robot.subsystems.climber.ClimberSubsystem;
import com.swrobotics.robot.subsystems.shooter.ShooterSubsystem;
import com.swrobotics.robot.subsystems.shooter.hood.HoodSubsystem;
import com.swrobotics.robot.subsystems.intake.IntakeSubsystem;
import com.swrobotics.robot.subsystems.intake.expansions.ExpansionSubsystem;
import com.swrobotics.robot.subsystems.intake.indexer.IndexerSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * The container for all of the robot's subsystems. This is separate from
 * {@link Robot} so that we can use a constructor for initialization instead of
 * {@code robotInit()}, which allows us to have final fields for subsystems.
 */
public class RobotContainer {
    // Create dashboard choosers
    private final SendableChooser<Command> autoSelector;
    private final SendableChooser<Double> autoDelaySelector;

    public final SwerveDriveSubsystem drive;
    public final VisionSubsystem vision;
    public final IndexerSubsystem indexer;
    public final ShooterSubsystem shooter;
    public final IntakeSubsystem intake;
    public final HoodSubsystem hood;
    public final ExpansionSubsystem expansion;
    //public final ClimberSubsystem climber;

    public final ControlBoard controlboard;

    public RobotContainer() {
        // Turn off joystick warnings in sim
        DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());

        drive = new SwerveDriveSubsystem();
        vision = new VisionSubsystem(drive);
        indexer = new IndexerSubsystem();
        shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();
        hood = new HoodSubsystem();
        expansion = new ExpansionSubsystem();
        //climber = new ClimberSubsystem();

        // ControlBoard must be initialized last
        controlboard = new ControlBoard(this);

        // Register Named Commands for Auto
        NamedCommands.registerCommand("Shoot", AutonomousCommands.getShootCommand(this));
        NamedCommands.registerCommand("Intake", AutonomousCommands.getIntakeCommand(this));
        NamedCommands.registerCommand("Expand", AutonomousCommands.getExpandCommand(this));
        NamedCommands.registerCommand("Retract", AutonomousCommands.getRetractCommand(this));

        // Create a chooser to select the autonomous
        List<AutoEntry> autos = buildPathPlannerAutos();
        autos.sort(Comparator.comparing(AutoEntry::name, String.CASE_INSENSITIVE_ORDER));
        autoSelector = new SendableChooser<>();
        autoSelector.setDefaultOption("None", Commands.none());
        SmartDashboard.putData("Auto Selector", autoSelector);

        for (AutoEntry auto : autos)
            autoSelector.addOption(auto.name(), auto.cmd());

        // Create a selector to select delay before running auto
        autoDelaySelector = new SendableChooser<>();
        autoDelaySelector.setDefaultOption("None", 0.0);
        for (int i = 0; i < 10; i++) {
            double time = i / 2.0 + 0.5;
            autoDelaySelector.addOption(time + " seconds", time);
        }
        SmartDashboard.putData("Auto Delay", autoDelaySelector);

        FieldView.publish();
        RobotView.publish();
    }

    private record AutoEntry(String name, Command cmd) {}
    
    private static List<AutoEntry> buildPathPlannerAutos() {
        if (!AutoBuilder.isConfigured()) {
            throw new RuntimeException(
                    "AutoBuilder was not configured before attempting to build an auto chooser");
        }

        List<String> autoNames = AutoBuilder.getAllAutoNames();
        autoNames.sort(String.CASE_INSENSITIVE_ORDER);

        List<PathPlannerAuto> options = new ArrayList<>();
        for (String autoName : autoNames) {
            PathPlannerAuto auto = new PathPlannerAuto(autoName);

            options.add(auto);
        }

        List<AutoEntry> entries = new ArrayList<>();
        for (PathPlannerAuto auto : options)
            entries.add(new AutoEntry(auto.getName(), auto));

        return entries;
    }

    public void disabledInit() {
    }

    public double getAutoDelay() {
        return autoDelaySelector.getSelected();
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }
}
