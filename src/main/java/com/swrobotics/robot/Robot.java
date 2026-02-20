package com.swrobotics.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * The main robot class.
 */
public final class Robot extends LoggedRobot {
    private static final Queue<Runnable> mainThreadOperations = new ConcurrentLinkedQueue<>();

    public static void runOnMainThread(Runnable runnable) {
        mainThreadOperations.add(runnable);
    }

    private Command autonomousCommand;
    private RobotContainer robotContainer;

    public Robot() {
        // Configure AdvantageKit logging before any other initialization
        Logger.recordMetadata("ProjectName", "GlopBot2026");
        Logger.recordMetadata("TeamNumber", "2129");

        if (isReal()) {
            // Real robot: log to USB drive + publish to NetworkTables for live AdvantageScope
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            // Simulation: check for replay log, otherwise just publish to NT
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            if (logPath == null) {
                // Normal simulation (no replay)
                Logger.addDataReceiver(new NT4Publisher());
            } else {
                // Replay mode
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            }
        }

        Logger.start();
    }

    @Override
    public void robotInit() {
        // Create a RobotContainer to manage our subsystems and our buttons
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);
        CommandScheduler.getInstance().run(); // Leave this alone

        Runnable r;
        while ((r = mainThreadOperations.poll()) != null) {
            r.run();
        }

        // Log general robot state
        Logger.recordOutput("Robot/BatteryVoltage", RobotController.getBatteryVoltage());
        Logger.recordOutput("Robot/MatchTime", DriverStation.getMatchTime());
        Logger.recordOutput("Robot/DSConnected", DriverStation.isDSAttached());
        Logger.recordOutput("Robot/Enabled", DriverStation.isEnabled());
    }

    @Override
    public void autonomousInit() {
        // If an autonomous command has already be set, reset it
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            System.out.println("Canceled the current auto command");
        }

        autonomousCommand = robotContainer.getAutonomousCommand();

        // Prevent crash if the same auto is run twice
        CommandScheduler.getInstance().removeComposedCommand(autonomousCommand);

        // Add delay if needed
        double delay = robotContainer.getAutoDelay();
        if (delay > 0) {
            autonomousCommand = Commands.sequence(
                    Commands.waitSeconds(delay),
                    autonomousCommand
            );
        }

        // Log whether auto was cancelled
        autonomousCommand = autonomousCommand
                .finallyDo((cancelled) -> {
                    if (cancelled)
                        DriverStation.reportWarning("Auto command ended early", false);
                });

        // For timing tests in simulator
        if (RobotBase.isSimulation()) {
            autonomousCommand = autonomousCommand
                    .withTimeout(15);
        }

        // Measure elapsed time
        double startTimestamp = Timer.getTimestamp();
        autonomousCommand = autonomousCommand
                .finallyDo(() -> {
                    double endTimestamp = Timer.getTimestamp();

                    System.out.println("Auto command took " + (endTimestamp - startTimestamp) + " seconds");
                });

        // Start autonomous command
        CommandScheduler.getInstance().schedule(autonomousCommand);
    }

    @Override
    public void autonomousExit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void disabledInit() {
        robotContainer.disabledInit();
    }

    // Override these so WPILib doesn't print unhelpful warnings
    @Override public void simulationPeriodic() {}
    @Override public void disabledPeriodic() {}
    @Override public void autonomousPeriodic() {}
    @Override public void teleopPeriodic() {}
    @Override public void testPeriodic() {}
}
