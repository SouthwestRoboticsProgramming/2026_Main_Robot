package com.swrobotics.robot.commands;

import java.util.Set;

import com.swrobotics.robot.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.swrobotics.robot.subsystems.intake.expansions.ExpansionSubsystem;
import com.swrobotics.robot.subsystems.intake.indexer.IndexerSubsystem;
import com.swrobotics.robot.subsystems.intake.IntakeSubsystem;
import com.swrobotics.robot.subsystems.shooter.ShooterSubsystem;

public class AutonomousCommands {
    public static Command getAutCommand(RobotContainer robot, String name) {
        return switch (name) {
            case "Shoot 3" -> Commands.sequence(
                    robot.shooter.commandSetState(ShooterSubsystem.State.SHOOT).withTimeout(3.0),
                    robot.indexer.commandSetState(IndexerSubsystem.State.FEED).withTimeout(3.0)
            );
            case "Intake and Shoot" -> Commands.sequence(
                    robot.intake.commandSetState(IntakeSubsystem.State.INTAKE).withTimeout(2.0),
                    robot.expansion.commandSetState(ExpansionSubsystem.State.EXTENDED).withTimeout(2.0),
                    robot.shooter.commandSetState(ShooterSubsystem.State.SHOOT).withTimeout(3.0),
                    robot.indexer.commandSetState(IndexerSubsystem.State.FEED).withTimeout(3.0)
            );
            default -> Commands.none();
        };

    
}
}
