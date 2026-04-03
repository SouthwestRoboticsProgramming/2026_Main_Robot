package com.swrobotics.robot.commands;

import com.swrobotics.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.swrobotics.robot.subsystems.intake.indexer.IndexerSubsystem;
import com.swrobotics.robot.subsystems.intake.IntakeSubsystem;
import com.swrobotics.robot.subsystems.intake.expansion.ExpansionSubsystem;
import com.swrobotics.robot.subsystems.shooter.ShooterSubsystem;

public class AutonomousCommands {
    
    public static Command getShootCommand(RobotContainer robot) {
        // Spin up flywheel, wait until at speed (2s safety timeout), then feed while keeping shooter spinning
        return robot.shooter.commandSetState(ShooterSubsystem.State.SHOOT)
                    .until(() -> robot.shooter.isAtTargetRPS())
                    .withTimeout(.5)
                    .andThen(
                        robot.shooter.commandSetState(ShooterSubsystem.State.SHOOT)
                        .alongWith(robot.indexer.commandSetState(IndexerSubsystem.State.FEED))
                        .withTimeout(4)
                    );
    }

    public static Command getIntakeCommand(RobotContainer robot) {
        return robot.intake.commandSetState(IntakeSubsystem.State.INTAKE)
        .alongWith(robot.indexer.commandSetState(IndexerSubsystem.State.INTAKE)).withTimeout(.1);
    }

    public static Command getExpandCommand(RobotContainer robot) {
        return robot.expansion.commandSetState(ExpansionSubsystem.State.EXTENDED).withTimeout(.3);
    }

    public static Command getRetractCommand(RobotContainer robot) {
        return robot.expansion.commandSetState(ExpansionSubsystem.State.LIFTED).withTimeout(.3);
    }

    public static Command getShootPath(RobotContainer robot) {
        return Commands.sequence(
            getShootCommand(robot),
            getRetractCommand(robot)
        );
    }

    public static Command getIntakePath(RobotContainer robot) {
        return Commands.sequence(
            getExpandCommand(robot),
            getIntakeCommand(robot)
        );
    }

 
    
}