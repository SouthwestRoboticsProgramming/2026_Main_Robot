package com.swrobotics.robot.commands;

import com.swrobotics.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.swrobotics.robot.subsystems.intake.expansions.ExpansionSubsystem;
import com.swrobotics.robot.subsystems.intake.indexer.IndexerSubsystem;
import com.swrobotics.robot.subsystems.intake.IntakeSubsystem;
import com.swrobotics.robot.subsystems.shooter.ShooterSubsystem;

public class AutonomousCommands {
    public static Command getShootCommand(RobotContainer robot) {
        return robot.shooter.commandSetState(ShooterSubsystem.State.SHOOT)
                    .withTimeout(.75)
                    .andThen(robot.indexer.commandSetState(IndexerSubsystem.State.FEED)).withTimeout(4);
    }

    public static Command getIntakeCommand(RobotContainer robot) {
        return robot.expansion.commandSetState(ExpansionSubsystem.State.EXTENDED)
                .withTimeout(.2)
                .andThen(robot.intake.commandSetState(IntakeSubsystem.State.INTAKE)
                    .alongWith(robot.indexer.commandSetState(IndexerSubsystem.State.INTAKE)).withTimeout(.1));
    }
    
}