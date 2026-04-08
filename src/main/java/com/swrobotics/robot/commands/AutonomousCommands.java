package com.swrobotics.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.control.AimCalc;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.swrobotics.robot.subsystems.intake.indexer.IndexerSubsystem;
import com.swrobotics.robot.subsystems.intake.IntakeSubsystem;
import com.swrobotics.robot.subsystems.intake.expansion.ExpansionSubsystem;
import com.swrobotics.robot.subsystems.shooter.ShooterSubsystem;
import com.swrobotics.robot.subsystems.shooter.hood.HoodSubsystem;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;

public class AutonomousCommands {
    
    public static Command getShootCommand(RobotContainer robot) {
        return robot.shooter.commandSetState(ShooterSubsystem.State.AUTO)           
        .alongWith(robot.hood.setMode(HoodSubsystem.HoodState.AUTO_TRACK).withTimeout(.75)
                    .andThen(robot.indexer.commandSetState(IndexerSubsystem.State.FEED))
                        
                    ).withTimeout(3);
    }

    public static Command getIntakeCommand(RobotContainer robot) {
        return robot.intake.commandSetState(IntakeSubsystem.State.INTAKE)
        .alongWith(robot.indexer.commandSetState(IndexerSubsystem.State.INTAKE));
    }


    public static Command getExpandCommand(RobotContainer robot) {
        return robot.expansion.commandSetState(ExpansionSubsystem.State.EXTENDED).withTimeout(.5);
    }

    public static Command getRetractCommand(RobotContainer robot) {
        return robot.expansion.commandSetState(ExpansionSubsystem.State.STOWED).withTimeout(.5);
    }

    public static Command getGoUnderTrench(RobotContainer robot) {
        return DriveCommands.driveThroughTrench(robot.drive).withTimeout(3); 
    }
    public static Command  getGoOverBump(RobotContainer robot) {
        return DriveCommands.driveOverBump(robot.drive).withTimeout(2);
    }
    public static Command Aim(
        SwerveDriveSubsystem drive,
        Supplier<Double> translationX,
        Supplier<Double> translationY
) {
    PIDController turnPid = new PIDController(Constants.kSnapTurnKp.get(), 0, Constants.kSnapTurnKd.get()); 
    turnPid.enableContinuousInput(-Math.PI, Math.PI);
    
    // TUNE: Set how close (in radians) the robot needs to be to the target angle to shoot
    turnPid.setTolerance(Math.toRadians(2.0)); 

    return Commands.run(() -> {
        // 1. Drivebase Aiming
        Pose2d currentPose = drive.getEstimatedPose();
        Rotation2d targetAngle = AimCalc.getInstance().getDrivebaseAimAngle();

        double rotOutput = turnPid.calculate(currentPose.getRotation().getRadians(), targetAngle.getRadians());

        drive.setControl(new SwerveRequest.FieldCentric()
            .withVelocityX(translationX.get())
            .withVelocityY(translationY.get())
            .withRotationalRate(rotOutput));

        


    }, drive)
    .withTimeout(.75); // TUNE: Set a timeout for how long the robot should try to aim before giving up
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