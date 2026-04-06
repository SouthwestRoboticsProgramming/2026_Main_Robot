package com.swrobotics.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.lib.utils.PolynomialRegression;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;
import com.swrobotics.robot.control.AimCalc;
import com.swrobotics.robot.subsystems.intake.indexer.IndexerSubsystem;
import com.swrobotics.robot.subsystems.shooter.ShooterSubsystem;
import com.swrobotics.robot.subsystems.shooter.hood.HoodSubsystem;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

public final class DriveCommands {
    public static Command driveRobotRelative(
            SwerveDriveSubsystem drive,
            Supplier<Translation2d> translationSupplier, // m/s
            Supplier<Double> rotationSupplier // rad/s
    ) {
        return Commands.run(() -> {
            Translation2d tx = translationSupplier.get();
            double rot = rotationSupplier.get();

            drive.setControl(new SwerveRequest.RobotCentric()
                    .withVelocityX(tx.getX())
                    .withVelocityY(tx.getY())
                    .withRotationalRate(rot));
        }, drive);
    }

    public static Command driveFieldRelative(
            SwerveDriveSubsystem drive,
            Supplier<Translation2d> translationSupplier, // m/s
            Supplier<Double> rotationSupplier // rad/s
    ) {
        return Commands.run(() -> {
            Translation2d tx = translationSupplier.get();
            double rot = rotationSupplier.get();

            drive.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(tx.getX())
                    .withVelocityY(tx.getY())
                    .withRotationalRate(rot));
        }, drive);
    }
        public static Command driveFieldRelativeSnapTo180(
            SwerveDriveSubsystem drive,

            Supplier<Translation2d> translationSupplier
) {
        PIDController turnPid = new PIDController(0, 0, 0); // Values set to 0, changed a few lines later
        turnPid.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.startRun(() -> {
            turnPid.setPID(Constants.kSnapTurnKp.get(), 0, Constants.kSnapTurnKd.get());
            turnPid.setTolerance(Math.toRadians(Constants.kSnapThetaDeadzone.get()));
            turnPid.reset();
        }, () -> {
            Translation2d tx = translationSupplier.get();

            Rotation2d currentRot = drive.getEstimatedPose().getRotation();
            Rotation2d targetRot = Rotation2d.fromDegrees(180);
            
            double rotOutput = turnPid.calculate(
                    MathUtil.wrap(currentRot.getRadians(), -Math.PI, Math.PI),
                    MathUtil.wrap(targetRot.getRadians(), -Math.PI, Math.PI)
            );
            if (turnPid.atSetpoint()) {
                rotOutput = 0;
            }else {
            double maxTurnSpeed = Units.rotationsToRadians(Constants.kSnapMaxTurnSpeed.get());
            rotOutput = MathUtil.clamp(rotOutput, -maxTurnSpeed, maxTurnSpeed);
            }

            drive.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(tx.getX())
                    .withVelocityY(tx.getY())
                    .withRotationalRate(rotOutput));
        }, drive);
    }

    public static Command driveFieldRelativeSnapToHub(
            SwerveDriveSubsystem drive,

            Supplier<Translation2d> translationSupplier
) {
        PIDController turnPid = new PIDController(0, 0, 0); // Values set to 0, changed a few lines later
        turnPid.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.startRun(() -> {
            turnPid.setPID(Constants.kSnapTurnKp.get(), 0, Constants.kSnapTurnKd.get());
            turnPid.setTolerance(Math.toRadians(Constants.kSnapThetaDeadzone.get()));
            turnPid.reset();
        }, () -> {
            Translation2d tx = translationSupplier.get();
            
            Rotation2d currentRot = drive.getEstimatedPose().getRotation();
            Rotation2d targetRot = AimCalc.getInstance().getDrivebaseAimAngle();
            // TEMP
            Constants.currentAngle.set(Math.abs(currentRot.getDegrees()));
            Constants.targetAngle.set(Math.abs(targetRot.getDegrees()));
            

            double rotOutput = turnPid.calculate(
                    MathUtil.wrap(currentRot.getRadians(), -Math.PI, Math.PI),
                    MathUtil.wrap(targetRot.getRadians(), -Math.PI, Math.PI)
            );
            if (turnPid.atSetpoint()) {
                rotOutput = 0;
            }else {
            double maxTurnSpeed = Units.rotationsToRadians(Constants.kSnapMaxTurnSpeed.get());
            rotOutput = MathUtil.clamp(rotOutput, -maxTurnSpeed, maxTurnSpeed);
            }

            drive.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(tx.getX())
                    .withVelocityY(tx.getY())
                    .withRotationalRate(rotOutput));
        }, drive);
    }



public static Command shootOnTheMove(
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
    .finallyDo(() -> {

    });
}

    private static Pose2d getClosestPose(Pose2d currentPose, List<Pose2d> candidates) {
    Pose2d closest = candidates.get(0);
    double minDistance = currentPose.getTranslation().getDistance(closest.getTranslation());

    for (Pose2d pose : candidates) {
        double dist = currentPose.getTranslation().getDistance(pose.getTranslation());
        if (dist < minDistance) {
            minDistance = dist;
            closest = pose;
        }
    }
    return closest;
    }
    public static Pose2d getCorrespondingPose(Pose2d currentTarget, Alliance alliance) {
    // Trench Mapping
    if (currentTarget.equals(FieldPositions.kLDSBlueTrenchPose)) return FieldPositions.kLCBlueTrenchPose;
    if (currentTarget.equals(FieldPositions.kLCBlueTrenchPose)) return FieldPositions.kLDSBlueTrenchPose;
    if (currentTarget.equals(FieldPositions.kRCBlueTrenchPose)) return FieldPositions.kRDSBlueTrenchPose;
    if (currentTarget.equals(FieldPositions.kRDSBlueTrenchPose)) return FieldPositions.kRCBlueTrenchPose;
    if (currentTarget.equals(FieldPositions.kLDSRedTrenchPose)) return FieldPositions.kLCRedTrenchPose;
    if (currentTarget.equals(FieldPositions.kLCRedTrenchPose)) return FieldPositions.kLDSRedTrenchPose;
    if (currentTarget.equals(FieldPositions.kRCRedTrenchPose)) return FieldPositions.kRDSRedTrenchPose;
    if (currentTarget.equals(FieldPositions.kRDSRedTrenchPose)) return FieldPositions.kRCRedTrenchPose;

    // Bump Mapping
    if (currentTarget.equals(FieldPositions.kLDSBlueBumpPose)) return FieldPositions.kLCBlueBumpPose;
    if (currentTarget.equals(FieldPositions.kLCBlueBumpPose)) return FieldPositions.kLDSBlueBumpPose;
    if (currentTarget.equals(FieldPositions.kRCBlueBumpPose)) return FieldPositions.kRDSBlueBumpPose;
    if (currentTarget.equals(FieldPositions.kRDSBlueBumpPose)) return FieldPositions.kRCBlueBumpPose;
    if (currentTarget.equals(FieldPositions.kLDSRedBumpPose)) return FieldPositions.kLCRedBumpPose;
    if (currentTarget.equals(FieldPositions.kLCRedBumpPose)) return FieldPositions.kLDSRedBumpPose;
    if (currentTarget.equals(FieldPositions.kRCRedBumpPose)) return FieldPositions.kRDSRedBumpPose;
    if (currentTarget.equals(FieldPositions.kRDSRedBumpPose)) return FieldPositions.kRCRedBumpPose;

    return currentTarget;
    }

    

    public static Command driveThroughTrench(SwerveDriveSubsystem drive) {
    return Commands.defer(() -> {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        Pose2d currentPose = drive.getEstimatedPose();

        // Gather candidates using your required alliance syntax
        List<Pose2d> trenchPoses = List.of(
            FieldPositions.kLDSBlueTrenchPose, 
            FieldPositions.kLCBlueTrenchPose, 
            FieldPositions.kRCBlueTrenchPose, 
            FieldPositions.kRDSBlueTrenchPose,
            FieldPositions.kLDSRedTrenchPose, 
            FieldPositions.kLCRedTrenchPose, 
            FieldPositions.kRCRedTrenchPose, 
            FieldPositions.kRDSRedTrenchPose
        );

        Pose2d entryPose = getClosestPose(currentPose, trenchPoses);
        Pose2d exitPose = getCorrespondingPose(entryPose, alliance);
        
        // Use 180 for Red, 0 for Blue
        Rotation2d trenchRot = (alliance == Alliance.Red) ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

        return Commands.sequence(
            // Phase 1: Drive to the entrance point
            snapToPoseUntilInTolerance(drive, () -> new Pose2d(entryPose.getTranslation(), trenchRot), () -> 0.1),
            // Phase 2: Drive through to the exit point
            snapToPose(drive, () -> new Pose2d(exitPose.getTranslation(), trenchRot))
        );
    }, Set.of(drive));
}

public static Command driveThroughBump(SwerveDriveSubsystem drive) {
    return Commands.defer(() -> {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        Pose2d currentPose = drive.getEstimatedPose();

        List<Pose2d> bumpPoses = List.of(
            FieldPositions.kLDSBlueBumpPose, 
            FieldPositions.kLCBlueBumpPose,
            FieldPositions.kRCBlueBumpPose,
            FieldPositions.kRDSBlueBumpPose,
            FieldPositions.kLDSRedBumpPose,
            FieldPositions.kLCRedBumpPose,
            FieldPositions.kRCRedBumpPose,
            FieldPositions.kRDSRedBumpPose
        );

        Pose2d entryPose = getClosestPose(currentPose, bumpPoses);
        Pose2d exitPose = getCorrespondingPose(entryPose, alliance);
        
        // Use 135 for Red, 45 for Blue
        Rotation2d bumpRot = (alliance == Alliance.Red) ? Rotation2d.fromDegrees(135) : Rotation2d.fromDegrees(45);

        return Commands.sequence(
            // Phase 1: Snap to the bump approach
            snapToPoseUntilInTolerance(drive, () -> new Pose2d(entryPose.getTranslation(), bumpRot), () -> 0.1),
            // Phase 2: Drive over the bump to the exit
            snapToPose(drive, () -> new Pose2d(exitPose.getTranslation(), bumpRot))
        );
    }, Set.of(drive));
}
    

    public static Command driveSnapToTarget(
        SwerveDriveSubsystem drive,
        Supplier<Translation2d> translationSupplier,
        Supplier<Rotation2d> targetRotationSupplier) {

    PIDController turnPid = new PIDController(0, 0, 0);
    turnPid.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(() -> {
        // Pull tunable values from your Constants (NTEntry)
        turnPid.setPID(Constants.kSnapTurnKp.get(), 0, Constants.kSnapTurnKd.get());

        // Get pose fused by your VisionSubsystem + Pigeon 2
        Pose2d currentPose = drive.getEstimatedPose();
        Rotation2d targetRot = targetRotationSupplier.get();

        double rotOutput = turnPid.calculate(
            currentPose.getRotation().getRadians(),
            targetRot.getRadians()
        );

        // Clamp using your NTEntry
        double maxSpeed = Units.rotationsToRadians(Constants.kSnapMaxTurnSpeed.get());
        rotOutput = MathUtil.clamp(rotOutput, -maxSpeed, maxSpeed);

        Translation2d tx = translationSupplier.get();

        // Use your subsystem's setControl method
        drive.setControl(new SwerveRequest.FieldCentric()
            .withVelocityX(tx.getX())
            .withVelocityY(tx.getY())
            .withRotationalRate(rotOutput)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity));
    }, drive);
}

    public static Command snapToPose(SwerveDriveSubsystem drive, Supplier<Pose2d> targetPoseSupplier) {
        PIDController driveXPid = new PIDController(0, 0, 0);
        PIDController driveYPid = new PIDController(0, 0, 0);
        PIDController turnPid = new PIDController(0, 0, 0);
        turnPid.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.startRun(() -> {
            // Update PIDs in case we tuned them since last time
            driveXPid.setPID(Constants.kSnapDriveKp.get(), 0, Constants.kSnapDriveKd.get());
            driveYPid.setPID(Constants.kSnapDriveKp.get(), 0, Constants.kSnapDriveKd.get());
            turnPid.setPID(Constants.kSnapTurnKp.get(), 0, Constants.kSnapTurnKd.get());

            driveXPid.reset();
            driveYPid.reset();
            turnPid.reset();
        }, () -> {
            Pose2d currentPose = drive.getEstimatedPose();
            Pose2d targetPose = targetPoseSupplier.get();

            double xyError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double thetaError = MathUtil.absDiffRad(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

            double xOutput = 0, yOutput = 0;
            if (xyError > Constants.kSnapXYDeadzone.get()) {
                xOutput = driveXPid.calculate(currentPose.getX(), targetPose.getX());
                yOutput = driveYPid.calculate(currentPose.getY(), targetPose.getY());
            }
            double rotOutput = 0;
            if (thetaError > Math.toRadians(Constants.kSnapThetaDeadzone.get())) {
                rotOutput = turnPid.calculate(
                        MathUtil.wrap(currentPose.getRotation().getRadians(), -Math.PI, Math.PI),
                        MathUtil.wrap(targetPose.getRotation().getRadians(), -Math.PI, Math.PI)
                );
            }

            double maxDriveSpeed = 10;//Constants.kSnapMaxSpeed.get();
            double driveSpeed = Math.hypot(xOutput, yOutput);
            if (driveSpeed > maxDriveSpeed) {
                double scale = maxDriveSpeed / driveSpeed;
                xOutput *= scale;
                yOutput *= scale;
            }

            double maxTurnSpeed = Units.rotationsToRadians(Constants.kSnapMaxTurnSpeed.get());
            rotOutput = MathUtil.clamp(rotOutput, -maxTurnSpeed, maxTurnSpeed);

            drive.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(xOutput)
                    .withVelocityY(yOutput)
                    .withRotationalRate(rotOutput));
        }, drive);
    }

    public static Command snapToPoseUntilInTolerance(
            SwerveDriveSubsystem drive,
            Supplier<Pose2d> poseSupplier,
            Supplier<Double> toleranceSupplier) {
        return snapToPose(drive, poseSupplier)
                .until(() -> drive.isCloseTo(
                        poseSupplier.get().getTranslation(),
                        toleranceSupplier.get()
                ));
    }

    // public static Command autoalignToHub(SwerveDriveSubsystem drive, LightsSubsystem lights, LimelightCamera frontLeftLimelight, LimelightCamera frontRightLimelight, LimelightCamera backLimelight, int hubTagId /*  AprilTag ID for 2026 Rebuilt hub*/){
        


    public static Command feedforwardCharacterization(SwerveDriveSubsystem drive) {
        List<Double> velocitySamples = new ArrayList<>();
        List<Double> voltageSamples = new ArrayList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient forward
                Commands.run(() -> drive.setControl(new SwerveRequest.SysIdSwerveTranslation()
                        .withVolts(0)), drive)
                        .withTimeout(1),

                Commands.runOnce(timer::restart),

                Commands.run(() -> {
                    double voltage = timer.get() * 0.1;

                    drive.setControl(new SwerveRequest.SysIdSwerveTranslation()
                            .withVolts(voltage));

                    // Convert units to rotor rotations, which is what CTRE wants
                    double metersPerSec = drive.getFFCharacterizationVelocity();
                    double wheelRadPerSec = metersPerSec / Constants.kModuleConstantsFactory.WheelRadius;
                    double wheelRotPerSec = Units.radiansToRotations(wheelRadPerSec);
                    double rotorRotPerSec = wheelRotPerSec * Constants.kModuleConstantsFactory.DriveMotorGearRatio;

                    velocitySamples.add(rotorRotPerSec);
                    voltageSamples.add(voltage);
                }, drive).finallyDo(() -> {
                    if (velocitySamples.size() < 2 || voltageSamples.size() < 2)
                        return;

                    double[] velocityArray = new double[velocitySamples.size()];
                    for (int i = 0; i < velocityArray.length; i++) {
                        velocityArray[i] = velocitySamples.get(i);
                    }

                    double[] voltageArray = new double[voltageSamples.size()];
                    for (int i = 0; i < voltageArray.length; i++) {
                        voltageArray[i] = voltageSamples.get(i);
                    }

                    PolynomialRegression regression = new PolynomialRegression(velocityArray, voltageArray, 1);
                    double kS = regression.beta(0); // y-intercept
                    double kV = regression.beta(1); // slope
                    double R2 = regression.R2();

                    System.out.println("FF Characterization Results:");
                    System.out.printf("\tR2=%.5f%n", R2);
                    System.out.printf("\tkS=%.5f%n", kS);
                    System.out.printf("\tkV=%.5f%n", kV);

                    SmartDashboard.putNumber("FF Characterization/kS", kS);
                    SmartDashboard.putNumber("FF Characterization/kV", kV);
                    SmartDashboard.putNumber("FF Characterization/R2", R2);
                })
        );
    }

    public static Command driveToPose(SwerveDriveSubsystem drive, Pose2d currentPose) {
        throw new UnsupportedOperationException("Unimplemented method 'driveToPose'");
    }
}
