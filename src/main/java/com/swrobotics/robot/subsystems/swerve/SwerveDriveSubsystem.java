package com.swrobotics.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.FieldView;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class SwerveDriveSubsystem extends SubsystemBase {
    private static final NTBoolean CALIBRATE = new NTBoolean("Drive/Modules/Calibrate", false);

    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    private final StatusSignal<Angle> rawGyroAngleSignal;

    private SwerveDrivetrain.SwerveDriveState currentState;
    private Rotation2d rawGyroRotation;


    public SwerveDriveSubsystem() {
        int kModuleCount = Constants.kSwerveModuleInfos.length;
        var moduleConstants = new SwerveModuleConstants[kModuleCount];
        for (int i = 0; i < kModuleCount; i++) {
            SwerveModuleInfo info = Constants.kSwerveModuleInfos[i];
            moduleConstants[i] = Constants.kModuleConstantsFactory.createModuleConstants(
                info.turnId(),
                info.driveId(),
                info.encoderId(),
                info.offset().get(),
                info.position().getX(),
                info.position().getY(),
                false,
                true,
                false
            );
        }

        drivetrain = new SwerveDrivetrain<>(
            TalonFX::new, TalonFX::new, CANcoder::new,
            Constants.kDrivetrainConstants,
            Constants.kOdometryUpdateFreq,
            Constants.kOdometryStdDevs,
            // These values have no effect, they are overridden by the
            // standard deviations given to drivetrain.addVisionMeasurement()
            VecBuilder.fill(0.6, 0.6, 0.6),
            moduleConstants
        );

        for (int i = 0; i < kModuleCount; i++) {
            SwerveModule<TalonFX, TalonFX, CANcoder> module = drivetrain.getModule(i);

            CurrentLimitsConfigs driveLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.kDriveStatorCurrentLimit)
                .withSupplyCurrentLowerLimit(Constants.kDriveSupplyCurrentLimit)
                .withSupplyCurrentLowerTime(Constants.kDriveCurrentLimitTime);
            module.getDriveMotor().getConfigurator().apply(driveLimits);
        }

        // Set operator perspective to field +X axis (0 degrees) so coordinate
        // system stays centered on blue alliance origin
        drivetrain.setOperatorPerspectiveForward(new Rotation2d(0));

        rawGyroAngleSignal = drivetrain.getPigeon2().getYaw();

        currentState = drivetrain.getState();
        rawGyroRotation = Rotation2d.fromDegrees(rawGyroAngleSignal.refresh().getValueAsDouble());

        AutoBuilder.configure(
                this::getEstimatedPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> {
                    setControl(new SwerveRequest.ApplyRobotSpeeds()
                            .withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity));
                },
                new PPHolonomicDriveController(
                        new PIDConstants(Constants.kAutoDriveKp, Constants.kAutoDriveKd),
                        new PIDConstants(Constants.kAutoTurnKp.get(), Constants.kAutoTurnKd.get())
                ),
                Constants.kPathPlannerRobotConfig,
                () -> FieldInfo.getAlliance() == DriverStation.Alliance.Red,
                this
        );

        PathPlannerLogging.setLogActivePathCallback((path) -> {
            if (path != null) {
                FieldView.pathPlannerPath.setPoses(path);
                if (path.isEmpty())
                    FieldView.pathPlannerSetpoint.setPoses();
            } else {
                FieldView.pathPlannerSetpoint.setPoses();
            }
        });
        PathPlannerLogging.setLogTargetPoseCallback((target) -> {
            if (target != null)
                FieldView.pathPlannerSetpoint.setPose(target);
        });
    }

    public void setControl(SwerveRequest request) {
        drivetrain.setControl(request);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return currentState.Speeds;
    }

    public SwerveModulePosition[] getModulePositions() {
        return currentState.ModulePositions;
    }

    public Pose2d getEstimatedPose() {
        return currentState.Pose;
    }

    public boolean isCloseTo(Translation2d position, double tolerance) {
        double distance = getEstimatedPose().getTranslation().getDistance(position);
        return distance <= tolerance;
    }

    public void resetRotation(Rotation2d robotRotation) {
        drivetrain.resetRotation(robotRotation);
    }

    public void resetPose(Pose2d robotPose) {
        drivetrain.resetPose(robotPose);
    }

    public void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs) {
        drivetrain.addVisionMeasurement(robotPose, Utils.fpgaToCurrentTime(timestamp), stdDevs);
    }

    public Rotation2d getRawGyroRotation() {
        return rawGyroRotation;
    }

    public double getFFCharacterizationVelocity() {
        double avgVelocity = 0;
        for (SwerveModuleState state : currentState.ModuleStates) {
            avgVelocity += Math.abs(state.speedMetersPerSecond);
        }
        avgVelocity /= 4;

        return avgVelocity;
    }

    public Translation2d fieldToRobotRelative(Translation2d fieldRel) {
        return fieldRel.rotateBy(currentState.Pose.getRotation().unaryMinus());
    }

    private void calibrateModuleOffsets() {
        SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = drivetrain.getModules();
        for (int i = 0; i < modules.length; i++) {
            CANcoder canCoder = modules[i].getEncoder();
            NTEntry<Double> offset = Constants.kSwerveModuleInfos[i].offset();

            StatusSignal<Angle> canCoderPos = canCoder.getAbsolutePosition();
            canCoderPos.waitForUpdate(1);
            double position = canCoderPos.getValueAsDouble();

            offset.set(offset.get() - position);
            canCoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(offset.get()));
        }
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            drivetrain.updateSimState(Constants.kPeriodicTime, 12.0);
        }

        currentState = drivetrain.getState();
        rawGyroRotation = Rotation2d.fromDegrees(rawGyroAngleSignal.refresh().getValueAsDouble());

        FieldView.robotPose.setPose(currentState.Pose);

        // AdvantageKit structured logging
        Logger.recordOutput("Drive/Pose", currentState.Pose);
        Logger.recordOutput("Drive/ModuleStates", currentState.ModuleStates);
        Logger.recordOutput("Drive/ModuleTargets", currentState.ModuleTargets);
        Logger.recordOutput("Drive/ChassisSpeeds", currentState.Speeds);
        Logger.recordOutput("Drive/GyroYawDeg", rawGyroRotation.getDegrees());

        if (CALIBRATE.get()) {
            CALIBRATE.set(false);
            calibrateModuleOffsets();
        }
    }
}
