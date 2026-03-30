package com.swrobotics.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.control.AimCalc;
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

public final class SwerveDriveSubsystem extends SubsystemBase {
    private static final NTBoolean CALIBRATE = new NTBoolean("Drive/Modules/Calibrate", false);

    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    private final StatusSignal<Angle> rawGyroAngleSignal;
    private SwerveDrivetrain.SwerveDriveState currentState;
    private Rotation2d rawGyroRotation;
    private final SwerveDriveBrake brakeRequest = new SwerveDriveBrake();

    public SwerveDriveSubsystem() {
        int kModuleCount = Constants.kSwerveModuleInfos.length;
        var moduleConstants = new SwerveModuleConstants[kModuleCount];
        for (int i = 0; i < kModuleCount; i++) {
            SwerveModuleInfo info = Constants.kSwerveModuleInfos[i];

            moduleConstants[i] = Constants.kModuleConstantsFactory.createModuleConstants(
                info.turnId(), info.driveId(), info.encoderId(),
                info.offset().get(), info.position().getX(), info.position().getY(),
                false, // drive inverted
                true,  // steer inverted
                false
            );
        }

        drivetrain = new SwerveDrivetrain<>(
            TalonFX::new, TalonFX::new, CANcoder::new,
            Constants.kDrivetrainConstants,
            Constants.kOdometryUpdateFreq,
            Constants.kOdometryStdDevs,
            VecBuilder.fill(0.1, 0.1, 0.1), // Vision std devs
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

        drivetrain.setOperatorPerspectiveForward(new Rotation2d(0));
        rawGyroAngleSignal = drivetrain.getPigeon2().getYaw();
        currentState = drivetrain.getState();
        rawGyroRotation = Rotation2d.fromDegrees(rawGyroAngleSignal.refresh().getValueAsDouble());

        // AUTOBUILDER
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
                new PIDConstants(Constants.kAutoDriveKp, 0.0),
                new PIDConstants(Constants.kAutoTurnKp.get(), 0.0)
            ),
            Constants.kPathPlannerRobotConfig,
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            this
        );

        setupPathLogging();
    }

    private void setupPathLogging() {
        PathPlannerLogging.setLogActivePathCallback((path) -> {
            if (path != null) FieldView.pathPlannerPath.setPoses(path);
        });
        PathPlannerLogging.setLogTargetPoseCallback((target) -> {
            if (target != null) FieldView.pathPlannerSetpoint.setPose(target);
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

    public void lockModules() {
        this.setControl(brakeRequest);
    }

    public Rotation2d getRawGyroRotation() {
        return rawGyroRotation;
    }

    public double getFFCharacterizationVelocity() {
        double avgVelocity = 0.0;
        for (SwerveModuleState state : currentState.ModuleStates) {
            avgVelocity += Math.abs(state.speedMetersPerSecond);
        }
        avgVelocity /= currentState.ModuleStates.length;
        return avgVelocity;
    }

    // Field-relative velocity helper
    public Translation2d getFieldRelativeVelocity() {
        ChassisSpeeds robotRel = currentState.Speeds;
        return new Translation2d(
            robotRel.vxMetersPerSecond,
            robotRel.vyMetersPerSecond
        ).rotateBy(currentState.Pose.getRotation());
    }

    public edu.wpi.first.wpilibj2.command.Command commandLockModules() {
        return run(this::lockModules);
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            drivetrain.updateSimState(Constants.kPeriodicTime, 12.0);
        }

        currentState = drivetrain.getState();
        rawGyroRotation = Rotation2d.fromDegrees(rawGyroAngleSignal.refresh().getValueAsDouble());
        FieldView.robotPose.setPose(currentState.Pose);

        if (CALIBRATE.get()) {
            CALIBRATE.set(false);
            calibrateModuleOffsets();
        }
        AimCalc.getInstance().update(currentState.Pose, currentState.Speeds);
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
}