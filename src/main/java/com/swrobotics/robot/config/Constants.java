package com.swrobotics.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.subsystems.swerve.SwerveModuleInfo;
import com.swrobotics.robot.subsystems.vision.LimelightCamera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

import static edu.wpi.first.units.Units.*;

/**
 * Constants governing physical robot chassis properties, onboard processing frequencies,
 * control profiles, tuning configurations, physical sensor locations, and device initialization.
 */
public final class Constants {
    // Loop Scheduling Timers
    public static final int kPeriodicFreq = 50; // Hz
    public static final double kPeriodicTime = 1.0 / kPeriodicFreq;
    public static final double kOdometryUpdateFreq = 200; // Hz
    public static final Matrix<N3, N1> kOdometryStdDevs = VecBuilder.fill(0.005, 0.005, 0.001);

    // Mechanical Chassis Dimensions
    public static final double kFrameLength = Units.inchesToMeters(27.25); // m
    public static final double kFrameWidth = Units.inchesToMeters(27.25); // m
    public static final double kRobotMass = Units.lbsToKilograms(135); 
    public static final double kRobotMOI = 1.0 / 12.0 * kRobotMass * (kFrameLength * kFrameLength + kFrameWidth * kFrameWidth);

    // Operator Interface Configuration
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kSuperControllerPort = 2;
    public static final double kDeadband = 0.15;
    public static final double kTriggerThreshold = 0.3;

    // Driver Control Curves
    public static final double kDriveControlMaxAccel = 3.5; // m/s^2
    public static final double kDriveControlMaxTurnSpeed = 1; // rot/s
    public static final double kDriveControlDrivePower = 2; // Exponent input shape
    public static final double kDriveControlTurnPower = 2;

    // Autonomous Trajectory Tracking PID
    public static final double kAutoDriveKp = 4;
    public static final double kAutoDriveKd = 0;
    public static final NTEntry<Double> kAutoTurnKp = new NTDouble("Drive/Auto/Turn PID/kP", 5).setPersistent();
    public static final NTEntry<Double> kAutoTurnKd = new NTDouble("Drive/Auto/Turn PID/kD", 0).setPersistent();

    // Target Snapping Control Loop Configs
    public static final NTEntry<Double> kSnapMaxSpeed = new NTDouble("Drive/Snap/Max Speed (meters per sec)", 10).setPersistent();
    public static final NTEntry<Double> kSnapMaxTurnSpeed = new NTDouble("Drive/Snap/Max Turn Speed (rot per sec)", 3.5).setPersistent();
    public static final NTEntry<Double> kSnapDriveKp = new NTDouble("Drive/Snap/Drive kP", 2).setPersistent();
    public static final NTEntry<Double> kSnapDriveKd = new NTDouble("Drive/Snap/Drive kD", 0.2).setPersistent();
    public static final NTEntry<Double> kSnapTurnKp = new NTDouble("Drive/Snap/Turn kP", 4).setPersistent();
    public static final NTEntry<Double> kSnapTurnKd = new NTDouble("Drive/Snap/Turn kD", 0).setPersistent();
    public static final NTEntry<Double> kSnapXYDeadzone = new NTDouble("Drive/Snap/XY Deadzone (m)", 0.005).setPersistent();
    public static final NTEntry<Double> kSnapThetaDeadzone = new NTDouble("Drive/Snap/Theta Deadzone (deg)", 0.2).setPersistent();

    // Swerve Structural Assembly Math
    public static final double kDriveMaxAchievableSpeed = Units.feetToMeters(18.9); // m/s
    public static final double kDriveStatorCurrentLimit = 60; // A
    public static final double kDriveSupplyCurrentLimit = 40; // A
    public static final double kDriveCurrentLimitTime = 0.25; // sec
    public static final double kDriveWheelCOF = 1.2;
    public static final double kDriveWheelSpacingX = 63.0 / 100; // m
    public static final double kDriveWheelSpacingY = 55.3 / 100; // m
    public static final double kDriveRadius = Math.hypot(kDriveWheelSpacingX / 2, kDriveWheelSpacingY / 2);

    // Module Calibration Offset References
    public static final NTEntry<Double> kFrontLeftOffset = new NTDouble("Drive/Modules/Front Left Offset (rot)", -0.08935546875).setPersistent();
    public static final NTEntry<Double> kFrontRightOffset = new NTDouble("Drive/Modules/Front Right Offset (rot)", 0.918701171875).setPersistent();
    public static final NTEntry<Double> kBackLeftOffset = new NTDouble("Drive/Modules/Back Left Offset (rot)", 0.1923828125).setPersistent();
    public static final NTEntry<Double> kBackRightOffset = new NTDouble("Drive/Modules/Back Right Offset (rot)", -0.416015625).setPersistent();

    public static final SwerveModuleInfo[] kSwerveModuleInfos = {
            new SwerveModuleInfo(IOAllocation.CAN.kSwerveFL, kDriveWheelSpacingX / 2, kDriveWheelSpacingY / 2, kFrontLeftOffset, "Front Left"),
            new SwerveModuleInfo(IOAllocation.CAN.kSwerveFR, kDriveWheelSpacingX / 2, -kDriveWheelSpacingY / 2, kFrontRightOffset, "Front Right"),
            new SwerveModuleInfo(IOAllocation.CAN.kSwerveBL, -kDriveWheelSpacingX / 2, kDriveWheelSpacingY / 2, kBackLeftOffset, "Back Left"),
            new SwerveModuleInfo(IOAllocation.CAN.kSwerveBR, -kDriveWheelSpacingX / 2, -kDriveWheelSpacingY / 2, kBackRightOffset, "Back Right")
    };

    // CTRE Hardware Drivetrain Instantiations
    public static final SwerveDrivetrainConstants kDrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(IOAllocation.CAN.kSwerveBus)
            .withPigeon2Id(IOAllocation.CAN.kJosh.id())
            .withPigeon2Configs(new Pigeon2Configuration());

    public static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kModuleConstantsFactory =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio((50.0 / 16) * (16.0 / 28) * (45.0 / 15))
                    .withSteerMotorGearRatio(150.0 / 7)
                    .withCouplingGearRatio(50.0 / 16)
                    .withWheelRadius(Meters.of(0.0485603333))
                    .withSteerMotorGains(new Slot0Configs().withKP(50).withKD(0.01).withKV(0.1))
                    .withDriveMotorGains(new Slot0Configs().withKP(0.35).withKD(0).withKV(0.012621).withKS(0.22109))
                    .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                    .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                    .withSlipCurrent(Amps.of(80))
                    .withSpeedAt12Volts(MetersPerSecond.of(kDriveMaxAchievableSpeed))
                    .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                    .withDriveMotorInitialConfigs(new TalonFXConfiguration())
                    .withSteerMotorInitialConfigs(new TalonFXConfiguration())
                    .withEncoderInitialConfigs(new CANcoderConfiguration());

    static {
        if (RobotBase.isSimulation()) {
            kModuleConstantsFactory.DriveMotorGains
                    .withKV(0.12612)
                    .withKS(0.22510);
        }
    }

    // PathPlanner Physics Configuration Map
    public static final RobotConfig kPathPlannerRobotConfig = new RobotConfig(
            kRobotMass,
            kRobotMOI,
            new ModuleConfig(
                    kModuleConstantsFactory.WheelRadius,
                    kDriveMaxAchievableSpeed,
                    kDriveWheelCOF,
                    DCMotor.getKrakenX60Foc(1).withReduction(kModuleConstantsFactory.DriveMotorGearRatio),
                    kDriveStatorCurrentLimit,
                    1
            ),
            kSwerveModuleInfos[0].position(),
            kSwerveModuleInfos[1].position(),
            kSwerveModuleInfos[2].position(),
            kSwerveModuleInfos[3].position()
    );

    // Vision Data Matrix Configurations
    public static final double kVisionMT2SpeedThreshold = 0.2; // m/s
    public static final LimelightCamera.Config kLimelightConfig = new LimelightCamera.Config(
            2, 0.00197, 0.002, 0.00117
    );

    public static final LimelightCamera.MountingLocation kLimelightBackLocation = new LimelightCamera.MountingLocation(
            Units.inchesToMeters(9.7), Units.inchesToMeters(4.75), Units.inchesToMeters(13.81),
            0, 15, 180
    );

    public static final LimelightCamera.MountingLocation kLimelightRightLocation = new LimelightCamera.MountingLocation(
            Units.inchesToMeters(10.05), Units.inchesToMeters(-12.3503), Units.inchesToMeters(9.502),
            0, 0, 90
    );

    public static final LimelightCamera.MountingLocation kLimelightFrontLocation = new LimelightCamera.MountingLocation(
            Units.inchesToMeters(9.92), Units.inchesToMeters(0), Units.inchesToMeters(13.81),
            0, 15, 0
    );

    // Mechanism Subsystem Properties
    public static final NTEntry<Boolean> kHoodInverted = new NTBoolean("Shooter/Hood/Inverted", false).setPersistent();
    public static final NTEntry<Double> kIndexerIdleVoltage = new NTDouble("Intake/Indexer/Idle Voltage", 0.0).setPersistent();

    // Aim Verification Calculations
    public static final NTEntry<Double> kBaseFlightTime = new NTDouble("Shooter/Aim/Base Flight Time (s)", 0.4).setPersistent();
    public static final NTEntry<Double> kFlightTimePerMeter = new NTDouble("Shooter/Aim/Flight Time Per Meter (s)", 0.10).setPersistent();
    public static final NTEntry<Double> kHoodAngleOffset = new NTDouble("Shooter/Aim/Hood Angle Offset (deg)", 0.0).setPersistent();

    // Shooter Physical Geometric Offsets
    public static final double kShooterOffsetX = Units.inchesToMeters(10.0);   // Forward Vector (+)
    public static final double kShooterOffsetY = Units.inchesToMeters(-10.0); // Right Vector (-)
    public static final double kShooterHeightMeters = Units.inchesToMeters(20.0);

    public static final NTEntry<Double> kIndexerStatorCurrentLimit = new NTDouble("Indexer/Current Limits/Stator (A)", 60.0).setPersistent();
    public static final NTEntry<Double> kIndexerKp = new NTDouble("Indexer/PID/kP", 0.3).setPersistent();
    public static final NTEntry<Double> kIndexerKv = new NTDouble("Indexer/PID/kV", 0.13).setPersistent();
    public static final NTEntry<Double> kIndexerFeedVoltage = new NTDouble("Indexer/Voltages/Feed Loop", 8.0).setPersistent();
    public static final NTEntry<Double> kIndexerReverseVoltage = new NTDouble("Indexer/Voltages/Reverse Loop", -8.0).setPersistent();

    // Telemetry Diagnostics
    public static NTEntry<Double> currentAngle = new NTDouble("Drive/Auto/Test/current", 0);
    public static NTEntry<Double> targetAngle = new NTDouble("Drive/Auto/Test/target", 0);

    

    private Constants() {
        throw new UnsupportedOperationException("This is a constant data file and cannot be instantiated");
    }
}