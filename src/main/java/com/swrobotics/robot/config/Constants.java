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
import com.swrobotics.lib.ctre.NTSlot0Configs;
import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.subsystems.swerve.SwerveModuleInfo;
import com.swrobotics.robot.subsystems.vision.LimelightCamera;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

// Use NTEntry when you want tunable
// Use double when value has been tuned in so it can't accidentally change
public final class Constants {
    public static final int kPeriodicFreq = 50; // Hz
    public static final double kPeriodicTime = 1.0 / kPeriodicFreq;

    public static final FieldInfo kField = FieldInfo.REBUILT_2026;
    public static final Pose2d kHubPose = new Pose2d(8.267, 4.105, new Rotation2d(0));
    public static final int kEndgameAlertTime = 20;
    public static final int kEndgameAlert2Time = 5;
    public static final int kTransferAlertTime = 130;//TODO: Set transfer alert time later
    public static final int kActive_InactiveAlert1Time = 120;//TODO: set active/inactive alert time later
    public static final int kActive_InactiveAlert1Time2 = 115;//TODO: set active/inactive alert time later
    public static final int kActive_InactiveAlert2Time = 95;//TODO: set active/inactive alert time later
    public static final int kActive_InactiveAlert2Time2 = 90;//TODO: set active/inactive alert time later
    public static final int kActive_InactiveAlert3Time = 70;//TODO: set active/inactive alert time later
    public static final int kActive_InactiveAlert3Time2 = 65;//TODO: set active/inactive alert time later
    public static final int kActive_InactiveAlert4Time = 35; //TODO: set active/inactive alert time later
    public static final int kActive_InactiveAlert4Time2 = 30; //TODO: set active/inactive alert time later

    // Robot dimensions
    public static final double kFrameLength = Units.inchesToMeters(27.25); // m
    public static final double kFrameWidth = Units.inchesToMeters(27.25); // m
    public static final double kHubHeightMeters = Units.inchesToMeters(49.5); // m
    public static final double kShooterHeightMeters = Units.inchesToMeters(20); //TODO: Measure inches

    public static final double kRobotMass = Units.lbsToKilograms(135); // TODO: Measure
    // Approximation of robot as uniform cuboid
    // See https://sleipnirgroup.github.io/Choreo/usage/estimating-moi/
    public static final double kRobotMOI = 1.0/12.0 * kRobotMass * (kFrameLength*kFrameLength + kFrameWidth*kFrameWidth);

    // TEMP
    public static NTEntry<Double> currentAngle = new NTDouble("Drive/Auto/Test/current", 0);
    public static NTEntry<Double> targetAngle = new NTDouble("Drive/Auto/Test/target", 0);

    // Controls
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kSuperControllerPort = 2;

    public static final double kDeadband = 0.15;
    public static final double kTriggerThreshold = 0.3;

    public static final double kDriveControlMaxAccel = 3.5; // m/s^2
    public static final double kDriveControlMaxTurnSpeed = 1; // rot/s
    public static final double kDriveControlDrivePower = 2; // Exponent input is raised to
    public static final double kDriveControlTurnPower = 2;

    // Auto (TODO: Tune)
    public static final double kAutoDriveKp = 4;
    public static final double kAutoDriveKd = 0;
    public static final NTEntry<Double> kAutoTurnKp = new NTDouble("Drive/Auto/Turn PID/kP", 5).setPersistent();
    public static final NTEntry<Double> kAutoTurnKd = new NTDouble("Drive/Auto/Turn PID/kD", 0).setPersistent();

    public static final NTEntry<Double> kSnapMaxSpeed = new NTDouble("Drive/Snap/Max Speed (meters per sec)", 10).setPersistent();
    public static final NTEntry<Double> kSnapMaxTurnSpeed = new NTDouble("Drive/Snap/Max Turn Speed (rot per sec)", 3.5).setPersistent();
    public static final NTEntry<Double> kSnapDriveKp = new NTDouble("Drive/Snap/Drive kP", 2).setPersistent();
    public static final NTEntry<Double> kSnapDriveKd = new NTDouble("Drive/Snap/Drive kD", 0.2).setPersistent();
    public static final NTEntry<Double> kSnapTurnKp = new NTDouble("Drive/Snap/Turn kP", 4).setPersistent();
    public static final NTEntry<Double> kSnapTurnKd = new NTDouble("Drive/Snap/Turn kD", 0).setPersistent();
    public static final NTEntry<Double> kSnapXYDeadzone = new NTDouble("Drive/Snap/XY Deadzone (m)", 0.005).setPersistent();
    public static final NTEntry<Double> kSnapThetaDeadzone = new NTDouble("Drive/Snap/Theta Deadzone (deg)", 0.2).setPersistent();


    // Drive
    public static final double kDriveMaxAchievableSpeed = Units.feetToMeters(18.9); // m/s  TODO: Measure

    public static final double kOdometryUpdateFreq = 200; // Hz
    public static final Matrix<N3, N1> kOdometryStdDevs = VecBuilder.fill(0.005, 0.005, 0.001);

    public static final double kDriveStatorCurrentLimit = 60; // A
    public static final double kDriveSupplyCurrentLimit = 40; // A
    public static final double kDriveCurrentLimitTime = 0.25; // sec

    public static final double kDriveWheelCOF = 1.2; // TODO: Measure?

    public static final double kDriveWheelSpacingX = 63.0 / 100; // m
    public static final double kDriveWheelSpacingY = 55.3 / 100; // m
    public static final double kDriveRadius = Math.hypot(kDriveWheelSpacingX / 2, kDriveWheelSpacingY / 2);

    public static final NTEntry<Double> kFrontLeftOffset = new NTDouble("Drive/Modules/Front Left Offset (rot)", -0.33935546875).setPersistent();
    public static final NTEntry<Double> kFrontRightOffset = new NTDouble("Drive/Modules/Front Right Offset (rot)", 0.323486328125).setPersistent();
    public static final NTEntry<Double> kBackLeftOffset = new NTDouble("Drive/Modules/Back Left Offset (rot)", -0.320556640625).setPersistent();
    public static final NTEntry<Double> kBackRightOffset = new NTDouble("Drive/Modules/Back Right Offset (rot)", -0.367431640625).setPersistent();
    public static final SwerveModuleInfo[] kSwerveModuleInfos = {
            new SwerveModuleInfo(IOAllocation.CAN.kSwerveFL, kDriveWheelSpacingX / 2, kDriveWheelSpacingY / 2, Constants.kFrontLeftOffset, "Front Left"),
            new SwerveModuleInfo(IOAllocation.CAN.kSwerveFR, kDriveWheelSpacingX / 2, -kDriveWheelSpacingY / 2, Constants.kFrontRightOffset, "Front Right"),
            new SwerveModuleInfo(IOAllocation.CAN.kSwerveBL, -kDriveWheelSpacingX / 2, kDriveWheelSpacingY / 2, Constants.kBackLeftOffset, "Back Left"),
            new SwerveModuleInfo(IOAllocation.CAN.kSwerveBR, -kDriveWheelSpacingX / 2, -kDriveWheelSpacingY / 2, Constants.kBackRightOffset, "Back Right")
    };

    public static final SwerveDrivetrainConstants kDrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(IOAllocation.CAN.kSwerveBus)
            .withPigeon2Id(IOAllocation.CAN.kJosh.id())
            .withPigeon2Configs(new Pigeon2Configuration());
    public static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kModuleConstantsFactory =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio((50.0/16) * (16.0/28) * (45.0/15))
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

    public static final RobotConfig kPathPlannerRobotConfig = new RobotConfig(
            Constants.kRobotMass,
            Constants.kRobotMOI,
            new ModuleConfig(
                    Constants.kModuleConstantsFactory.WheelRadius,
                    Constants.kDriveMaxAchievableSpeed,
                    Constants.kDriveWheelCOF,
                    DCMotor.getKrakenX60Foc(1).withReduction(Constants.kModuleConstantsFactory.DriveMotorGearRatio),
                    Constants.kDriveStatorCurrentLimit,
                    1
            ),
            kSwerveModuleInfos[0].position(),
            kSwerveModuleInfos[1].position(),
            kSwerveModuleInfos[2].position(),
            kSwerveModuleInfos[3].position()
    );

    // Vision
    public static final double kVisionMT2SpeedThreshold = 0.2; // m/s

    // This will be different for each lens type, cameras with same lens should
    // have the same config
    public static final LimelightCamera.Config kLimelightConfig = new LimelightCamera.Config(
            // These were tuned at MURA using red alliance speaker AprilTags
            2,
            0.00197,
            0.002,
            0.00117
    );

    public static final LimelightCamera.MountingLocation kLimelightBackLocation = new LimelightCamera.MountingLocation(
        Units.inchesToMeters(13), Units.inchesToMeters(-10.673), Units.inchesToMeters(17.558), //z x y? jonah lowkey weird
        // Degrees CCW
        0, 0, 180
    );

    public static final LimelightCamera.MountingLocation kLimelightRightLocation = new LimelightCamera.MountingLocation(
        Units.inchesToMeters(10.05),Units.inchesToMeters(-12.3503), Units.inchesToMeters(9.502),
        // Degrees CCW
        0, 0, 270
    );

    public static final LimelightCamera.MountingLocation kLimelightFrontLocation = new LimelightCamera.MountingLocation(
        // TODO: These are guesses, they should be measured in CAD
        Units.inchesToMeters(-4.84), Units.inchesToMeters(-10.213), Units.inchesToMeters(18.132),
        0, 15, 0
    );

   /* --- Hood --- */ //TODO: Adjust constant values
    public static final NTEntry<Boolean> kHoodInverted = new NTBoolean("Shooter/Hood/Inverted", false).setPersistent();
    public static final NTEntry<Double> kHoodMaxAngle = new NTDouble("Shooter/Hood/Max Angle (Deg)", 25.0).setPersistent();
    public static final NTEntry<Double> kHoodMinAngle = new NTDouble("Shooter/Hood/Min Angle (Deg)", 10.0).setPersistent();
    public static final NTEntry<Double> kHoodCruiseVelocity = new NTDouble("Shooter/Hood/Cruise Velocity", 60.0).setPersistent();
    public static final NTEntry<Double> kHoodAcceleration = new NTDouble("Shooter/Hood/Acceleration", 120.0).setPersistent();

    public static final NTSlot0Configs kHoodPID = new NTSlot0Configs("Shooter/Hood/PID", 6.0, 0.8, 0.0, 0.0, 0.1, 0.0);

    

    /* --- Climber ---  */ //TODO: Add climber constants here later
        // Logic for 16.875:1 Gear Ratio
        public static final NTEntry<Double> kClimberTall = new NTDouble("Climber/Height", 45.0).setPersistent(); 
        public static final NTEntry<Double> kClimberCalibrationTime = new NTDouble("Climber/CalibrationTime", 0.2).setPersistent();
        public static final NTEntry<Double> kClimberCalibrationVelocity = new NTDouble("Climber/CalibrationVelocity", 0.05).setPersistent(); 
        public static final NTEntry<Double> kClimberCalibrationVoltage = new NTDouble("Climber/CalibrationVoltage", 2.5).setPersistent();
        public static final NTEntry<Double> kClimberCalibrationPosition = new NTDouble("Climber/Calibration Position", 0.0).setPersistent();
        // With a 16.875 ratio, 1 rotation of the output is 16.875 rotations of the motor.
        // We want a strong P-gain to hold 130lbs. 
        public static final NTSlot0Configs kClimberPID = new NTSlot0Configs("Climber/PID", 15.0, 0.0, 0.2, 0.0, 0.0, 0.0);

    /* --- Expansion ---  */ 
    public static final NTEntry<Double> kExpansionRetractedRotations = new NTDouble("Intake/Expansion/Retracted Rotations", 0).setPersistent();
    public static final NTEntry<Double> kExpansionExtendedRotations  = new NTDouble("Intake/Expansion/Extended Rotations", 10.0).setPersistent();
    public static final NTEntry<Double> kExpansionCruiseVelocity = new NTDouble("Intake/Expansion/Cruise Velocity", 40.0).setPersistent();   
    public static final NTEntry<Double> kExpansionAcceleration  = new NTDouble("Intake/Expansion/Acceleration", 160.0).setPersistent();
    public static final NTSlot0Configs kExpansionPID = new NTSlot0Configs("Intake/Expansion/PID", 3.0, 0.001, 0.01, 0.0, 0.0, 0.0);
    
    /* --- Indexer --- */
    public static final NTEntry<Double> kIndexerRollVoltage = new NTDouble("Intake/Indexer/Intake Voltage", 10.0).setPersistent();
    public static final NTEntry<Double> kIndexerIdleVoltage = new NTDouble("Intake/Indexer/Idle Voltage", 0.0).setPersistent();
    public static final NTEntry<Double> kIndexerHoldVoltage = new NTDouble("intake/Indexer/Hold Voltage", 10.0).setPersistent();
    
    /* --- Intake  ---  */ 
    public static final NTEntry<Double> kIntakeVoltage= new NTDouble("Intake/Intake Voltage", 6.0).setPersistent();
    public static final NTEntry<Double> kIntakeIdleVoltage = new NTDouble("Intake/Idle Voltage", 0.0).setPersistent();
    public static final NTSlot0Configs kIntakePID = new NTSlot0Configs("Intake/PID", 1.0, 0, 0, 0, 0, 0 );


    /* --- Shooter --- */
    public static final NTEntry<Double> kShooterRPS = new NTDouble("Shooter/Intake RPS", 40).setPersistent();
    public static final NTEntry<Double> kShooterIdleRPS = new NTDouble("Shooter/Idle RPS", 0.0).setPersistent();
    public static final NTEntry<Double> kShooterWarmRPS = new NTDouble("Shooter/Warm RPS", 1.0).setPersistent();
    public static final NTEntry<Double> kShooterRindexRPS = new NTDouble("Shooter/Rindex RPS", -6.0).setPersistent();
    public static final NTSlot0Configs kShooterPID = new NTSlot0Configs("Shooter/PID", 3.0, 0.000, 0.0, 0.0, 0.13, 0.53);
    
   
}
