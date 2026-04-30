package com.swrobotics.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.control.AimCalc;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class TurretSubsystem extends SubsystemBase {
    public enum TurretMode { SCORING, IDLE }

    private static final double TURRET_GEAR_TEETH = 200.0;
    private static final double MOTOR_GEAR_TEETH = 40.0;
    private static final double CANCODER_GEAR_TEETH = 43.0;
    private static final double MOTOR_TO_TURRET_RATIO = TURRET_GEAR_TEETH / MOTOR_GEAR_TEETH;

    private static final double TURRET_ZERO_OFFSET_ROT = 0.0; 

    private static final double IDLE_POS_ROT = 270.0 / 360.0;
    private static final double TURRET_MIN_ROT = (270.0 - 165.0) / 360.0;
    private static final double TURRET_MAX_ROT = (270.0 + 165.0) / 360.0;
    private static final double ON_TARGET_TOLERANCE_ROT = 1.0 / 360.0;

    private final TalonFX turretX44;
    private final CANcoder cancoder;
    private final SwerveDriveSubsystem drive;
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
    private TurretMode currentMode = TurretMode.SCORING;

    public TurretSubsystem(SwerveDriveSubsystem drive) {
        this.drive = drive;
        
        turretX44 = IOAllocation.CAN.kTurretMotor.createTalonFX();
        // Assuming your IOAllocation has a CANcoder definition. If not, replace with `new CANcoder(ID, "bus");`
        cancoder = IOAllocation.CAN.kTurretCANcoder.createCANcoder(); 

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kP = 18.0; // Snappier turret response
        config.Slot0.kD = 0.3;
        config.MotionMagic.MotionMagicCruiseVelocity = 15.0;
        config.MotionMagic.MotionMagicAcceleration = 40.0;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TURRET_MAX_ROT * MOTOR_TO_TURRET_RATIO;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TURRET_MIN_ROT * MOTOR_TO_TURRET_RATIO;

        config.apply(turretX44);

        // Calculate and seed the absolute position on subsystem init
        seedAbsolutePosition();
    }

    /**
     * Uses the Chinese Remainder Theorem (CRT) to uniquely determine the absolute position 
     * of the 200t turret gear using the 40t motor internal encoder and 43t CANcoder.
     */
    private void seedAbsolutePosition() {
        // 1. Get fractional rotations [0, 1) of both the motor and CANcoder gears
        double r1 = (turretX44.getPosition().getValueAsDouble() % 1.0 + 1.0) % 1.0;
        double r2 = (cancoder.getAbsolutePosition().getValueAsDouble() % 1.0 + 1.0) % 1.0;

        // 2. Calculate the alignment delta scaled to tooth counts
        long delta = Math.round(CANCODER_GEAR_TEETH * r2 - MOTOR_GEAR_TEETH * r1);

        // 3. Apply CRT modular multiplicative inverse step
        // We need 'm' such that (40m - 43k) = delta. 
        // 14 is the modular inverse of 40 modulo 43.
        long m = (14 * delta) % (long) CANCODER_GEAR_TEETH;
        if (m < 0) {
            m += (long) CANCODER_GEAR_TEETH;
        }

        // 4. Calculate absolute rotations of the motor gear
        double absoluteMotorRotations = r1 + m;

        // 5. Convert up to absolute turret rotations
        double absoluteTurretRotations = absoluteMotorRotations * (MOTOR_GEAR_TEETH / TURRET_GEAR_TEETH);

        // Apply physical mounting offset
        absoluteTurretRotations -= TURRET_ZERO_OFFSET_ROT;

        // 6. Wrap into our physically allowed domain.
        // The CRT cycle for 40t/43t/200t is exactly 8.6 turret rotations (1720 / 200).
        double cycleRotations = (MOTOR_GEAR_TEETH * CANCODER_GEAR_TEETH) / TURRET_GEAR_TEETH;
        absoluteTurretRotations = (absoluteTurretRotations % cycleRotations + cycleRotations) % cycleRotations;

        // Shift it iteratively to fall inside our physical soft limit window
        while (absoluteTurretRotations < TURRET_MIN_ROT) {
            absoluteTurretRotations += cycleRotations;
        }
        while (absoluteTurretRotations > TURRET_MAX_ROT && (absoluteTurretRotations - cycleRotations) >= TURRET_MIN_ROT) {
            absoluteTurretRotations -= cycleRotations;
        }

        // Seed the TalonFX internal encoder
        turretX44.setPosition(absoluteTurretRotations * MOTOR_TO_TURRET_RATIO);
    }

    @Override
    public void periodic() {
        switch (currentMode) {
            case SCORING:
                setControlWithWrap(AimCalc.getInstance().getTurretAimAngle());
                break;
            case IDLE:
            default:
                setControlWithWrap(Rotation2d.fromRotations(IDLE_POS_ROT));
                break;
        }
        
        UpdateTurretAngle();
    }

    private void setControlWithWrap(Rotation2d targetAngle) {
        double currentTurretRot = turretX44.getPosition().getValueAsDouble() / MOTOR_TO_TURRET_RATIO;
        double error = MathUtil.inputModulus(targetAngle.getRotations() - currentTurretRot, -0.5, 0.5);
        double targetTurretRot = currentTurretRot + error;

        double finalTarget = MathUtil.clamp(targetTurretRot, TURRET_MIN_ROT, TURRET_MAX_ROT);
        
        // Feedforward to compensate for robot spinning
        double chassisYawRateRotPerSec = drive.getRobotRelativeSpeeds().omegaRadiansPerSecond / (2.0 * Math.PI);

        turretX44.setControl(positionControl
            .withPosition(finalTarget * MOTOR_TO_TURRET_RATIO)
            .withFeedForward(-chassisYawRateRotPerSec * MOTOR_TO_TURRET_RATIO));
    }

    public Command cmdScoring() { return runOnce(() -> currentMode = TurretMode.SCORING); }
    public Command cmdIdle() { return runOnce(() -> currentMode = TurretMode.IDLE); }

    public boolean isOnTarget() {
        double currentTurretRot = turretX44.getPosition().getValueAsDouble() / MOTOR_TO_TURRET_RATIO;
        double error = MathUtil.inputModulus(AimCalc.getInstance().getTurretAimAngle().getRotations() - currentTurretRot, -0.5, 0.5);
        return Math.abs(error) < ON_TARGET_TOLERANCE_ROT;
    }

    private void UpdateTurretAngle() {
        SmartDashboard.putNumber("Turret/targetAngle", AimCalc.getInstance().getTurretAimAngle().getDegrees());
        SmartDashboard.putNumber("Turret/currentAngle", turretX44.getPosition().getValueAsDouble() / MOTOR_TO_TURRET_RATIO * 360.0);
    }
}